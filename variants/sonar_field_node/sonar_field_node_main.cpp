#include <Arduino.h>   // needed for PlatformIO
#include <Wire.h>      // TWI handle, so sleep can release the bus (SLEEP_RELEASE_TWI)
#include <Mesh.h>

#if defined(NRF52_PLATFORM)
  #include <InternalFileSystem.h>
#elif defined(RP2040_PLATFORM)
  #include <LittleFS.h>
#elif defined(ESP32)
  #include <SPIFFS.h>
#endif

#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/IdentityStore.h>
#include <RTClib.h>
#include <target.h>

extern RADIO_CLASS radio;   // RadioLib radio object (target.cpp) -- for getDeviceErrors()

/* ---------------------------------- CONFIGURATION ------------------------------------- */

#define FIRMWARE_VER_TEXT   "sonar_field_node (build: Jul 22 2026, v3.1 gateA fixes) [v3-ultrasonic + MaxBotix MB7388 -> distance_meters; duty-cycle mode machine; link-health LED (send=toggle, ACK=off, timeout=on, off on sleep) + STATUS tx-fail count]"

#ifndef LORA_FREQ
  #define LORA_FREQ   910.525
#endif
#ifndef LORA_BW
  #define LORA_BW     62.5
#endif
#ifndef LORA_SF
  #define LORA_SF     7
#endif
#ifndef LORA_CR
  #define LORA_CR      5
#endif
#ifndef LORA_TX_POWER
  #define LORA_TX_POWER  20
#endif

#ifndef MAX_CONTACTS
  #define MAX_CONTACTS   32
#endif

#ifndef ULTRASONIC_BAUD
  #define ULTRASONIC_BAUD  9600
#endif

// How long to wait for a fresh MaxBotix frame (continuous mode is ~6 Hz, so
// any wait > ~170 ms should hit at least one frame).
#ifndef ULTRASONIC_READ_TIMEOUT_MS
  #define ULTRASONIC_READ_TIMEOUT_MS  400
#endif

#ifndef ADVERT_NAME
  #define ADVERT_NAME  "Sensor"
#endif

#ifndef SENSOR_SEND_INTERVAL_SECS
  #define SENSOR_SEND_INTERVAL_SECS  300
#endif

#include <helpers/BaseChatMesh.h>

#define SEND_TIMEOUT_BASE_MILLIS          500
#define FLOOD_SEND_TIMEOUT_FACTOR         16.0f
#define DIRECT_SEND_PERHOP_FACTOR         6.0f
#define DIRECT_SEND_PERHOP_EXTRA_MILLIS   250

#define TARGET_PREFIX_LEN  6
#ifndef MAX_SEND_TARGETS
  #define MAX_SEND_TARGETS  4
#endif

/* ---------------------------------- DISPLAY PAGES ------------------------------------- */

enum DisplayPage {
  PAGE_STATUS = 0,   // Shows distance, battery, target, interval
  PAGE_SEND,         // "Send sensor" -- long press to send
  PAGE_ADVERT,       // "Send advert" -- long press to broadcast
  PAGE_COUNT
};

#define DISPLAY_REFRESH_MS    500
#define DISPLAY_AUTO_OFF_MS   30000
#define LONG_PRESS_MILLIS     1200
#define ALERT_DURATION_MS     1500

// MB7388 pin 4 (Ranging Start/Stop) wired to Arduino D1 = P0.06, reclaimed from
// Serial1 TX (no TX wire needed). Commanded ranging: hold high to range, low to
// stop. Serial1.begin() otherwise owns D1 as UART TXD idling HIGH -> the sensor
// free-runs and backlogs unsolicited frames, so setup() detaches TXD via PSEL.
#define PIN_STROBE   1
#ifndef SONAR_SAMPLES
#define SONAR_SAMPLES  7   // frames to collect per reading; median rejects surface chop
#endif

// --- power knobs, each separately measurable ------------------------------------
// Default 0 so the default build stays exactly the configuration measured at
// 1.066 mA on 2026-07-18. Each gets its own env so its effect is attributable
// before it is adopted: measure first, then make it the default.
//
// PWR_ENABLE_DCDC: RookBoard derives from NRF52BoardDCDC, but RookBoard::begin()
// calls NRF52Board::begin() -- the GRANDparent -- so NRF52BoardDCDC::begin() never
// runs and the DC/DC enable is silently skipped. The board has been running on the
// LDO. Enabling DC/DC on an nRF52840 typically saves 30-40% of supply current.
// Done app-side rather than by patching Don's RookBoard.cpp; worth reporting
// upstream as a board-support bug either way.
#ifndef PWR_ENABLE_DCDC
#define PWR_ENABLE_DCDC 0
#endif

// SLEEP_RELEASE_TWI: Wire.begin() runs once in RookBoard::begin() and is never
// ended, so TWIM stays enabled across sleep. Same class of leak as the UART.
#ifndef SLEEP_RELEASE_TWI
#define SLEEP_RELEASE_TWI 0
#endif

// Compile knob for the attribution A/B. 1 (default) releases UARTE0 across sleep;
// 0 leaves it enabled the way the pre-2026-07-18 firmware did, which is what the
// measured 6.6 mA floor came from. Build the paired env to reproduce that arm
// rather than passing an ephemeral -D, so the comparison stays replicable.
#ifndef SLEEP_RELEASE_UART
#define SLEEP_RELEASE_UART 1
#endif

// SLEEP_PIN_DISCONNECT_AUDIT: during sleep, put the explicitly listed non-radio
// pins into the nRF52's buffer-disconnected reset state (PIN_CNF INPUT=Disconnect
// via nrf_gpio_cfg_default) instead of whatever floating/input state the sleep
// path leaves them in. pinMode(INPUT) keeps the input buffer connected, and a
// floating buffer near threshold conducts. Restore contract: each pin's own
// bring-up in MODE_WAKE rewrites its PIN_CNF (sonar_serial_begin -> strobe
// OUTPUT + Serial1 RX; Wire.begin -> SDA/SCL), so no explicit reconnect step
// exists to forget. The list is closed-form on purpose -- only pins whose wake
// path provably reconfigures them. Sonar gate (D5/P0.24) stays a driven OUTPUT.
#ifndef SLEEP_PIN_DISCONNECT_AUDIT
#define SLEEP_PIN_DISCONNECT_AUDIT 0
#endif

// SLEEP_DISABLE_USBD: turn the USB peripheral off during battery sleep.
//
// The Adafruit core calls TinyUSB_Device_Init unconditionally, so USBD stays
// enabled on battery with no host attached. An enabled USBD holds the HFCLK and
// is the classic signature of a ~1 mA nRF52840 sleep floor. Only fires when
// USBREGSTATUS reports no VBUS, so every USB-attached bench flow is untouched.
// One-way on battery: replugging USB later needs a reset to re-enumerate --
// acceptable for a field node, and stated wherever this knob gets adopted.
#ifndef SLEEP_DISABLE_USBD
#define SLEEP_DISABLE_USBD 0
#endif

// SLEEP_FPU_CLEAR: apply the nRF52840 Errata-87 workaround before each sleep
// poll. Float math latches an FPSCR exception bit which keeps the FPU IRQ
// pending; a pending IRQ makes WFE fall straight through, so the FreeRTOS idle
// task spins instead of sleeping. QT1 (2026-07-20) measured fpu_pend=1 at
// every sleep entry on this firmware, so this is live, not theoretical.
#ifndef SLEEP_FPU_CLEAR
#define SLEEP_FPU_CLEAR 0
#endif

// --- bisection strip knobs (cumulative, app layer only) ---
// Each STRIP_* removes one stack layer so adjacent JS220 floors attribute its
// cost. Ladder: S1 display -> S2 sonar -> S3 filesystem -> S4 mesh (radio goes
// RadioLib-direct: init then immediate warm sleep; loop is a bare heartbeat).
// All default 0 -- the deployment build is untouched.
#ifndef STRIP_DISPLAY
#define STRIP_DISPLAY 0
#endif
#ifndef STRIP_SONAR
#define STRIP_SONAR 0
#endif
#ifndef STRIP_FS
#define STRIP_FS 0
#endif
#ifndef STRIP_MESH
#define STRIP_MESH 0
#endif
// HEARTBEAT: '[HB] <millis>' each sleep poll -- the alive-check for stripped
// arms that cannot TX/ACK.
#ifndef HEARTBEAT
#define HEARTBEAT 0
#endif

#if STRIP_DISPLAY
// turnOn() re-inits the panel (SSD1306Display.cpp:24), so a begin() guard alone
// is not enough -- stub the on/off calls at their sites.
#define DISPLAY_TURN_ON()  ((void)0)
#define DISPLAY_TURN_OFF() ((void)0)
#else
#define DISPLAY_TURN_ON()  display.turnOn()
#define DISPLAY_TURN_OFF() display.turnOff()
#endif

// SLEEP_HFCLK_STOP: stop the HFXO during battery sleep. QT1 showed the 64 MHz
// crystal RUNNING through sleep; disabling USBD alone did not release it.
// Suspected mechanism: the node always boots on USB, the USB bring-up requests
// HFCLK, and nothing releases the request when VBUS goes away mid-run -- the
// release rides a power event no task processes on battery. Rather than chase
// the requester, stop the clock at sleep entry when VBUS is absent. Peripherals
// that need HF later fall back to HFINT on demand; the SX1262 runs its own
// TCXO and does not care about the MCU crystal.
#ifndef SLEEP_HFCLK_STOP
#define SLEEP_HFCLK_STOP 0
#endif

// SLEEP_DIAG: print the power-relevant machine state at each sleep entry
// (SoftDevice enabled? USBD enabled? HFCLK source+state? FPU IRQ pending?
// RXEN/NSS levels). Costs a few ms of serial once per cycle; bench-only knob.
#ifndef SLEEP_DIAG
#define SLEEP_DIAG 0
#endif

#if SLEEP_DIAG
static void sleep_diag_dump() {
  // Boot-entry guard: the mode machine STARTS in MODE_SLEEP, so this runs in the
  // first instants of boot -- and reading USBD registers before the USB power
  // domain finishes sequencing bus-faults the core (node dies before CDC ever
  // enumerates; cost us three invisible flashes). Skip the boot entry; the
  // first informative dump is after a real wake/TX cycle anyway. Same reason
  // there is no sd_softdevice_is_enabled() SVC here.
  static bool first_entry = true;
  if (first_entry) { first_entry = false; return; }
  Serial.print("[DIAG] usbd_en="); Serial.print(NRF_USBD->ENABLE);
  Serial.print(" vbus="); Serial.print((NRF_POWER->USBREGSTATUS & 1) ? 1 : 0);
  Serial.print(" hfclkstat=0x"); Serial.print(NRF_CLOCK->HFCLKSTAT, HEX);
  Serial.print(" fpu_pend="); Serial.print(NVIC_GetPendingIRQ(FPU_IRQn));
  Serial.print(" rxen="); Serial.print(digitalRead(SX126X_RXEN));
  Serial.print(" nss="); Serial.println(digitalRead(P_LORA_NSS));
}
#endif

#if SLEEP_PIN_DISCONNECT_AUDIT
#include "nrf_gpio.h"
static void sleep_pins_disconnect() {
  nrf_gpio_cfg_default(g_ADigitalPinMap[PIN_STROBE]);       // D1/P0.06: sonar_serial_end left it INPUT (buffer on)
  nrf_gpio_cfg_default(g_ADigitalPinMap[PIN_SERIAL1_RX]);   // D0/P0.08: sonar RX pad after UARTE release
#if SLEEP_RELEASE_TWI
  nrf_gpio_cfg_default(g_ADigitalPinMap[PIN_WIRE_SDA]);     // I2C pads only when TWIM is truly ended
  nrf_gpio_cfg_default(g_ADigitalPinMap[PIN_WIRE_SCL]);
#endif
}
#endif

// --- sonar serial lifecycle (deep-idle support) --------------------------------
// UARTE0 is a power domain of its own: left enabled it holds the HFCLK running and
// costs on the order of a milliamp even with no traffic, which puts a floor under
// any System-ON idle. So the sleep path must DISABLE it, not merely stop reading.
//
// Every Serial1.begin() re-attaches UARTE TXD to D1, so the PSEL detach has to be
// re-applied here on each bring-up or the sensor free-runs and backlogs unsolicited
// frames (an earlier bug). Keeping both halves in one place is what stops the
// wake path from silently forgetting it.
static void sonar_serial_begin() {
#if STRIP_SONAR
  return;                              // bisection arm: sonar layer removed
#endif
  Serial1.begin(ULTRASONIC_BAUD);
  NRF_UARTE0->PSEL.TXD = 0xFFFFFFFF;   // reclaim D1 from UART TX -> GPIO strobe
  pinMode(PIN_STROBE, OUTPUT);
  digitalWrite(PIN_STROBE, LOW);       // ranging stopped until a read commands it
}

// Release UARTE0 and both sonar pads. Called BEFORE gating the sensor off: an
// active UART RX pad sneak-loads the MB7388's output while its ground floats,
// which is the loading path chased earlier.
static void sonar_serial_end() {
#if STRIP_SONAR
  return;                              // bisection arm: sonar layer removed
#endif
  Serial1.end();
  NRF_UARTE0->ENABLE = 0;              // ensure the peripheral is truly off, not idle
  pinMode(PIN_STROBE, INPUT);          // hi-Z: never drive into an unpowered sensor
}

// median of the first n ints in a[] (n small; insertion sort in place)
static int median_int(int *a, int n) {
  for (int i = 1; i < n; i++) {
    int v = a[i], j = i - 1;
    while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
    a[j + 1] = v;
  }
  return a[n / 2];
}

// Believe it or not, this std C function is busted on some platforms!
static uint32_t _atoi(const char* sp) {
  uint32_t n = 0;
  while (*sp && *sp >= '0' && *sp <= '9') {
    n *= 10;
    n += (*sp++ - '0');
  }
  return n;
}

/* -------------------------------------------------------------------------------------- */

// MaxBotix MB7388 sensor globals.
// Reads ASCII frames of the form "Rxxxx\r" on Serial1 (9600 baud, TTL).
// MB7388 free-runs at ~6 Hz when its pin 4 (RX/strobe) is left floating or HIGH.
// xxxx is millimeters; range 300 mm - 5000 mm. 0 / out-of-range readings are
// reported by the sensor itself (e.g. "R5000" for max-out).
static bool has_sensor = false;
static unsigned long sensor_last_frame_at = 0;

// Cached sensor readings (updated each send interval)
static float last_dist_m = 0;     // meters, parsed from MaxBotix
static float last_batt = 0;
static bool has_reading = false;

// v3: last-send ACK status -- surfaced on the OLED status page so a button-triggered
// send shows whether the receiver actually acknowledged.
enum AckStatus {
  ACK_STATUS_NONE = 0,
  ACK_STATUS_PENDING,
  ACK_STATUS_OK,
  ACK_STATUS_TIMEOUT
};

class MyMesh : public BaseChatMesh, ContactVisitor {
  FILESYSTEM* _fs;
  uint32_t expected_ack_crc;
  unsigned long last_msg_sent;
  char command[512];
  uint8_t tmp_buf[256];
  char hex_buf[512];

  // Target contacts (6-byte pub key prefixes, persisted to flash)
  uint8_t target_prefixes[MAX_SEND_TARGETS][TARGET_PREFIX_LEN];
  uint8_t target_count;

  // Node config
  char node_name[32];
  uint32_t send_interval_secs;
  uint32_t last_send_time;  // RTC timestamp of last send
  uint16_t node_id;

  // Last send result for display feedback
  uint8_t last_send_successes;
  uint8_t last_send_attempts;

  // v2: remember which target prefix was used for the most recent direct send,
  // so onSendTimeout can invalidate its cached path. 0xFF = none pending.
  uint8_t last_ack_target_idx;

  // v3: ACK status of most recent send, displayed on OLED status page.
  AckStatus last_ack_status;
  uint32_t last_ack_rt_ms;            // round-trip ms of the most recent ACK (serial log only)
  unsigned long last_ack_at_millis;   // millis() when the most recent OK ACK arrived

  // link-health LED (P0.15). Rules: a send TOGGLES it (packet in flight), a
  // positive ACK forces it OFF (healthy), a timeout forces it ON (problem). The
  // mode machine forces it OFF on MODE_SLEEP entry so a fully-failed transmit
  // never leaves the lamp lit into sleep (power).
  bool led_on;
  void ledWrite(bool on) {
    led_on = on;
    digitalWrite(LED_PIN, (on == (LED_STATE_ON != 0)) ? HIGH : LOW);
  }
  void ledToggle() { ledWrite(!led_on); }

  // last-transmit outcome, surfaced on the STATUS page. Set by the mode machine
  // when a MODE_TRANSMIT sequence ends: failed=true means all attempts missed.
  bool last_tx_failed;
  uint8_t last_tx_attempts;           // sends attempted in that last transmit

  void loadContacts() {
    if (_fs->exists("/contacts")) {
    #if defined(RP2040_PLATFORM)
      File file = _fs->open("/contacts", "r");
    #else
      File file = _fs->open("/contacts");
    #endif
      if (file) {
        bool full = false;
        while (!full) {
          ContactInfo c;
          uint8_t pub_key[32];
          uint8_t unused;
          uint32_t reserved;

          bool success = (file.read(pub_key, 32) == 32);
          success = success && (file.read((uint8_t *) &c.name, 32) == 32);
          success = success && (file.read(&c.type, 1) == 1);
          success = success && (file.read(&c.flags, 1) == 1);
          success = success && (file.read(&unused, 1) == 1);
          success = success && (file.read((uint8_t *) &reserved, 4) == 4);
          success = success && (file.read((uint8_t *) &c.out_path_len, 1) == 1);
          success = success && (file.read((uint8_t *) &c.last_advert_timestamp, 4) == 4);
          success = success && (file.read(c.out_path, 64) == 64);
          c.gps_lat = c.gps_lon = 0;

          if (!success) break;  // EOF

          c.id = mesh::Identity(pub_key);
          c.lastmod = 0;
          if (!addContact(c)) full = true;
        }
        file.close();
      }
    }
  }

  void saveContacts() {
  #if defined(NRF52_PLATFORM)
    _fs->remove("/contacts");
    File file = _fs->open("/contacts", FILE_O_WRITE);
  #elif defined(RP2040_PLATFORM)
    File file = _fs->open("/contacts", "w");
  #else
    File file = _fs->open("/contacts", "w", true);
  #endif
    if (file) {
      ContactsIterator iter;
      ContactInfo c;
      uint8_t unused = 0;
      uint32_t reserved = 0;

      while (iter.hasNext(this, c)) {
        bool success = (file.write(c.id.pub_key, 32) == 32);
        success = success && (file.write((uint8_t *) &c.name, 32) == 32);
        success = success && (file.write(&c.type, 1) == 1);
        success = success && (file.write(&c.flags, 1) == 1);
        success = success && (file.write(&unused, 1) == 1);
        success = success && (file.write((uint8_t *) &reserved, 4) == 4);
        success = success && (file.write((uint8_t *) &c.out_path_len, 1) == 1);
        success = success && (file.write((uint8_t *) &c.last_advert_timestamp, 4) == 4);
        success = success && (file.write(c.out_path, 64) == 64);

        if (!success) break;
      }
      file.close();
    }
  }

  void loadTargets() {
    target_count = 0;

    // New multi-target format: /targets
    if (_fs->exists("/targets")) {
    #if defined(RP2040_PLATFORM)
      File file = _fs->open("/targets", "r");
    #else
      File file = _fs->open("/targets");
    #endif
      if (file) {
        uint8_t count = 0;
        if (file.read(&count, 1) == 1 && count <= MAX_SEND_TARGETS) {
          for (uint8_t i = 0; i < count; i++) {
            if (file.read(target_prefixes[i], TARGET_PREFIX_LEN) != TARGET_PREFIX_LEN) break;
            target_count++;
          }
        }
        file.close();
      }
      return;
    }

    // Migrate from old single-target format: /target
    if (_fs->exists("/target")) {
    #if defined(RP2040_PLATFORM)
      File file = _fs->open("/target", "r");
    #else
      File file = _fs->open("/target");
    #endif
      if (file) {
        if (file.read(target_prefixes[0], TARGET_PREFIX_LEN) == TARGET_PREFIX_LEN) {
          target_count = 1;
        }
        file.close();
      }
      if (target_count > 0) {
        saveTargets();  // write in new format
        _fs->remove("/target");
      }
    }
  }

  void saveTargets() {
  #if defined(NRF52_PLATFORM)
    _fs->remove("/targets");
    File file = _fs->open("/targets", FILE_O_WRITE);
  #elif defined(RP2040_PLATFORM)
    File file = _fs->open("/targets", "w");
  #else
    File file = _fs->open("/targets", "w", true);
  #endif
    if (file) {
      file.write(&target_count, 1);
      for (uint8_t i = 0; i < target_count; i++) {
        file.write(target_prefixes[i], TARGET_PREFIX_LEN);
      }
      file.close();
    }
  }

  int findTargetIndex(const uint8_t* prefix) {
    for (uint8_t i = 0; i < target_count; i++) {
      if (memcmp(target_prefixes[i], prefix, TARGET_PREFIX_LEN) == 0) return i;
    }
    return -1;
  }

  bool addTargetPrefix(const uint8_t* prefix) {
    if (findTargetIndex(prefix) >= 0) return true;  // already present
    if (target_count >= MAX_SEND_TARGETS) return false;
    memcpy(target_prefixes[target_count], prefix, TARGET_PREFIX_LEN);
    target_count++;
    saveTargets();
    return true;
  }

  bool removeTargetPrefix(const uint8_t* prefix) {
    int idx = findTargetIndex(prefix);
    if (idx < 0) return false;
    for (uint8_t i = idx; i < target_count - 1; i++) {
      memcpy(target_prefixes[i], target_prefixes[i+1], TARGET_PREFIX_LEN);
    }
    target_count--;
    saveTargets();
    return true;
  }

  void loadPrefs() {
    if (_fs->exists("/sensor_prefs")) {
    #if defined(RP2040_PLATFORM)
      File file = _fs->open("/sensor_prefs", "r");
    #else
      File file = _fs->open("/sensor_prefs");
    #endif
      if (file) {
        uint32_t interval;
        if (file.read((uint8_t*)&interval, 4) == 4) {
          send_interval_secs = interval;
        }
        char name[32];
        if (file.read((uint8_t*)name, 32) == 32 && name[0] != 0) {
          memcpy(node_name, name, 32);
        }
        uint16_t id;
        if (file.read((uint8_t*)&id, 2) == 2) {
          node_id = id;
        }
        file.close();
      }
    }
  }

  void savePrefs() {
  #if defined(NRF52_PLATFORM)
    _fs->remove("/sensor_prefs");
    File file = _fs->open("/sensor_prefs", FILE_O_WRITE);
  #elif defined(RP2040_PLATFORM)
    File file = _fs->open("/sensor_prefs", "w");
  #else
    File file = _fs->open("/sensor_prefs", "w", true);
  #endif
    if (file) {
      file.write((const uint8_t*)&send_interval_secs, 4);
      file.write((const uint8_t*)node_name, 32);
      file.write((const uint8_t*)&node_id, 2);
      file.close();
    }
  }

  void importCard(const char* command) {
    while (*command == ' ') command++;
    if (memcmp(command, "meshcore://", 11) == 0) {
      command += 11;
      char *ep = strchr(command, 0);
      while (ep > command) {
        ep--;
        if (mesh::Utils::isHexChar(*ep)) break;
        *ep = 0;
      }
      int len = strlen(command);
      Serial.printf("   hex len=%d\n", len);
      if (len % 2 == 0) {
        len >>= 1;
        if (mesh::Utils::fromHex(tmp_buf, len, command)) {
          if (importContact(tmp_buf, len)) {
            Serial.printf("   Advert queued. Contacts before: %d/%d\n", getNumContacts(), MAX_CONTACTS);
            Serial.println("   Wait for 'ADVERT from -> ...' to confirm actual add.");
          } else {
            Serial.println("   error: importContact failed (bad packet).");
          }
          return;
        }
      }
    }
    Serial.println("   error: invalid format");
  }

  // Drain Serial1 looking for the most recent complete MaxBotix frame
  // ("Rdddd\r"). Returns distance in meters, or -1.0 on no-frame / parse error.
  // We loop until ULTRASONIC_READ_TIMEOUT_MS elapses to make sure the value
  // we return is the freshest one in the buffer.
  float readDistanceMeters() {
    // Commanded ranging: flush, strobe pin4 high, collect up to SONAR_SAMPLES
    // "Rdddd\r" frames, strobe low, return their median (mm -> m). Commanding
    // the strobe (vs floating pin4) is what keeps the MB7388 from free-running
    // and backlogging unsolicited frames when Serial1 shares the pin.
    while (Serial1.available()) Serial1.read();          // drop stale frames
    digitalWrite(PIN_STROBE, HIGH);                       // pin4 high -> ranging

    int samp[SONAR_SAMPLES];
    int ns = 0;
    char digits[8];
    int di = 0;
    bool capturing = false;
    // ~6 Hz frames, so give ~200 ms each plus a settle margin
    unsigned long deadline = millis() + (unsigned long)SONAR_SAMPLES * 200UL + 300UL;
    while (ns < SONAR_SAMPLES && (long)(deadline - millis()) > 0) {
      while (Serial1.available()) {
        char c = (char)Serial1.read();
        if (c == 'R') {
          capturing = true;
          di = 0;
        } else if (capturing) {
          if (c == '\r') {
            digits[di] = 0;
            if (di >= 3) {  // accept 3 or 4 digit frames
              samp[ns++] = atoi(digits);
              sensor_last_frame_at = millis();
              if (ns >= SONAR_SAMPLES) break;
            }
            capturing = false;
            di = 0;
          } else if (di < (int)sizeof(digits) - 1 && c >= '0' && c <= '9') {
            digits[di++] = c;
          } else {
            capturing = false;
            di = 0;
          }
        }
      }
      delay(2);
    }

    digitalWrite(PIN_STROBE, LOW);                        // pin4 low -> stop ranging

    if (ns == 0) {
      Serial.println("   MaxBotix: no frame within timeout");
      return -1.0f;
    }
    return (float)median_int(samp, ns) / 1000.0f;
  }

  float readBatteryVoltage() {
    uint16_t mv = board.getBattMilliVolts();
    return mv / 1000.0;
  }

  void updateSensorReadings() {
    float d = readDistanceMeters();
    if (d >= 0.0f) {
      last_dist_m = d;
      has_sensor = true;
      has_reading = true;
    } else {
      // No fresh frame this cycle. Keep prior last_dist_m so display still
      // shows a value, but flag has_sensor=false so payload tags it as stale.
      has_sensor = false;
      has_reading = true;  // still send so receiver knows we're alive
    }
    last_batt = readBatteryVoltage();
  }

public:
  // Returns count of successful sends. sets last_send_successes/last_send_attempts.
  int sendSensorReading() {
    last_send_successes = 0;
    last_send_attempts = 0;
    // Decisive probe: clear the fault register, read it back (0x0000 => clear
    // works; still 0x0020 => the XOSC fault re-latches instantly = persistent),
    // then read again after the TX (0x0020 => the transmit itself re-faults it).
    radio.clearDeviceErrors();
    uint16_t err_cleared = radio.getDeviceErrors();

    if (target_count == 0) {
      Serial.println("   No targets set, skipping send (use 'target add <name>')");
      return 0;
    }

    // reading was cached in MODE_MEASURE; re-reading here finds the sonar
    // already gated off and clobbers has_sensor with a false failure

    for (uint8_t i = 0; i < target_count; i++) {
      last_send_attempts++;

      ContactInfo* recipient = lookupContactByPubKey(target_prefixes[i], TARGET_PREFIX_LEN);
      if (!recipient) {
        Serial.print("   skip: contact not found for prefix ");
        mesh::Utils::printHex(Serial, target_prefixes[i], TARGET_PREFIX_LEN);
        Serial.println();
        continue;
      }

      // v2: embed the sender's view of the forward path in the payload so the
      // receiver can correlate with its measured arrival, and (new) identify the
      // specific repeaters by hash byte. Format:
      //   fwd_hops=N fwd_path=aa.bb.cc   (N = byte length of path)
      //   fwd_hops=255 fwd_path=none     (path unknown, will flood)
      uint8_t fwd_hops = (recipient->out_path_len == OUT_PATH_UNKNOWN) ? 255 : recipient->out_path_len;
      char path_str[48];
      if (recipient->out_path_len == OUT_PATH_UNKNOWN || recipient->out_path_len == 0) {
        StrHelper::strncpy(path_str, "none", sizeof(path_str));
      } else {
        size_t off = 0;
        // Cap at what fits in 47 chars: 15 hops x "xx." = 45 chars + "xx" = 47. Path_len max
        // is 64 but in practice routes are short; we truncate defensively.
        uint8_t to_write = recipient->out_path_len;
        if (to_write > 15) to_write = 15;
        for (uint8_t h = 0; h < to_write && off < sizeof(path_str) - 3; h++) {
          int n = snprintf(path_str + off, sizeof(path_str) - off,
                           h == 0 ? "%02x" : ".%02x", recipient->out_path[h]);
          if (n < 0) break;
          off += n;
        }
        path_str[sizeof(path_str) - 1] = 0;
      }

      char msg[200];
      snprintf(msg, sizeof(msg),
               "[SENSOR] node_id=%u dist=%.3fm batt=%.2fV fwd_hops=%u fwd_path=%s",
               (unsigned)node_id, last_dist_m, last_batt, (unsigned)fwd_hops, path_str);
      Serial.printf("   [SENSOR] %s\n", msg);

      // v2: log the cached path bytes (not just length) before the send.
      Serial.printf("   -> %s: ", recipient->name);
      if (recipient->out_path_len == OUT_PATH_UNKNOWN) {
        Serial.print("(path UNKNOWN -> will FLOOD) ");
      } else {
        Serial.printf("(direct, len=%d, hashes=", recipient->out_path_len);
        for (uint8_t h = 0; h < recipient->out_path_len; h++) {
          Serial.printf("%02x%s", recipient->out_path[h], h < recipient->out_path_len - 1 ? "." : "");
        }
        Serial.print(") ");
      }

      // link-health LED: toggle the instant before every radio send (attempt 1
      // and each retry). processAck() drives it OFF on a good ACK, onSendTimeout()
      // ON on a miss; the mode machine forces it OFF entering sleep.
      ledToggle();

      uint32_t est_timeout;
      int result = sendMessage(*recipient, getRTCClock()->getCurrentTime(), 0, msg, expected_ack_crc, est_timeout);
      if (result == MSG_SEND_FAILED) {
        Serial.println("FAILED");
      } else {
        last_msg_sent = _ms->getMillis();
        last_ack_target_idx = i;  // v2: remember target so onSendTimeout can invalidate
        last_ack_status = ACK_STATUS_PENDING;
        Serial.println(result == MSG_SEND_SENT_FLOOD ? "sent (FLOOD)" : "sent (DIRECT)");
        last_send_successes++;
      }
    }

    Serial.printf("   (%d/%d sent)\n", last_send_successes, last_send_attempts);
    // 0x20=XOSC_START 0x40=PLL_LOCK 0x100=PA_RAMP. cleared = state right after a
    // clear (before TX); after_tx = state once the transmit finished.
    Serial.printf("   [RADIO_ERR] cleared=0x%04X after_tx=0x%04X\n", err_cleared, radio.getDeviceErrors());
    return last_send_successes;
  }

  bool sendAdvert() {
    auto pkt = createSelfAdvert(node_name, 0.0, 0.0);
    if (pkt) {
      sendZeroHop(pkt);
      Serial.println("   (advert sent, zero hop).");
      return true;
    }
    Serial.println("   ERR: unable to send");
    return false;
  }

  const char* getNodeName() const { return node_name; }
  uint16_t getNodeId() const { return node_id; }
  uint32_t getSendInterval() const { return send_interval_secs; }
  uint8_t getTargetCount() const { return target_count; }
  uint8_t getLastSendSuccesses() const { return last_send_successes; }
  uint8_t getLastSendAttempts() const { return last_send_attempts; }
  AckStatus getLastAckStatus() const { return last_ack_status; }
  uint32_t getLastAckRoundTripMs() const { return last_ack_rt_ms; }
  unsigned long getLastAckAtMillis() const { return last_ack_at_millis; }

  // link-health LED + last-transmit outcome, driven by the mode machine.
  void linkLedOff() { ledWrite(false); }   // force dark (power) on sleep entry
  void recordTxOutcome(bool failed, uint8_t attempts) { last_tx_failed = failed; last_tx_attempts = attempts; }
  bool getLastTxFailed() const { return last_tx_failed; }
  uint8_t getLastTxAttempts() const { return last_tx_attempts; }

  // Read the sensor + battery and cache the values WITHOUT sending -- used by
  // the OLED's Send page so the user can see a fresh reading before deciding
  // to long-press send. Blocks for up to ULTRASONIC_READ_TIMEOUT_MS.
  void refreshReading() { updateSensorReadings(); }

  // Returns name of target at index, or NULL if not found.
  const char* getTargetName(uint8_t idx) {
    if (idx >= target_count) return NULL;
    ContactInfo* t = lookupContactByPubKey(target_prefixes[idx], TARGET_PREFIX_LEN);
    return t ? t->name : NULL;
  }

  // Fills summary string with target names separated by commas.
  void formatTargetSummary(char* out, size_t out_sz) {
    if (target_count == 0) {
      StrHelper::strncpy(out, "(none)", out_sz);
      return;
    }
    out[0] = 0;
    size_t used = 0;
    for (uint8_t i = 0; i < target_count && used < out_sz - 1; i++) {
      const char* name = getTargetName(i);
      int n;
      if (i == 0) {
        n = snprintf(out + used, out_sz - used, "%s", name ? name : "?");
      } else {
        n = snprintf(out + used, out_sz - used, ",%s", name ? name : "?");
      }
      if (n < 0) break;
      used += n;
    }
  }

protected:
  float getAirtimeBudgetFactor() const override {
    return 1.0;
  }

  int calcRxDelay(float score, uint32_t air_time) const override {
    return 0;
  }

  bool allowPacketForward(const mesh::Packet* packet) override {
    return true;
  }

  // Auto-evict oldest non-favourite contact when list is full
  bool shouldOverwriteWhenFull() const override { return true; }

  void onContactOverwrite(const uint8_t* pub_key) override {
    Serial.print("   (evicted oldest non-favourite contact: ");
    mesh::Utils::printHex(Serial, pub_key, 4);
    Serial.println(")");
  }

  void onContactsFull() override {
    Serial.printf("   WARNING: contacts list is full (%d/%d). Use 'purge' to clear.\n", getNumContacts(), MAX_CONTACTS);
  }

  void onDiscoveredContact(ContactInfo& contact, bool is_new, uint8_t path_len, const uint8_t* path) override {
    Serial.printf("ADVERT from -> %s\n", contact.name);
    Serial.print("   public key: "); mesh::Utils::printHex(Serial, contact.id.pub_key, PUB_KEY_SIZE); Serial.println();
    saveContacts();
  }

  void onContactPathUpdated(const ContactInfo& contact) override {
    // v2: log path bytes, not just length, so stale-path debugging is possible.
    Serial.printf("PATH to: %s, path_len=%d, hashes=", contact.name, (uint32_t) contact.out_path_len);
    for (uint8_t h = 0; h < contact.out_path_len; h++) {
      Serial.printf("%02x%s", contact.out_path[h], h < contact.out_path_len - 1 ? "." : "");
    }
    Serial.println();
    saveContacts();
  }

  ContactInfo* processAck(const uint8_t *data) override {
    if (memcmp(data, &expected_ack_crc, 4) == 0) {
      last_ack_rt_ms = _ms->getMillis() - last_msg_sent;
      last_ack_at_millis = millis();
      last_ack_status = ACK_STATUS_OK;
      ledWrite(false);  // good ACK -> LED OFF (link healthy)
      Serial.printf("   Got ACK! (round trip: %lu millis)\n", (unsigned long)last_ack_rt_ms);
      expected_ack_crc = 0;
      return NULL;
    }
    return NULL;
  }

  void onMessageRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const char *text) override {
    Serial.printf("(%s) MSG -> from %s\n", pkt->isRouteDirect() ? "DIRECT" : "FLOOD", from.name);
    Serial.printf("   %s\n", text);
  }

  void onCommandDataRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const char *text) override {
  }
  void onSignedMessageRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const uint8_t *sender_prefix, const char *text) override {
  }

  void onChannelMessageRecv(const mesh::GroupChannel& channel, mesh::Packet* pkt, uint32_t timestamp, const char *text) override {
  }

  uint8_t onContactRequest(const ContactInfo& contact, uint32_t sender_timestamp, const uint8_t* data, uint8_t len, uint8_t* reply) override {
    return 0;
  }

  void onContactResponse(const ContactInfo& contact, const uint8_t* data, uint8_t len) override {
  }

  uint32_t calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const override {
    return SEND_TIMEOUT_BASE_MILLIS + (FLOOD_SEND_TIMEOUT_FACTOR * pkt_airtime_millis);
  }
  uint32_t calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const override {
    uint8_t path_hash_count = path_len & 63;
    return SEND_TIMEOUT_BASE_MILLIS +
         ( (pkt_airtime_millis*DIRECT_SEND_PERHOP_FACTOR + DIRECT_SEND_PERHOP_EXTRA_MILLIS) * (path_hash_count + 1));
  }

  void onSendTimeout() override {
    // If processAck already cleared expected_ack_crc, the ACK arrived just as the
    // timeout fired -- treat this as a late-but-no-op timeout and skip invalidation.
    // Without this guard, a slow-but-successful FLOOD round-trip would invalidate
    // the path we just learned from the receiver's PATH-return.
    if (expected_ack_crc == 0) return;
    last_ack_status = ACK_STATUS_TIMEOUT;
    ledWrite(true);  // no ACK -> LED ON (problem); next resend toggles it briefly off

    // v2: on timeout, invalidate the cached out_path for the most recent target.
    // The NEXT scheduled send will flood, which forces a fresh path discovery on
    // both sides: the flood accumulates path hashes, and the receiver updates its
    // own reverse out_path (which is what the ACK uses).
    if (last_ack_target_idx < target_count) {
      ContactInfo* c = lookupContactByPubKey(target_prefixes[last_ack_target_idx], TARGET_PREFIX_LEN);
      if (c && c->out_path_len != OUT_PATH_UNKNOWN) {
        Serial.printf("   ERROR: timed out, no ACK. Invalidating cached path to %s (was len=%d). Next send will FLOOD.\n",
                      c->name, (int)c->out_path_len);
        c->out_path_len = OUT_PATH_UNKNOWN;
        saveContacts();
        last_ack_target_idx = 0xFF;
        return;
      }
    }
    Serial.println("   ERROR: timed out, no ACK.");
  }

public:
  MyMesh(mesh::Radio& radio, StdRNG& rng, mesh::RTCClock& rtc, SimpleMeshTables& tables)
     : BaseChatMesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(16), tables)
  {
    command[0] = 0;
    expected_ack_crc = 0;
    last_msg_sent = 0;
    target_count = 0;
    memset(target_prefixes, 0, sizeof(target_prefixes));
    StrHelper::strncpy(node_name, ADVERT_NAME, sizeof(node_name));
    send_interval_secs = SENSOR_SEND_INTERVAL_SECS;
    last_send_time = 0;
    last_send_successes = 0;
    last_send_attempts = 0;
    last_ack_target_idx = 0xFF;  // v2: no send pending
    last_ack_status = ACK_STATUS_NONE;
    last_ack_rt_ms = 0;
    last_ack_at_millis = 0;
    led_on = false;
    last_tx_failed = false;
    last_tx_attempts = 0;
    node_id = 1;
  }

  void begin(FILESYSTEM& fs) {
    _fs = &fs;

    BaseChatMesh::begin();

  #if defined(NRF52_PLATFORM)
    IdentityStore store(fs, "");
  #elif defined(RP2040_PLATFORM)
    IdentityStore store(fs, "/identity");
    store.begin();
  #else
    IdentityStore store(fs, "/identity");
  #endif
    if (!store.load("_main", self_id, node_name, sizeof(node_name))) {
      Serial.println("Press ENTER to generate key:");
      char c = 0;
      while (c != '\n') {
        if (Serial.available()) c = Serial.read();
      }
      ((StdRNG *)getRNG())->begin(millis());

      self_id = mesh::LocalIdentity(getRNG());
      int count = 0;
      while (count < 10 && (self_id.pub_key[0] == 0x00 || self_id.pub_key[0] == 0xFF)) {
        self_id = mesh::LocalIdentity(getRNG()); count++;
      }
      store.save("_main", self_id);
    }

    loadPrefs();
    loadContacts();
    loadTargets();
  }

  void showWelcome() {
    Serial.println("===== MeshCore Companion Sensor =====");
    Serial.println();
    Serial.printf("Node: %s\n", node_name);
    Serial.print("Public key: "); mesh::Utils::printHex(Serial, self_id.pub_key, PUB_KEY_SIZE); Serial.println();
    Serial.printf("Node ID: %u\n", (unsigned)node_id);
    Serial.printf("Send interval: %d seconds\n", send_interval_secs);

    if (has_sensor) {
      Serial.println("DS18B20: detected");
    } else {
      Serial.println("DS18B20: not found (using dummy value 99.9C)");
    }

    if (target_count > 0) {
      char summary[128];
      formatTargetSummary(summary, sizeof(summary));
      Serial.printf("Targets (%d): %s\n", (int)target_count, summary);
    } else {
      Serial.println("Targets: none (use 'target add <name>')");
    }

    Serial.println();
    Serial.println("   (enter 'help' for commands)");
    Serial.println();
  }

  void sendSelfAdvert(int delay_millis) {
    auto pkt = createSelfAdvert(node_name, 0.0, 0.0);
    if (pkt) {
      sendFlood(pkt, delay_millis);
    }
  }

  // ContactVisitor
  void onContactVisit(const ContactInfo& contact) override {
    Serial.printf("   %s - ", contact.name);
    char tmp[40];
    int32_t secs = contact.last_advert_timestamp - getRTCClock()->getCurrentTime();
    AdvertTimeHelper::formatRelativeTimeDiff(tmp, secs, false);
    Serial.println(tmp);
  }

  void handleCommand(const char* command) {
    while (*command == ' ') command++;

    if (memcmp(command, "target add ", 11) == 0) {
      const char* name = &command[11];
      ContactInfo* contact = searchContactsByPrefix(name);
      if (!contact) {
        Serial.println("   Error: name prefix not found in contacts.");
      } else if (addTargetPrefix(contact->id.pub_key)) {
        Serial.printf("   Added target: %s (%d/%d)\n", contact->name, (int)target_count, MAX_SEND_TARGETS);
      } else {
        Serial.printf("   Error: target list full (max %d)\n", MAX_SEND_TARGETS);
      }
    } else if (memcmp(command, "target remove ", 14) == 0) {
      const char* name = &command[14];
      ContactInfo* contact = searchContactsByPrefix(name);
      if (!contact) {
        Serial.println("   Error: name prefix not found in contacts.");
      } else if (removeTargetPrefix(contact->id.pub_key)) {
        Serial.printf("   Removed target: %s\n", contact->name);
      } else {
        Serial.println("   Error: contact not in target list.");
      }
    } else if (strcmp(command, "target clear") == 0) {
      target_count = 0;
      saveTargets();
      Serial.println("   All targets cleared.");
    } else if (memcmp(command, "target ", 7) == 0) {
      // Legacy: `target <name>` replaces all targets with just this one
      const char* name = &command[7];
      ContactInfo* contact = searchContactsByPrefix(name);
      if (contact) {
        memcpy(target_prefixes[0], contact->id.pub_key, TARGET_PREFIX_LEN);
        target_count = 1;
        saveTargets();
        Serial.printf("   Target set to: %s (replaced any existing)\n", contact->name);
      } else {
        Serial.println("   Error: name prefix not found in contacts.");
      }
    } else if (strcmp(command, "target") == 0) {
      if (target_count == 0) {
        Serial.println("   No targets set.");
      } else {
        Serial.printf("   Targets (%d/%d):\n", (int)target_count, MAX_SEND_TARGETS);
        for (uint8_t i = 0; i < target_count; i++) {
          ContactInfo* t = lookupContactByPubKey(target_prefixes[i], TARGET_PREFIX_LEN);
          Serial.printf("     [%d] ", (int)i);
          if (t) {
            Serial.println(t->name);
          } else {
            mesh::Utils::printHex(Serial, target_prefixes[i], TARGET_PREFIX_LEN);
            Serial.println(" (contact not found)");
          }
        }
      }
    } else if (strcmp(command, "send") == 0) {
      Serial.println("   Sending sensor reading now...");
      sendSensorReading();
    } else if (memcmp(command, "list", 4) == 0) {
      int n = 0;
      if (command[4] == ' ') {
        n = atoi(&command[5]);
      }
      Serial.printf("Contacts: %d/%d\n", getNumContacts(), MAX_CONTACTS);
      scanRecentContacts(n, this);
    } else if (strcmp(command, "purge") == 0) {
      int before = getNumContacts();
      resetContacts();
      saveContacts();
      Serial.printf("   Purged %d contacts (0/%d now)\n", before, MAX_CONTACTS);
    } else if (memcmp(command, "to ", 3) == 0) {
      // Alias for `target <name>` (replace all)
      const char* name = &command[3];
      ContactInfo* contact = searchContactsByPrefix(name);
      if (contact) {
        memcpy(target_prefixes[0], contact->id.pub_key, TARGET_PREFIX_LEN);
        target_count = 1;
        saveTargets();
        Serial.printf("   Target set to: %s (replaced any existing)\n", contact->name);
      } else {
        Serial.println("   Error: name prefix not found in contacts.");
      }
    } else if (memcmp(command, "card", 4) == 0) {
      Serial.printf("Hello %s\n", node_name);
      auto pkt = createSelfAdvert(node_name, 0.0, 0.0);
      if (pkt) {
        uint8_t len = pkt->writeTo(tmp_buf);
        releasePacket(pkt);
        mesh::Utils::toHex(hex_buf, tmp_buf, len);
        Serial.println("Your MeshCore biz card:");
        Serial.print("meshcore://"); Serial.println(hex_buf);
        Serial.println();
      } else {
        Serial.println("  Error");
      }
    } else if (memcmp(command, "import ", 7) == 0) {
      importCard(&command[7]);
    } else if (strcmp(command, "advert") == 0) {
      sendAdvert();
    } else if (strcmp(command, "radio") == 0) {
      Serial.printf("[RADIO] freq=%.3f MHz  bw=%.1f kHz  sf=%d  cr=4/%d  txpwr=%d\n",
                    (double)LORA_FREQ, (double)LORA_BW, (int)LORA_SF, (int)LORA_CR, (int)LORA_TX_POWER);
    } else if (memcmp(command, "set ", 4) == 0) {
      const char* config = &command[4];
      if (memcmp(config, "name ", 5) == 0) {
        StrHelper::strncpy(node_name, &config[5], sizeof(node_name));
        savePrefs();
        Serial.printf("   OK, name set to: %s\n", node_name);
      } else if (memcmp(config, "node_id ", 8) == 0) {
        uint32_t val = _atoi(&config[8]);
        if (val > 0 && val <= 65535) {
          node_id = (uint16_t)val;
          savePrefs();
          Serial.printf("   OK, node_id set to %u\n", (unsigned)node_id);
        } else {
          Serial.println("   Error: node_id must be 1-65535");
        }
      } else if (memcmp(config, "interval ", 9) == 0) {
        uint32_t val = _atoi(&config[9]);
        if (val >= 10) {
          send_interval_secs = val;
          savePrefs();
          Serial.printf("   OK, interval set to %d seconds\n", send_interval_secs);
        } else {
          Serial.println("   Error: minimum interval is 10 seconds");
        }
      } else {
        Serial.printf("   ERROR: unknown config: %s\n", config);
      }
    } else if (memcmp(command, "ver", 3) == 0) {
      Serial.println(FIRMWARE_VER_TEXT);
    } else if (memcmp(command, "help", 4) == 0) {
      Serial.println("Commands:");
      Serial.println("   card                  - show your biz card for sharing");
      Serial.println("   import <biz card>     - import a contact's biz card");
      Serial.println("   list {n}              - list contacts (last n)");
      Serial.println("   purge                 - clear all contacts");
      Serial.println("   target                - show current targets");
      Serial.println("   target add <name>     - add a send target (max 4)");
      Serial.println("   target remove <name>  - remove a send target");
      Serial.println("   target clear          - clear all targets");
      Serial.println("   target <name>         - set to single target (replaces all)");
      Serial.println("   to <name>             - alias for 'target <name>'");
      Serial.println("   send                  - send a reading now");
      Serial.println("   set name <name>       - set node name");
      Serial.println("   set node_id <id>      - set node ID (1-65535)");
      Serial.println("   set interval <secs>   - set send interval (min 10)");
      Serial.println("   advert                - send advertisement");
      Serial.println("   ver                   - show firmware version");
      Serial.println("   help                  - show this help");
    } else {
      Serial.print("   ERROR: unknown command: "); Serial.println(command);
    }
  }

  void loop() {
    BaseChatMesh::loop();

    // NOTE: the field-node mode machine (MODE_TRANSMIT) is the SOLE transmit
    // authority now -- the old interval auto-send is removed so a send can never
    // fire outside the duty cycle (e.g. during SLEEP). We still call this loop()
    // for BaseChatMesh radio/ACK servicing and the serial command console below.

    // Serial command handling
    int len = strlen(command);
    while (Serial.available() && len < (int)sizeof(command)-1) {
      char c = Serial.read();
      if (c == '\r' || c == '\n') {
        if (len > 0) {
          Serial.println();
          command[len] = 0;
          handleCommand(command);
          command[0] = 0;
          len = 0;
        }
        continue;  // skip \r and \n
      }
      command[len++] = c;
      command[len] = 0;
      Serial.print(c);
    }
    if (len == (int)sizeof(command)-1) {
      command[len] = 0;
      Serial.println();
      handleCommand(command);
      command[0] = 0;
    }
  }
};

StdRNG fast_rng;
SimpleMeshTables tables;
MyMesh the_mesh(radio_driver, fast_rng, rtc_clock, tables);

/* ---------------------------------- DISPLAY & BUTTON ---------------------------------- */

#ifdef DISPLAY_CLASS
static uint8_t current_page = PAGE_STATUS;
static unsigned long next_display_refresh = 0;
static unsigned long last_button_activity = 0;
static char alert_text[32] = {0};
static unsigned long alert_expiry = 0;

static void showAlert(const char* text) {
  strncpy(alert_text, text, sizeof(alert_text)-1);
  alert_text[sizeof(alert_text)-1] = 0;
  alert_expiry = millis() + ALERT_DURATION_MS;
}

static void drawPageDots(DisplayDriver& d) {
  int y = 2;
  int x = d.width() / 2 - 5 * (PAGE_COUNT - 1);
  for (uint8_t i = 0; i < PAGE_COUNT; i++, x += 10) {
    if (i == current_page) {
      d.fillRect(x-1, y-1, 3, 3);
    } else {
      d.fillRect(x, y, 1, 1);
    }
  }
}

static bool g_demo_banner = false;   // DEMO renders the status page; banner tells them apart

static void renderStatusPage(DisplayDriver& d) {
  d.setTextSize(1);
  d.setColor(DisplayDriver::LIGHT);

  // Node name and ID (DEMO banner in demo mode so the pages are tellable apart)
  char buf[32];
  if (g_demo_banner) {
    snprintf(buf, sizeof(buf), "** DEMO ** [%u]", (unsigned)the_mesh.getNodeId());
  } else {
    snprintf(buf, sizeof(buf), "%s [%u]", the_mesh.getNodeName(), (unsigned)the_mesh.getNodeId());
  }
  d.setCursor(0, 10);
  d.print(buf);

  // Distance (water-level when mounted pointing down)
  if (!has_sensor) {
    snprintf(buf, sizeof(buf), "Sensor: NOT DETECTED");
  } else if (has_reading) {
    snprintf(buf, sizeof(buf), "Dist: %.2fm", last_dist_m);
  } else {
    snprintf(buf, sizeof(buf), "Dist: --");
  }
  d.setCursor(0, 22);
  d.print(buf);

  // Battery
  if (has_reading) {
    snprintf(buf, sizeof(buf), "Batt: %.2fV", last_batt);
  } else {
    snprintf(buf, sizeof(buf), "Batt: --");
  }
  d.setCursor(0, 34);
  d.print(buf);

  // Targets
  uint8_t tc = the_mesh.getTargetCount();
  if (tc == 0) {
    snprintf(buf, sizeof(buf), "To: (none)");
  } else if (tc == 1) {
    const char* tname = the_mesh.getTargetName(0);
    snprintf(buf, sizeof(buf), "To: %s", tname ? tname : "(?)");
  } else {
    char summary[24];
    the_mesh.formatTargetSummary(summary, sizeof(summary));
    snprintf(buf, sizeof(buf), "To(%d): %s", (int)tc, summary);
  }
  d.setCursor(0, 46);
  d.print(buf);

  // Bottom line: if the last deploy transmit failed every attempt, show that as
  // an error with the try count (the installer's "is this node getting through?"
  // check). Otherwise the usual last-ACK-age / interval feedback.
  if (the_mesh.getLastTxFailed()) {
    snprintf(buf, sizeof(buf), "ERR: no ACK x%d", (int)the_mesh.getLastTxAttempts());
  } else {
    switch (the_mesh.getLastAckStatus()) {
      case ACK_STATUS_OK: {
        unsigned long age_ms = millis() - the_mesh.getLastAckAtMillis();
        unsigned long age_s = age_ms / 1000UL;
        if (age_s < 60) {
          snprintf(buf, sizeof(buf), "Last: ACK %lus ago", age_s);
        } else if (age_s < 3600) {
          snprintf(buf, sizeof(buf), "Last: ACK %lum ago", age_s / 60);
        } else if (age_s < 86400) {
          snprintf(buf, sizeof(buf), "Last: ACK %luh ago", age_s / 3600);
        } else {
          snprintf(buf, sizeof(buf), "Last: ACK %lud ago", age_s / 86400);
        }
        break;
      }
      case ACK_STATUS_PENDING:
        snprintf(buf, sizeof(buf), "Last: sending...");
        break;
      case ACK_STATUS_TIMEOUT:
        snprintf(buf, sizeof(buf), "Last: no ACK");
        break;
      default:
        snprintf(buf, sizeof(buf), "Every %ds", (int)the_mesh.getSendInterval());
        break;
    }
  }
  d.setCursor(0, 56);
  d.print(buf);
}

static void renderSendPage(DisplayDriver& d) {
  d.setTextSize(1);
  d.setColor(DisplayDriver::LIGHT);
  d.drawTextCentered(d.width() / 2, 8, "Send Sensor");

  // Live preview: refreshed on page-arrival via refreshReading(). Shows the
  // user what would actually go out if they long-press now.
  char buf[24];
  if (!has_sensor) {
    snprintf(buf, sizeof(buf), "Sensor: NOT DETECTED");
  } else if (has_reading) {
    snprintf(buf, sizeof(buf), "Dist: %.2fm", last_dist_m);
  } else {
    snprintf(buf, sizeof(buf), "Dist: --");
  }
  d.drawTextCentered(d.width() / 2, 28, buf);

  snprintf(buf, sizeof(buf), "Batt: %.2fV", last_batt);
  d.drawTextCentered(d.width() / 2, 40, buf);

  d.drawTextCentered(d.width() / 2, 54, "long press = send");
}

static void renderAdvertPage(DisplayDriver& d) {
  d.setTextSize(1);
  d.setColor(DisplayDriver::LIGHT);
  d.drawTextCentered(d.width() / 2, 24, "Send Advert");
  d.drawTextCentered(d.width() / 2, 44, "long press");
}

static void renderAlert(DisplayDriver& d) {
  int y = d.height() / 3;
  int p = d.height() / 16;
  d.setColor(DisplayDriver::DARK);
  d.fillRect(p, y, d.width() - p*2, y);
  d.setColor(DisplayDriver::LIGHT);
  d.drawRect(p, y, d.width() - p*2, y);
  d.drawTextCentered(d.width() / 2, y + p + 6, alert_text);
}

static void renderDisplay() {
  if (!display.isOn()) return;
  if (millis() < next_display_refresh) return;

  display.startFrame();
  drawPageDots(display);

  switch (current_page) {
    case PAGE_STATUS:  renderStatusPage(display); break;
    case PAGE_SEND:    renderSendPage(display); break;
    case PAGE_ADVERT:  renderAdvertPage(display); break;
  }

  if (millis() < alert_expiry) {
    renderAlert(display);
  }

  display.endFrame();
  next_display_refresh = millis() + DISPLAY_REFRESH_MS;
}

static void handleButtonEvents() {
  int ev = user_btn.check();
  if (ev == BUTTON_EVENT_NONE) return;

  // Any button press turns display on and resets auto-off timer
  last_button_activity = millis();
  if (!display.isOn()) {
    DISPLAY_TURN_ON();
    next_display_refresh = 0;  // force immediate refresh
    return;  // consume this press just to turn on the screen
  }

  if (ev == BUTTON_EVENT_CLICK) {
    // Short press: cycle to next page
    current_page = (current_page + 1) % PAGE_COUNT;
    next_display_refresh = 0;  // force immediate refresh

    // Landing on the Send page kicks off an immediate sensor read so the page
    // shows a live preview -- easy way to verify the sensor is alive without
    // sending anything over the radio.
    if (current_page == PAGE_SEND) {
      the_mesh.refreshReading();
    }
  } else if (ev == BUTTON_EVENT_LONG_PRESS) {
    if (current_page == PAGE_SEND) {
      the_mesh.sendSensorReading();
      uint8_t succ = the_mesh.getLastSendSuccesses();
      uint8_t att = the_mesh.getLastSendAttempts();
      if (att == 0) {
        // No `target add ...` has been done yet -- make this very explicit.
        showAlert("Target not set");
      } else {
        // Switch to status page so the user can watch the "Last: ..." line cycle
        // through sending -> ACK Nms / no ACK. Overlay a brief alert as immediate
        // confirmation that the long-press registered.
        current_page = PAGE_STATUS;
        next_display_refresh = 0;
        char alert[24];
        if (succ == att) {
          snprintf(alert, sizeof(alert), att == 1 ? "Sent" : "Sent %d/%d", succ, att);
        } else {
          snprintf(alert, sizeof(alert), "Sent %d/%d", succ, att);
        }
        showAlert(alert);
      }
    } else if (current_page == PAGE_ADVERT) {
      if (the_mesh.sendAdvert()) {
        showAlert("Advert sent!");
      } else {
        showAlert("Advert failed");
      }
    }
    next_display_refresh = 0;
  }
}

static void displayLoop() {
  handleButtonEvents();
  renderDisplay();

  // Auto-off after inactivity
  if (display.isOn() && last_button_activity > 0 &&
      (millis() - last_button_activity) > DISPLAY_AUTO_OFF_MS) {
    DISPLAY_TURN_OFF();
  }
}
#endif  // DISPLAY_CLASS

/* ============================ FIELD-NODE MODE MACHINE ============================ */
// Duty-cycle deployment layer over the companion_sensor mesh node. The pure
// transition logic lives in modes.h (host-tested, 29 checks); this wires each
// mode to the sonar gate, OLED, and mesh transmit/ACK path. Defaults to the
// low-power deploy cycle; the dev views (STATUS/DEMO/TX_DEBUG) are opt-in via B1.
#include "modes.h"
#include "radio_lowpower.h"   // app-side radio SPI-sleep/wake (MeshCore left pristine)

#define PIN_SONAR_GATE     5          // D5 = GPS_EN = AO3400 low-side gate (HIGH = sonar powered)
#ifndef CYCLE_SECS
#define CYCLE_SECS         300        // deploy measure/transmit cadence (5 min); -D override for bench
#endif
#ifndef STATUS_SECS
#define STATUS_SECS        30         // MODE_STATUS dwell on the OLED; -D override for bench
#endif
#define DEMO_SECS          CYCLE_SECS // MODE_DEMO runs one cycle then lapses back to sleep
#define TX_RETRY_MS        5000       // wait this long for an ACK before a retry
#define TX_MAX_ATTEMPTS    5          // give up after this many sends
#define WAKE_SETTLE_MS     300        // sonar power-up settle on a plain (timer) wake
// MODE_SLEEP idle granularity: delay() WFE-sleeps the CPU this long between
// cadence/button re-checks. At 50 ms the core is woken 20x per second purely to
// re-read a clock and a pin, and each wake costs. Overridable so the wake rate can
// be measured as its own variable: the arithmetic (147 uA System-OFF vs ~1066 uA
// System-ON) says the remaining floor is idle overhead, not a peripheral.
// Cost of raising it: button-press latency during sleep, and cadence granularity -
// both irrelevant against a 20 s bench or 300 s deployment cycle.
#ifndef SLEEP_POLL_MS
#define SLEEP_POLL_MS      50
#endif
#define GESTURE_WINDOW_MS  1500       // bounded button-classify cap in WAKE (> long-press 1200ms)
#ifndef SONAR_QUIET_MS
#define SONAR_QUIET_MS     2000       // let the gated sonar fully spin down before TX (RX desense fix)
#endif

static mode g_mode = MODE_POST;
static unsigned long g_mode_since = 0;   // millis() at mode entry
static bool g_woke_by_button = false;    // how SLEEP was interrupted (button vs RTC)
static int  g_tx_attempts = 0;
static unsigned long g_tx_sent_at = 0;
static unsigned long g_txdbg_settled_at = 0;
#define TXDBG_LINGER_MS 3000

static void sonar_power(bool on) {
#if STRIP_SONAR
  (void)on; return;                    // bisection arm: sonar layer removed
#endif
#ifdef SONAR_FORCE_OFF
  (void)on;
  digitalWrite(PIN_SONAR_GATE, LOW);   // TEST: sonar forced OFF to isolate whether its
                                       // gated power/RF noise desenses the radio RX
#else
  digitalWrite(PIN_SONAR_GATE, on ? HIGH : LOW);
#endif
}

// draw the shared status page (sensor / battery / target / ACK age) in its own frame
static void render_status_frame() {
  if (!display.isOn()) return;
  display.startFrame();
  renderStatusPage(display);
  display.endFrame();
}

static void render_post() {
  if (!display.isOn()) return;
  display.startFrame();
  display.setCursor(0, 0);  display.print("POST self-test");
  display.setCursor(0, 16); display.print(has_sensor ? "sonar: OK" : "sonar: --");
  display.setCursor(0, 28); display.print("radio: OK");
  display.endFrame();
}

static void render_tx_debug() {
  if (!display.isOn()) return;
  char line[40];
  const char* as = "-";
  switch (the_mesh.getLastAckStatus()) {
    case ACK_STATUS_PENDING: as = "pending"; break;
    case ACK_STATUS_OK:      as = "ACK OK";  break;
    case ACK_STATUS_TIMEOUT: as = "TIMEOUT"; break;
    default: break;
  }
  display.startFrame();
  display.setCursor(0, 0);  display.print("TX DEBUG");
  snprintf(line, sizeof(line), "dist %.2fm", last_dist_m); display.setCursor(0, 14); display.print(line);
  snprintf(line, sizeof(line), "batt %.2fV", last_batt);   display.setCursor(0, 26); display.print(line);
  snprintf(line, sizeof(line), "ack %s rt%lu", as, (unsigned long)the_mesh.getLastAckRoundTripMs());
  display.setCursor(0, 38); display.print(line);
  snprintf(line, sizeof(line), "try %d/%d", g_tx_attempts, TX_MAX_ATTEMPTS);
  display.setCursor(0, 50); display.print(line);
  display.endFrame();
}

// entry actions: run once when a mode becomes current.
static void enter_mode(mode m) {
  Serial.printf("[MODE] %s -> %s\n", mode_name(g_mode), mode_name(m));   // serial trace for bench HIL
  g_mode = m;
  g_mode_since = millis();

  switch (m) {
    case MODE_POST:
      DISPLAY_TURN_ON();
      break;
    case MODE_STATUS:
#if SLEEP_RELEASE_TWI
      Wire.begin();                  // direct hop from SLEEP: bus up before the panel
#endif
      g_demo_banner = false;
      DISPLAY_TURN_ON();
      break;
    case MODE_SLEEP:
      DISPLAY_TURN_OFF();
#if SLEEP_RELEASE_UART
      sonar_serial_end();            // release UARTE0 + both pads BEFORE gating: an enabled
                                     // UARTE holds HFCLK (~mA) and an active RX pad loads the
                                     // sensor output while its ground floats. Measured
                                     // 2026-07-18: this is worth ~5.5 mA of sleep floor.
#endif
      sonar_power(false);            // gate cuts sonar draw
      NRF_P1->LATCH = (1u << 0);     // stale press latched while awake must not rewake us
      the_mesh.linkLedOff();         // force the lamp dark for sleep, even after a fully-failed transmit
      radio_sleep_lp();              // SPI-sleep the radio (~1uA); loop() then skips the_mesh.loop() while slept
#if SLEEP_RELEASE_TWI
      Wire.end();                    // TWIM is another always-enabled HFCLK holder; the
                                     // display is already off, so the bus has no user here
#endif
#if SLEEP_PIN_DISCONNECT_AUDIT
      sleep_pins_disconnect();       // LAST: after every release above, silence the listed
                                     // pads (buffer-disconnected). Wake bring-up rewrites
                                     // each pin's PIN_CNF, so there is no restore to forget.
#endif
#if SLEEP_DIAG
      sleep_diag_dump();             // state snapshot AFTER all releases, before the idle loop
#endif
#if SLEEP_DISABLE_USBD
      if (!(NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk)) {
        NRF_USBD->ENABLE = 0;        // battery only: drop USBD so it stops holding HFCLK.
                                     // One-way until reset; VBUS-present skips this entirely.
      }
#endif
#if SLEEP_HFCLK_STOP
      if (!(NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk)) {
        NRF_CLOCK->TASKS_HFCLKSTOP = 1;   // battery only: release the latched HFXO.
                                          // HF users fall back to HFINT on demand.
      }
#endif
      break;
    case MODE_WAKE:
#if SLEEP_RELEASE_TWI
      Wire.begin();                  // bus back up before anything talks to the display
#endif
      radio_wake_lp();               // wake radio + re-arm Rx before measure/transmit; un-pauses the mesh
      sonar_power(true);
#if SLEEP_RELEASE_UART
      sonar_serial_begin();          // UARTE0 back up, D1 reclaimed as the pin-4 strobe.
                                     // Paired with the sleep-side release: if we did not
                                     // release, we must not re-begin.
#endif
      break;
    case MODE_MEASURE:
      break;
    case MODE_TRANSMIT:
      DISPLAY_TURN_OFF();
      sonar_power(false);            // sonar off for the TX/ACK window: its gated power/RF
                                     // noise desenses the radio RX (bench-confirmed). The
                                     // reading was already cached in MODE_MEASURE.
      delay(SONAR_QUIET_MS);         // let the MaxBotix fully spin down (its bulk caps keep
                                     // it emitting after the gate cuts) before the radio TX
      g_tx_attempts = 1;
      the_mesh.sendSensorReading();
      g_tx_sent_at = millis();
      break;
    case MODE_DEMO:
      g_demo_banner = true;
      DISPLAY_TURN_ON();
      sonar_power(true);
#if SLEEP_RELEASE_UART
      sonar_serial_begin();          // STATUS gateway: UARTE may still be released
#endif
      break;
    case MODE_TRANSMIT_DEBUG:
      DISPLAY_TURN_ON();
      radio_wake_lp();               // STATUS gateway: radio may still be slept
      g_txdbg_settled_at = 0;
      g_tx_attempts = 1;
      the_mesh.sendSensorReading();
      g_tx_sent_at = millis();
      break;
    default:
      break;
  }
}

// per-tick driver: build the event for the current mode, run its continuous
// action, then advance the state machine via the pure next_mode().
static void mode_loop() {
  int btn = user_btn.check();               // classifies CLICK/DOUBLE/LONG when ready
  unsigned long in_mode = millis() - g_mode_since;

  mode_event me;
  me.btn = btn;
  me.elapsed = false;
  me.done = false;
  me.woke_by_button = g_woke_by_button;

  switch (g_mode) {
    case MODE_POST:
      render_post();
      me.done = (in_mode > 3000);           // self-test banner, long enough to read
      break;
    case MODE_STATUS:
      render_status_frame();
      me.elapsed = (in_mode > STATUS_SECS * 1000UL);
      break;
    case MODE_SLEEP:
      // Display off + sonar gated off. CPU idle via FreeRTOS delay(): vTaskDelay
      // blocks this task so the core WFE-sleeps in the idle task, and the RTC-
      // driven RTOS tick reliably resumes us to re-check the cadence + button.
      // (A raw waitForEvent() here hung on battery -- with USB, CDC interrupts were
      // masking it by waking WFE; the JS220 caught the stuck node. delay() is the
      // FreeRTOS-correct low-power idle.) Stage 2: radio.sleep() for the ~147uA
      // floor per the sleep_test derisking; radio still RX-listening here (~8mA floor).
#if SLEEP_FPU_CLEAR
      // Errata 87: a float op latches the FPU exception -> FPU IRQ pending ->
      // WFE returns immediately and the idle task spins instead of sleeping.
      // QT1 measured fpu_pend=1 at sleep entry on this firmware. Clear the
      // FPSCR exception bits and the pending IRQ before each idle interval.
      __set_FPSCR(__get_FPSCR() & ~(0x0000009FUL));
      (void) __get_FPSCR();
      NVIC_ClearPendingIRQ(FPU_IRQn);
#endif
      delay(SLEEP_POLL_MS);
      if (in_mode > CYCLE_SECS * 1000UL) { g_woke_by_button = false; me.elapsed = true; }
      else if (btn != BUTTON_EVENT_NONE || user_btn.isPressed() ||
               (NRF_P1->LATCH & (1u << 0))) {
        NRF_P1->LATCH = (1u << 0);   // consume the hardware-latched edge
        g_woke_by_button = true;
      }
      me.woke_by_button = g_woke_by_button;
      break;
    case MODE_WAKE:
      if (g_woke_by_button) {
        if (in_mode > GESTURE_WINDOW_MS) me.done = true;   // spurious wake -> measure anyway
      } else {
        me.done = (in_mode > WAKE_SETTLE_MS);              // timer wake: settle then measure
      }
      break;
    case MODE_MEASURE:
      the_mesh.refreshReading();            // sonar + battery into cache (blocks ~read timeout)
      me.done = true;
      break;
    case MODE_TRANSMIT: {
      // Retry only after the FULL TX_RETRY_MS. The mesh flags its own send-timeout
      // in <1s, but the real ACK round-trip is ~2.7s -- retrying on the mesh flag
      // would re-send before the ACK can arrive (retry < RTT) and never catch it.
      // expected_ack_crc stays set across the wait, so a late ACK still lands.
      AckStatus s = the_mesh.getLastAckStatus();
      if (s == ACK_STATUS_OK) {
        the_mesh.recordTxOutcome(false, g_tx_attempts);        // acked (took g_tx_attempts tries)
        me.done = true;
      } else if ((millis() - g_tx_sent_at) > TX_RETRY_MS) {
        if (g_tx_attempts >= TX_MAX_ATTEMPTS) {
          the_mesh.recordTxOutcome(true, g_tx_attempts);       // all attempts missed -> STATUS shows error
          me.done = true;                                      // exhausted -> back to sleep (LED forced off there)
        } else {
          g_tx_attempts++; the_mesh.sendSensorReading(); g_tx_sent_at = millis();
        }
      }
      break;
    }
    case MODE_DEMO:
      the_mesh.refreshReading();
      render_status_frame();
      me.elapsed = (in_mode > DEMO_SECS * 1000UL);
      break;
    case MODE_TRANSMIT_DEBUG: {
      render_tx_debug();
      AckStatus s = the_mesh.getLastAckStatus();
      bool settled = (s == ACK_STATUS_OK || s == ACK_STATUS_TIMEOUT ||
                      (millis() - g_tx_sent_at) > TX_RETRY_MS);
      if (settled && g_txdbg_settled_at == 0) g_txdbg_settled_at = millis();
      if (settled && (millis() - g_txdbg_settled_at) > TXDBG_LINGER_MS)
        me.done = true;   // linger so the outcome is readable before leaving
      break;
    }
    default:
      break;
  }

  mode next = next_mode(g_mode, me);
  if (next != g_mode) {
    if (g_mode == MODE_SLEEP || g_mode == MODE_WAKE) g_woke_by_button = false;   // wake consumed on SLEEP exit
    enter_mode(next);
  }
}

/* ---------------------------------- SETUP & LOOP -------------------------------------- */

void halt() {
  while (1) ;
}

void setup() {
  Serial.begin(115200);
  // Give USB CDC a moment to enumerate so the first prints actually land.
  delay(200);

  Serial.println();
  Serial.println("================================================");
  Serial.println(FIRMWARE_VER_TEXT);
  Serial.println("================================================");
  Serial.println("setup: Serial up.");

  // link-health LED (P0.15, active-high): start dark. Toggled per send, ACK->off,
  // timeout->on, forced off on sleep entry (see MyMesh + the mode machine).
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_STATE_ON ? LOW : HIGH);

  Serial.println("setup: board.begin()...");
  board.begin();

#if PWR_ENABLE_DCDC
  // RookBoard::begin() calls NRF52Board::begin(), skipping NRF52BoardDCDC::begin(),
  // so the DC/DC enable never fires and the board runs on the LDO. Do it here
  // rather than patching Don's board file. Must come AFTER board.begin(), which is
  // where the SoftDevice comes up.
  {
    uint8_t sd_enabled = 0;
    sd_softdevice_is_enabled(&sd_enabled);
    if (sd_enabled) {
      sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    } else {
      NRF_POWER->DCDCEN = 1;
    }
    Serial.println("setup: DC/DC regulator ENABLED (PWR_ENABLE_DCDC=1)");
  }
#endif

  // Bring the radio up with the sonar gate OFF. The Rook's SX1262 runs on a TCXO;
  // switching the gated sonar rail (AO3400 on pin 5) on during radio_init sags the
  // supply right as the TCXO starts -> XOSC_START_ERR (0x0020) and an off-frequency
  // radio that can't ACK. Order matters: radio first, sonar second. (HW backstop:
  // bulk decoupling on the sonar rail is the follow-up.)
  pinMode(PIN_SONAR_GATE, OUTPUT);
  sonar_power(false);              // keep sonar off through radio startup

  // Without begin() the nRF pin's input buffer stays disconnected and
  // digitalRead never sees the button, no matter what the pad does.
  user_btn.begin();
  // Polled sampling misses taps that start and end between polls. SENSE-low
  // makes the port's LATCH register catch the falling edge in hardware (async,
  // no clock, no interrupt); the sleep poll reads and clears it. Same SENSE
  // machinery a future System-OFF button-wake needs.
  NRF_P1->PIN_CNF[0] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
  NRF_P1->LATCH = (1u << 0);

  Serial.println("setup: radio_init()...");
  if (!radio_init()) {
    Serial.println("setup: radio_init() FAILED -- halting.");
    halt();
  }
  delay(20);                       // let the TCXO fully settle before trusting it
  radio.clearDeviceErrors();       // clear any startup XOSC/cal latch now that it's stable
  Serial.printf("setup: radio OK. [RADIO_ERR post-init=0x%04X]\n", radio.getDeviceErrors());

  fast_rng.begin(radio_get_rng_seed());

#if STRIP_MESH
  // S3+: no mesh, no cadence -- the arm is a bare sleep floor. Warm-sleep the
  // radio right after init (the v4 pattern) and let the stripped loop() idle.
  radio_sleep_lp();
  Serial.println("setup: STRIP_MESH -- radio warm-slept, entering bare idle loop.");
#endif

#if !STRIP_SONAR
  // radio is up and stable -- now power the sonar for the MaxBotix probe + cycles
  sonar_power(true);

  // Initialize MaxBotix MB7388 (TTL serial, 9600 baud, "Rxxxx\r" frames).
  // Wire the MaxBotix pin 5 (serial TX) to the Rook's Serial1 RX pin; no TX
  // wire back is needed. Power: 3.0-5.5 V on pin 6, ground on pin 7.
  Serial.println("setup: probing MaxBotix on Serial1 @ 9600 baud...");
  // Brings up UARTE0 and reclaims D1 from Serial1's TXD so pin 4 is a GPIO strobe
  // (else it idles HIGH and the sensor free-runs). Same helper the wake path uses.
  sonar_serial_begin();
  digitalWrite(PIN_STROBE, HIGH);   // range during the probe window
  // Give the sensor up to ~500 ms to emit a first frame so we can flag
  // has_sensor on the OLED. has_sensor will be re-asserted on every successful
  // updateSensorReadings() anyway.
  {
    unsigned long start = millis();
    bool got = false;
    bool capturing = false;
    int di = 0;
    while ((millis() - start) < 500 && !got) {
      while (Serial1.available()) {
        char c = (char)Serial1.read();
        if (c == 'R') { capturing = true; di = 0; }
        else if (capturing) {
          if (c == '\r' && di >= 3) { got = true; break; }
          if (c >= '0' && c <= '9') di++;
          else { capturing = false; di = 0; }
        }
      }
      delay(5);
    }
    if (got) {
      has_sensor = true;
      Serial.println("MaxBotix MB7388 detected on Serial1.");
    } else {
      has_sensor = false;
      Serial.println("No MaxBotix frames on Serial1 (check wiring / 9600 baud).");
    }
    digitalWrite(PIN_STROBE, LOW);   // stop ranging; cycles command it per reading
  }
#endif  /* !STRIP_SONAR */

#if defined(DISPLAY_CLASS) && !STRIP_DISPLAY
  if (display.begin()) {
    display.startFrame();
    display.setCursor(0, 0);
    display.print("Starting...");
    display.endFrame();
  }
#endif

#if STRIP_FS && !STRIP_MESH
#error "STRIP_FS requires STRIP_MESH: the_mesh.begin() needs the filesystem"
#endif
#if !STRIP_FS
  Serial.println("setup: filesystem.begin()...");
#if defined(NRF52_PLATFORM)
  InternalFS.begin();
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
#elif defined(ESP32)
  SPIFFS.begin(true);
#else
  #error "need to define filesystem"
#endif
#endif  /* !STRIP_FS */

#if !STRIP_MESH
  Serial.println("setup: the_mesh.begin() -- if first-boot, will block here on 'Press ENTER to generate key:'");
#if defined(NRF52_PLATFORM)
  the_mesh.begin(InternalFS);
#elif defined(RP2040_PLATFORM)
  the_mesh.begin(LittleFS);
#elif defined(ESP32)
  the_mesh.begin(SPIFFS);
#endif
  Serial.println("setup: mesh ready.");

  radio_set_params(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR);
  radio_set_tx_power(LORA_TX_POWER);
  Serial.printf("[RADIO] freq=%.3f MHz  bw=%.1f kHz  sf=%d  cr=4/%d  txpwr=%d\n",
                (double)LORA_FREQ, (double)LORA_BW, (int)LORA_SF, (int)LORA_CR, (int)LORA_TX_POWER);

  the_mesh.showWelcome();
  Serial.println("setup: done, entering loop().");

#ifdef DISPLAY_CLASS
  last_button_activity = millis();
#endif

  // Send out initial advertisement
#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvert(1200);
#endif

  // hand control to the field-node mode machine: reset always enters POST.
  enter_mode(MODE_POST);
#endif  /* !STRIP_MESH */
}

void loop() {
#if STRIP_MESH
  // Bare bisection idle: radio already warm-slept in setup, no mesh, no mode
  // machine. Same per-poll sleep actions the deployment MODE_SLEEP applies.
#if HEARTBEAT
  Serial.print("[HB] "); Serial.println(millis());
#endif
#if SLEEP_HFCLK_STOP
  if (!(NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk)) {
    NRF_CLOCK->TASKS_HFCLKSTOP = 1;
  }
#endif
  delay(SLEEP_POLL_MS);
#else
  // Pause the mesh while the radio is SPI-slept (MODE_SLEEP) -- servicing it would
  // poll the slept radio over SPI and deadlock on BUSY. WAKE re-arms Rx before unpausing.
  if (!g_radio_slept) {
    the_mesh.loop();   // services the radio and fires processAck/onSendTimeout
  }
  rtc_clock.tick();
  mode_loop();         // field-node duty-cycle state machine (replaces displayLoop)
#endif
}
