#include <Arduino.h>   // needed for PlatformIO
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

/* ---------------------------------- CONFIGURATION ------------------------------------- */

#define FIRMWARE_VER_TEXT   "companion_sensor v3-ultrasonic (build: May 2026) [v3 + MaxBotix MB7388 → distance_meters]"

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
// xxxx is millimeters; range 300 mm – 5000 mm. 0 / out-of-range readings are
// reported by the sensor itself (e.g. "R5000" for max-out).
static bool has_sensor = false;
static unsigned long sensor_last_frame_at = 0;

// Cached sensor readings (updated each send interval)
static float last_dist_m = 0;     // meters, parsed from MaxBotix
static float last_batt = 0;
static bool has_reading = false;

// v3: last-send ACK status — surfaced on the OLED status page so a button-triggered
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
    int last_mm = -1;
    char digits[8];
    int di = 0;
    bool capturing = false;

    unsigned long start = millis();
    while ((millis() - start) < ULTRASONIC_READ_TIMEOUT_MS) {
      while (Serial1.available()) {
        char c = (char)Serial1.read();
        if (c == 'R') {
          capturing = true;
          di = 0;
        } else if (capturing) {
          if (c == '\r') {
            digits[di] = 0;
            if (di >= 3) {  // accept 3 or 4 digit frames
              last_mm = atoi(digits);
              sensor_last_frame_at = millis();
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
      delay(5);
    }

    if (last_mm < 0) {
      Serial.println("   MaxBotix: no frame within timeout");
      return -1.0f;
    }
    return (float)last_mm / 1000.0f;
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

    if (target_count == 0) {
      Serial.println("   No targets set, skipping send (use 'target add <name>')");
      return 0;
    }

    updateSensorReadings();

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
        // Cap at what fits in 47 chars: 15 hops × "xx." = 45 chars + "xx" = 47. Path_len max
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

  // Read the sensor + battery and cache the values WITHOUT sending — used by
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
    // timeout fired — treat this as a late-but-no-op timeout and skip invalidation.
    // Without this guard, a slow-but-successful FLOOD round-trip would invalidate
    // the path we just learned from the receiver's PATH-return.
    if (expected_ack_crc == 0) return;
    last_ack_status = ACK_STATUS_TIMEOUT;

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

    // Check if it's time to send a sensor reading
    uint32_t now = getRTCClock()->getCurrentTime();
    if (now > 0 && (last_send_time == 0 || (now - last_send_time) >= send_interval_secs)) {
      updateSensorReadings();
      Serial.printf("[SENSOR] node_id=%u dist=%.3fm batt=%.2fV\n", (unsigned)node_id, last_dist_m, last_batt);
      sendSensorReading();
      last_send_time = now;
    }

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

static void renderStatusPage(DisplayDriver& d) {
  d.setTextSize(1);
  d.setColor(DisplayDriver::LIGHT);

  // Node name and ID
  char buf[32];
  snprintf(buf, sizeof(buf), "%s [%u]", the_mesh.getNodeName(), (unsigned)the_mesh.getNodeId());
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

  // v3: last ACK status. After a button-triggered send, this is the user's
  // "did the receiver hear me?" feedback: sending… → ACK Xs/Xm/Xh ago / no ACK.
  // Falls back to the send interval when no send has happened this boot.
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
    display.turnOn();
    next_display_refresh = 0;  // force immediate refresh
    return;  // consume this press just to turn on the screen
  }

  if (ev == BUTTON_EVENT_CLICK) {
    // Short press: cycle to next page
    current_page = (current_page + 1) % PAGE_COUNT;
    next_display_refresh = 0;  // force immediate refresh

    // Landing on the Send page kicks off an immediate sensor read so the page
    // shows a live preview — easy way to verify the sensor is alive without
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
        // No `target add …` has been done yet — make this very explicit.
        showAlert("Target not set");
      } else {
        // Switch to status page so the user can watch the "Last: …" line cycle
        // through sending → ACK Nms / no ACK. Overlay a brief alert as immediate
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
    display.turnOff();
  }
}
#endif  // DISPLAY_CLASS

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

  Serial.println("setup: board.begin()...");
  board.begin();

  Serial.println("setup: radio_init()...");
  if (!radio_init()) {
    Serial.println("setup: radio_init() FAILED — halting.");
    halt();
  }
  Serial.println("setup: radio OK.");

  fast_rng.begin(radio_get_rng_seed());

  // Initialize MaxBotix MB7388 (TTL serial, 9600 baud, "Rxxxx\r" frames).
  // Wire the MaxBotix pin 5 (serial TX) to the Rook's Serial1 RX pin; no TX
  // wire back is needed. Power: 3.0–5.5 V on pin 6, ground on pin 7.
  Serial.println("setup: probing MaxBotix on Serial1 @ 9600 baud...");
#if defined(ESP32) && defined(ULTRASONIC_RX_PIN)
  Serial1.begin(ULTRASONIC_BAUD, SERIAL_8N1, ULTRASONIC_RX_PIN, -1);
#else
  Serial1.begin(ULTRASONIC_BAUD);
#endif
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
  }

#ifdef DISPLAY_CLASS
  if (display.begin()) {
    display.startFrame();
    display.setCursor(0, 0);
    display.print("Starting...");
    display.endFrame();
  }
#endif

  Serial.println("setup: filesystem.begin()...");
#if defined(NRF52_PLATFORM)
  InternalFS.begin();
  Serial.println("setup: the_mesh.begin() — if first-boot, will block here on 'Press ENTER to generate key:'");
  the_mesh.begin(InternalFS);
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  the_mesh.begin(LittleFS);
#elif defined(ESP32)
  SPIFFS.begin(true);
  the_mesh.begin(SPIFFS);
#else
  #error "need to define filesystem"
#endif
  Serial.println("setup: mesh ready.");

  radio_set_params(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR);
  radio_set_tx_power(LORA_TX_POWER);

  the_mesh.showWelcome();
  Serial.println("setup: done, entering loop().");

#ifdef DISPLAY_CLASS
  last_button_activity = millis();
#endif

  // Send out initial advertisement
#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvert(1200);
#endif
}

void loop() {
  the_mesh.loop();
#ifdef DISPLAY_CLASS
  displayLoop();
#endif
  rtc_clock.tick();
}
