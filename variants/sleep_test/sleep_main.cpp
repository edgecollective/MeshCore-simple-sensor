// v4: SX1262 SPI sleep + nRF52840 System OFF, for JS220 sleep profiling (Rook).
//
// v1 (System OFF) 2.82 mA; v2/v3 (+ POWER_EN low + USB off + cold cycle) stuck flat at
// 1.05 mA, voltage-INDEPENDENT -> a load behind a regulator that never got the sleep
// command. Hypothesis: POWER_EN low only killed the TCXO/RF-switch (the 1.77 mA we saw
// drop); the SX1262 digital core is still in standby (~1 mA). Only a SPI SLEEP opcode
// (RadioLib radio.sleep()) puts the core to ~1 uA. v4 powers the radio, inits it, sleeps
// it over SPI, then sleeps the MCU.

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

#define P_LORA_NSS   13
#define P_LORA_DIO_1 11
#define P_LORA_RESET 10
#define P_LORA_BUSY  16
#define P_LORA_MISO  15
#define P_LORA_SCLK  12
#define P_LORA_MOSI  14
#define SX126X_POWER_EN 21       // P0.13 active-HIGH radio module enable
#define SX126X_RXEN      2       // P0.17 RF switch RX

SPIClass spiLora(NRF_SPIM3, P_LORA_MISO, P_LORA_SCLK, P_LORA_MOSI);
SX1262 radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, spiLora);

void setup() {
  delay(3000);                                  // awake window (boot blip)

  pinMode(SX126X_POWER_EN, OUTPUT); digitalWrite(SX126X_POWER_EN, HIGH);  // power radio to talk to it
  pinMode(SX126X_RXEN, OUTPUT);     digitalWrite(SX126X_RXEN, LOW);
  delay(10);

  spiLora.begin();
  // freq, bw, sf, cr, sync, power, preamble, tcxoVoltage=1.8 (Rook DIO3 TCXO), useLDO=false
  radio.begin(910.525, 62.5, 7, 5, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 22, 8, 1.8, false);
  radio.setDio2AsRfSwitch(true);
  radio.sleep();                                // SX1262 -> deep sleep (~1 uA)

  // ---- System-ON build-up knobs (v5) ---------------------------------
  // V5_SYSTEMON: stay in System ON with a delay(1000) idle loop (the deployment
  //   firmware's sleep shape) instead of SYSTEMOFF. THE bridge build: measures the
  //   bare Adafruit-core System-ON floor with the radio slept.
  // V5_USBD_ON: leave USBD enabled (undo the v4 release) -- cross-checks the
  //   deployment USBD null in the minimal context.
  // V5_PRINTS: one Serial line per loop pass (CDC writes with no host).
  // V5_FSRTC: InternalFS.begin() (flash service cost in isolation).
  // V5_D21_LOW: drive D21/P0.13 LOW -- the clone EXT-VCC pull-up A/B; expect
  //   ~+0.6 mA if the SuperMini-style 5.6 k is populated.
#ifndef V5_SYSTEMON
#define V5_SYSTEMON 0
#endif
#ifndef V5_USBD_ON
#define V5_USBD_ON 0
#endif
#ifndef V5_PRINTS
#define V5_PRINTS 0
#endif
#ifndef V5_FSRTC
#define V5_FSRTC 0
#endif
#ifndef V5_D21_LOW
#define V5_D21_LOW 0
#endif
  // ---- idle-surgery knobs (v6, each also sets V5_SYSTEMON) ----
  // V6_AUDIT: latch clock/peripheral enable state + idle-hook rate each pass on
  //   battery; dump the latched battery snapshot over CDC whenever USB is back
  //   (attach does not reset the chip, so the snapshot survives into readout).
  // V6_HFCLKSTOP: idle hook issues TASKS_HFCLKSTOP when VBUS is absent -- tests
  //   the nobody-ever-stops-HFXO fact directly.
  // V6_USBTASK_SUSPEND: once VBUS is gone, find the "usbd" task by name and
  //   suspend it (TRACE_FACILITY walk; core discards the handle at create).
  // V6_NOUSB: built with build_unflags stripping USE_TINYUSB/USBCON -- no usb
  //   task, no CDC ever. Alive gate = LED: triple blink at boot, then a 50 ms
  //   flash every 30 s (the JS220 trace shows the cadence; floor comes from the
  //   quiet windows between flashes).
#ifndef V6_AUDIT
#define V6_AUDIT 0
#endif
#ifndef V6_HFCLKSTOP
#define V6_HFCLKSTOP 0
#endif
#ifndef V6_USBTASK_SUSPEND
#define V6_USBTASK_SUSPEND 0
#endif
#ifndef V6_NOUSB
#define V6_NOUSB 0
#endif
  // V6_SPIM3_OFF: spiLora.end() after radio.sleep() -- nRF52840 anomaly 195 is
  //   ~900 uA continuous while SPIM3 is ENABLED. RadioLib's endTransaction()
  //   already calls nrf_spim_disable, so this may be a no-op; the audit arm
  //   reads NRF_SPIM3->ENABLE to say which. Single knob either way.
#ifndef V6_SPIM3_OFF
#define V6_SPIM3_OFF 0
#endif
  // V6_AUDIT2: audit + DWT CYCCNT awake-cycles-per-pass latch. CYCCNT clocks
  //   only while the CPU runs, so the per-second delta is a direct measure of
  //   how much the WFE micro-loop actually sleeps (invisible to the hook count).
  //   Env sets V6_AUDIT=1 too. Pure diagnostic, no cuts.
#ifndef V6_AUDIT2
#define V6_AUDIT2 0
#endif
  // V6_AUDIT3: + the blind-spot registers: GPIOTE CONFIG[0..7] (hi-acc IN
  //   channels hold HFCLK -- the attachInterrupt trap), CRYPTOCELL, QDEC,
  //   COMP, LPCOMP, AAR, CCM, MWU regions, RADIO state.
#ifndef V6_AUDIT3
#define V6_AUDIT3 0
#endif
  // V6_LOWPWR: TASKS_LOWPWR once when VBUS drops -- insurance against a
  //   bootloader-latched constant-latency mode (no readable status exists).
#ifndef V6_LOWPWR
#define V6_LOWPWR 0
#endif

#if V6_AUDIT2
  dwt_enable();                                 // core delay.h: CYCCNT on
#endif
  // V6_TIMERSTOP: stop+shutdown TIMER0-4 at setup. Timers have no ENABLE
  //   readback (audit-blind) and a running timer holds HFCLK through CPU
  //   sleep; the UF2 bootloader runs before every app boot and could leave
  //   one started. Single knob for the bootloader-residue hypothesis.
#ifndef V6_TIMERSTOP
#define V6_TIMERSTOP 0
#endif
  // V6_SD: Bluefruit.begin() -- enable the (already-linked) S140 SoftDevice so
  //   the port's tickless idle takes the sd_app_evt_wait() branch instead of
  //   the bare WFE loop. No advertising, no connections; just SD-managed sleep.
#ifndef V6_SD
#define V6_SD 0
#endif
  // V6_RESLEEP: re-issue radio.sleep() every loop pass. A sleeping SX1262
  //   wakes on any NSS edge and the module is permanently powered (clone
  //   pull-up on EXT-VCC); the earlier ladder showed radio-not-slept == 1.05 mA, our exact
  //   floor. If something in the System-ON runtime wakes it, holding it down
  //   collapses the floor; if not, the radio is innocent and the cost is MCU.
#ifndef V6_RESLEEP
#define V6_RESLEEP 0
#endif
#if V6_SD
  { extern void v6_sd_begin(); v6_sd_begin(); }
#endif
#if V6_TIMERSTOP
  {
    NRF_TIMER_Type* t[5] = {NRF_TIMER0, NRF_TIMER1, NRF_TIMER2, NRF_TIMER3, NRF_TIMER4};
    for (int i = 0; i < 5; i++) { t[i]->TASKS_STOP = 1; t[i]->TASKS_SHUTDOWN = 1; }
  }
#endif

#if V6_SPIM3_OFF
  spiLora.end();                                // SPIM3 ENABLE=0 (anomaly 195)
#endif

#if V6_NOUSB
  pinMode(LED_PIN, OUTPUT);                     // boot gate: triple blink
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW);  delay(200);
  }
#endif

#if V5_FSRTC
  extern int _v5_fs_begin();                    // defined below to keep v4 path clean
  _v5_fs_begin();
#endif
#if V5_D21_LOW
  pinMode(21, OUTPUT); digitalWrite(21, LOW);   // D21 = P0.13, clone EXT-VCC control
#endif

#if !V5_USBD_ON && !V5_SYSTEMON
  NRF_USBD->ENABLE = 0;                         // release USB (v4 behavior)
#endif
  // (System-ON arms release USBD per loop pass once VBUS is gone -- CDC stays
  // alive for the USB heartbeat check, and the battery floor matches v4.)

#if V5_SYSTEMON
  // v5: System ON idle, tickless FreeRTOS delay -- fall through to loop()
#else
  NRF_POWER->SYSTEMOFF = 1;                     // nRF System OFF (v4 behavior)
  while (1) { __WFE(); }
#endif
}

#if V5_FSRTC
#include <InternalFileSystem.h>
int _v5_fs_begin() { InternalFS.begin(); return 0; }
#endif

#if V6_SD
#include <bluefruit.h>
void v6_sd_begin() { Bluefruit.begin(); }   // S140 on; idle now sd_app_evt_wait
#endif

// ---- v6 file-scope machinery (knob macros defined above in setup's text) ----

#if V6_AUDIT || V6_HFCLKSTOP
static volatile uint32_t v6_idle_count;
extern "C" void vApplicationIdleHook(void) {  // weak in core hooks.c
  v6_idle_count++;
#if V6_HFCLKSTOP
  if (!(NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk)) {
    NRF_CLOCK->TASKS_HFCLKSTOP = 1;
  }
#endif
}
#endif

#if V6_USBTASK_SUSPEND
static void v6_suspend_usbd_once() {
  static bool done;
  if (done) return;
  TaskStatus_t st[8];
  UBaseType_t n = uxTaskGetSystemState(st, 8, NULL);
  for (UBaseType_t i = 0; i < n; i++) {
    if (0 == strcmp(st[i].pcTaskName, "usbd")) {
      vTaskSuspend(st[i].xHandle);
      done = true;
      return;
    }
  }
}
#endif

#if V6_AUDIT
// last battery-idle snapshot; survives USB replug (attach doesn't reset)
static struct {
  uint32_t passes, idle_rate;
  uint32_t hfclkstat, lfclkstat, usbreg, usbd_en;
  uint32_t uarte0, uarte1, box0, box1, spim2, spim3;
  uint32_t saadc, qspi, pwm0, pwm1, pwm2, pwm3, pdm, i2s;
  uint32_t cyc;                           // V6_AUDIT2: awake cycles per pass
#if V6_AUDIT3
  uint32_t gpiote[8];                     // hi-acc IN channels hold HFCLK
  uint32_t cryptocell, qdec, comp, lpcomp, aar, ccm, mwu_regionen, radio_state;
#endif
} v6b;

static void v6_audit_pass(bool vbus) {
  static uint32_t last;
  uint32_t rate = v6_idle_count - last;   // idle-hook calls per ~1 s pass
  last = v6_idle_count;
#if V6_AUDIT2
  static uint32_t last_cyc;
  uint32_t cyc_now = DWT->CYCCNT;
  uint32_t cyc = cyc_now - last_cyc;      // counts only while CPU runs
  last_cyc = cyc_now;
#endif
  if (!vbus) {
    v6b.passes++; v6b.idle_rate = rate;
#if V6_AUDIT2
    v6b.cyc = cyc;
#endif
    v6b.hfclkstat = NRF_CLOCK->HFCLKSTAT; v6b.lfclkstat = NRF_CLOCK->LFCLKSTAT;
    v6b.usbreg = NRF_POWER->USBREGSTATUS; v6b.usbd_en = NRF_USBD->ENABLE;
    v6b.uarte0 = NRF_UARTE0->ENABLE;      v6b.uarte1 = NRF_UARTE1->ENABLE;
    v6b.box0 = NRF_SPIM0->ENABLE;         v6b.box1 = NRF_SPIM1->ENABLE;
    v6b.spim2 = NRF_SPIM2->ENABLE;        v6b.spim3 = NRF_SPIM3->ENABLE;
    v6b.saadc = NRF_SAADC->ENABLE;        v6b.qspi = NRF_QSPI->ENABLE;
    v6b.pwm0 = NRF_PWM0->ENABLE; v6b.pwm1 = NRF_PWM1->ENABLE;
    v6b.pwm2 = NRF_PWM2->ENABLE; v6b.pwm3 = NRF_PWM3->ENABLE;
    v6b.pdm = NRF_PDM->ENABLE;   v6b.i2s = NRF_I2S->ENABLE;
#if V6_AUDIT3
    for (int i = 0; i < 8; i++) v6b.gpiote[i] = NRF_GPIOTE->CONFIG[i];
    v6b.cryptocell = NRF_CRYPTOCELL->ENABLE;
    v6b.qdec = NRF_QDEC->ENABLE;   v6b.comp = NRF_COMP->ENABLE;
    v6b.lpcomp = NRF_LPCOMP->ENABLE;
    v6b.aar = NRF_AAR->ENABLE;     v6b.ccm = NRF_CCM->ENABLE;
    v6b.mwu_regionen = NRF_MWU->REGIONEN;
    v6b.radio_state = NRF_RADIO->STATE;
#endif
  } else {
    // on USB: report the battery snapshot (if any) then live state
    Serial.print("[AUDIT] batt_passes="); Serial.print(v6b.passes);
    Serial.print(" idle_rate=");          Serial.print(v6b.idle_rate);
    Serial.print(" hfclkstat=0x");        Serial.print(v6b.hfclkstat, HEX);
    Serial.print(" lfclkstat=0x");        Serial.print(v6b.lfclkstat, HEX);
    Serial.print(" usbd=");               Serial.print(v6b.usbd_en);
    Serial.println();
    Serial.print("[AUDIT] uarte0/1=");    Serial.print(v6b.uarte0);
    Serial.print("/");                    Serial.print(v6b.uarte1);
    Serial.print(" box0/1=");             Serial.print(v6b.box0);
    Serial.print("/");                    Serial.print(v6b.box1);
    Serial.print(" spim2/3=");            Serial.print(v6b.spim2);
    Serial.print("/");                    Serial.print(v6b.spim3);
    Serial.print(" saadc=");              Serial.print(v6b.saadc);
    Serial.print(" qspi=");               Serial.print(v6b.qspi);
    Serial.print(" pwm=");                Serial.print(v6b.pwm0);
    Serial.print(v6b.pwm1); Serial.print(v6b.pwm2); Serial.print(v6b.pwm3);
    Serial.print(" pdm=");                Serial.print(v6b.pdm);
    Serial.print(" i2s=");                Serial.print(v6b.i2s);
    Serial.print(" live_idle_rate=");     Serial.print(rate);
    Serial.print(" live_hfclkstat=0x");   Serial.print(NRF_CLOCK->HFCLKSTAT, HEX);
    Serial.print(" live_spim3=");         Serial.print(NRF_SPIM3->ENABLE);
#if V6_AUDIT2
    Serial.print(" batt_cyc=");           Serial.print(v6b.cyc);
    Serial.print(" live_cyc=");           Serial.print(cyc);
#endif
    Serial.println();
#if V6_AUDIT3
    Serial.print("[AUDIT3] gpiote=");
    for (int i = 0; i < 8; i++) { Serial.print(v6b.gpiote[i], HEX); Serial.print(i < 7 ? "," : ""); }
    Serial.print(" cc310=");   Serial.print(v6b.cryptocell);
    Serial.print(" qdec=");    Serial.print(v6b.qdec);
    Serial.print(" comp=");    Serial.print(v6b.comp);
    Serial.print(" lpcomp=");  Serial.print(v6b.lpcomp);
    Serial.print(" aar=");     Serial.print(v6b.aar);
    Serial.print(" ccm=");     Serial.print(v6b.ccm);
    Serial.print(" mwu=0x");   Serial.print(v6b.mwu_regionen, HEX);
    Serial.print(" radio_state="); Serial.print(v6b.radio_state);
    Serial.print(" live_gpiote0=0x"); Serial.print(NRF_GPIOTE->CONFIG[0], HEX);
    Serial.println();
#endif
  }
}
#endif

void loop() {
#if V5_SYSTEMON
#if V6_NOUSB
  // no USB stack in this build: 50 ms LED flash every 30 s is the alive +
  // RTC-cadence gate (visible in the JS220 trace); dark otherwise
  static uint32_t pass;
  if (++pass % 30 == 0) {
    digitalWrite(LED_PIN, HIGH); delay(50); digitalWrite(LED_PIN, LOW);
    delay(950);
  } else {
    delay(1000);
  }
#else
  bool vbus = NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk;
#if !V5_USBD_ON
  if (!vbus && NRF_USBD->ENABLE) {
    NRF_USBD->ENABLE = 0;    // USB pulled mid-run: now match v4's battery state
  }
#endif
#if V6_USBTASK_SUSPEND
  if (!vbus) v6_suspend_usbd_once();
#endif
#if V6_LOWPWR
  if (!vbus) NRF_POWER->TASKS_LOWPWR = 1;   // idempotent; clears any CONSTLAT
#endif
#if V6_AUDIT
  v6_audit_pass(vbus);
#endif
  // Heartbeat on USB always (the alive-check), on battery only for the
  // V5_PRINTS arm -- keeps the print-cost delta clean on the battery floor.
#if !V5_PRINTS
  if (vbus)
#endif
  { Serial.print("[HB] "); Serial.println(millis()); }
#if V6_RESLEEP
  radio.sleep();                // hold the SX1262 down every pass
#endif
  delay(1000);
#endif
#endif
}
