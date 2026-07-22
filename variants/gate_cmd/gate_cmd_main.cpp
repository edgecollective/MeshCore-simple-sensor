// gate_cmd -- battery voltage read + commanded (single-shot) ranging via pin4/D1.
//
// Adds two things to the proven low-side gate:
//   1. Battery voltage via the VBAT divider (Arduino pin 17 = P0.31 = AIN7,
//      ADC_MULT 1.815 -- same as RookBoard.getBattMilliVolts). Reads ~4100 mV
//      with the LiPo switched on, near 0 with the battery OFF -> the firmware can
//      SEE "forgot the battery" / low battery instead of failing silently.
//   2. Commanded ranging: MB7388 pin4 (Ranging Start/Stop) wired to D1. Held LOW
//      stops the free-run; a >20us HIGH pulse commands exactly one reading, which
//      the sonar reports on pin5 (Serial1 RX / D0). We never UART-TX to the sonar,
//      so D1 is reclaimed from Serial1 TX as a plain GPIO strobe.
//
// Each period: read+print battery, confirm pin4-low actually stopped the free-run
// (unsolicited frame count should be ~0), strobe once, read the single R#### back.
// unsolicited>0 => pin4 isn't controlling (strobe wiring / D1-reclaim failed);
// cmd->NO RESPONSE => strobe or read path broken.

#include <Arduino.h>

#define PIN_SONAR_GATE 5        // D5 = P0.24 = GPS_EN = Q2 gate (HIGH = sonar on)
#define PIN_S1_RX      0        // D0 = P0.08 (MB7388 pin5 via 10k)
#define PIN_STROBE     1        // D1 = P0.06 (MB7388 pin4) -- reclaimed from Serial1 TX
#define PIN_VBAT       17       // P0.31 = AIN7 (VBAT divider)
#define ADC_MULT       1.815f
#define USB_BAUD       115200
#define SONAR_BAUD     9600
#define STROBE_US      30       // >20us commands one reading
#define RESP_TIMEOUT_MS 250
#define PERIOD_MS      3000

// build knobs (override in platformio.ini build_flags with -D):
#ifndef USE_MEDIAN
#define USE_MEDIAN     1        // 1 = median-filter N samples/measurement; 0 = raw single reading (debug)
#endif
#ifndef N_SAMPLES
#define N_SAMPLES      5        // samples per measurement when USE_MEDIAN
#endif

static uint32_t g_no_target = 0;   // cumulative 9999 (no-target) samples seen -- glitch-rate telemetry

static uint16_t batt_mv() {
  analogReadResolution(12);
  uint32_t raw = 0;
  for (int i = 0; i < 8; i++) raw += analogRead(PIN_VBAT);
  raw /= 8;
  return (uint16_t)(ADC_MULT * raw);
}

// parse one 'Rdddd\r' frame within timeout; return mm, or -1 on timeout
static int read_frame(uint32_t timeout_ms) {
  uint32_t t0 = millis();
  bool cap = false; char d[8]; int di = 0;
  while (millis() - t0 < timeout_ms) {
    while (Serial1.available()) {
      char c = (char)Serial1.read();
      if (c == 'R') { cap = true; di = 0; }
      else if (cap && c == '\r') { d[di] = 0; if (di >= 3) return atoi(d); cap = false; di = 0; }
      else if (cap && di < 6 && c >= '0' && c <= '9') d[di++] = c;
      else if (cap) { cap = false; di = 0; }
    }
  }
  return -1;
}

// count self-initiated frames over a window (should be ~0 with pin4 held low)
static unsigned count_unsolicited(uint32_t win_ms) {
  uint32_t t0 = millis(); unsigned n = 0;
  bool cap = false; char d[8]; int di = 0;
  while (millis() - t0 < win_ms) {
    while (Serial1.available()) {
      char c = (char)Serial1.read();
      if (c == 'R') { cap = true; di = 0; }
      else if (cap && c == '\r') { d[di] = 0; if (di >= 3) n++; cap = false; di = 0; }
      else if (cap && di < 6 && c >= '0' && c <= '9') d[di++] = c;
      else if (cap) { cap = false; di = 0; }
    }
  }
  return n;
}

void setup() {
  Serial.begin(USB_BAUD);
  delay(500);
  Serial.println();
  Serial.println("=== gate_cmd: battery + commanded ranging ===");

  uint16_t mv = batt_mv();
  Serial.print("battery: "); Serial.print(mv); Serial.print(" mV ");
  Serial.println(mv < 3000 ? "(LOW / battery OFF?)" : "(ok)");

  pinMode(PIN_SONAR_GATE, OUTPUT); digitalWrite(PIN_SONAR_GATE, HIGH);  // gate on
  Serial1.begin(SONAR_BAUD);                                           // D0 = RX (UARTE0)
  NRF_UARTE0->PSEL.TXD = 0xFFFFFFFF;   // detach UART TX from D1 so it can be a GPIO strobe (RXD/D0 stays live)
  delay(300);                                                          // sonar power-up

  // POSITIVE CONTROL: float pin4 (D1 hi-Z) -> internal pull-up -> continuous mode.
  // frames>0 => sonar alive + serial path good + pin4-float=continuous. frames=0
  // => sonar silent (disturbed), no command would work either.
  pinMode(PIN_STROBE, INPUT);
  delay(200);
  unsigned fr = count_unsolicited(3000);
  Serial.print("float-test (pin4 hi-Z = continuous): frames in 3s = "); Serial.println(fr);

  pinMode(PIN_STROBE, OUTPUT); digitalWrite(PIN_STROBE, LOW);          // back to commanded (pin4 low = stop)
  delay(200);
}

// median of n ints (n small); insertion-sorts a scratch copy, returns the middle.
// 9999 (no-target) participates: it sorts to the top, so a minority of no-target
// glitches is outvoted; only a majority makes the median 9999 (correctly no-target).
static int median_int(int *a, int n) {
  for (int i = 1; i < n; i++) {
    int v = a[i], j = i - 1;
    while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
    a[j + 1] = v;
  }
  return a[n / 2];
}

void loop() {
  uint16_t mv = batt_mv();

  unsigned unsol = count_unsolicited(200);          // pin4-low should give ~0

  while (Serial1.available()) Serial1.read();        // flush
  digitalWrite(PIN_STROBE, HIGH);                    // pin4 high -> continuous ranging while high

#if USE_MEDIAN
  int want = N_SAMPLES;
#else
  int want = 1;
#endif
  // grab up to `want` consecutive Rdddd frames; tally raw 9999s as we go.
  int samp[N_SAMPLES]; int ns = 0, nbytes = 0;
  uint32_t t0 = millis();
  bool cap = false; char d[8]; int di = 0;
  while (ns < want && millis() - t0 < 1500) {
    while (Serial1.available()) {
      char c = (char)Serial1.read(); nbytes++;
      if (c == 'R') { cap = true; di = 0; }
      else if (cap && c == '\r') { d[di] = 0; if (di >= 3) { int v = atoi(d); samp[ns++] = v; if (v == 9999) g_no_target++; } cap = false; di = 0; }
      else if (cap && di < 6 && c >= '0' && c <= '9') d[di++] = c;
      else if (cap) { cap = false; di = 0; }
    }
  }
  digitalWrite(PIN_STROBE, LOW);                     // pin4 low -> stop free-run again

#if USE_MEDIAN
  int reading = ns ? median_int(samp, ns) : -1;
#else
  int reading = ns ? samp[0] : -1;
#endif

  Serial.print("batt="); Serial.print(mv); Serial.print("mV");
  Serial.print(mv < 3000 ? "(!) " : "  ");
  Serial.print("unsol="); Serial.print(unsol);
  Serial.print("  n="); Serial.print(ns);
#if USE_MEDIAN
  Serial.print(" [");                                // raw samples, so the filter is auditable on the bench
  for (int i = 0; i < ns; i++) { if (i) Serial.print(' '); Serial.print(samp[i]); }
  Serial.print("]");
#endif
  Serial.print(USE_MEDIAN ? "  med->" : "  raw->");
  if (reading >= 0) { Serial.print(reading); Serial.print("mm ("); Serial.print(reading / 1000.0f, 3); Serial.print(" m)"); }
  else Serial.print(nbytes ? "bytes but no frame" : "NO BYTES (dead serial)");
  Serial.print("  9999seen="); Serial.print(g_no_target);
  Serial.println();

  delay(PERIOD_MS);
}
