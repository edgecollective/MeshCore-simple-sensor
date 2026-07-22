// gate_probe -- disambiguate a "powered but silent" MB7388 sonar.
//
// V+ and GPS_GND already metered good (3.3V across the sonar, gate conducting), yet
// gate_cmd reported NO BYTES. Three causes remain: (1) sonar wedged, (2) command
// strobe (D1->pin4) not driving it, (3) RX path (pin5->D0 via 10k) silent. This
// firmware settles all three each loop:
//   - POWER-CYCLE the sonar (gate off 1.5s, on 0.4s) to clear a wedge.
//   - CONTINUOUS test: float pin4 (D1 hi-Z -> internal pull-up) -> free-run.
//   - COMMANDED test: drive pin4 high via D1 -> range while high.
// Prints frame count AND raw byte count for each mode, so no-bytes (dead RX / sonar)
// is distinguishable from bytes-but-no-frames (baud/level) and from working.
//   CONT frames>0                -> sonar + RX good; if CMD=0 the D1 strobe is the fault
//   both 0 bytes after cycle     -> sonar wedged/dead or RX wire open
//   bytes>0 but frames=0         -> serial arriving but garbled

#include <Arduino.h>

#define PIN_SONAR_GATE 5        // D5 = P0.24 = GPS_EN = Q2 gate (HIGH = sonar on)
#define PIN_S1_RX      0        // D0 = P0.08 (MB7388 pin5 via 10k)
#define PIN_STROBE     1        // D1 = P0.06 (MB7388 pin4) -- reclaimed from Serial1 TX
#define PIN_VBAT       17       // P0.31 = AIN7 (VBAT divider)
#define ADC_MULT       1.815f
#define USB_BAUD       115200
#define SONAR_BAUD     9600

static uint16_t batt_mv() {
  analogReadResolution(12);
  uint32_t raw = 0;
  for (int i = 0; i < 8; i++) raw += analogRead(PIN_VBAT);
  raw /= 8;
  return (uint16_t)(ADC_MULT * raw);
}

// read for win_ms; count raw bytes and complete 'Rdddd\r' frames, keep last mm
static void read_window(uint32_t win_ms, int *frames, int *nbytes, int *last_mm) {
  uint32_t t0 = millis();
  int fr = 0, nb = 0, mm = -1;
  bool cap = false; char d[8]; int di = 0;
  while (millis() - t0 < win_ms) {
    while (Serial1.available()) {
      char c = (char)Serial1.read(); nb++;
      if (c == 'R') { cap = true; di = 0; }
      else if (cap && c == '\r') { d[di] = 0; if (di >= 3) { mm = atoi(d); fr++; } cap = false; di = 0; }
      else if (cap && di < 6 && c >= '0' && c <= '9') d[di++] = c;
      else if (cap) { cap = false; di = 0; }
    }
  }
  *frames = fr; *nbytes = nb; *last_mm = mm;
}

void setup() {
  Serial.begin(USB_BAUD);
  delay(500);
  Serial.println();
  Serial.println("=== gate_probe: power-cycle + continuous/commanded discriminator ===");
  pinMode(PIN_SONAR_GATE, OUTPUT); digitalWrite(PIN_SONAR_GATE, HIGH);
  Serial1.begin(SONAR_BAUD);
  NRF_UARTE0->PSEL.TXD = 0xFFFFFFFF;   // reclaim D1 from UART TX so it can strobe pin4
  delay(300);
}

void loop() {
  uint16_t mv = batt_mv();

  // 1) power-cycle the sonar to clear a possible wedge
  digitalWrite(PIN_SONAR_GATE, LOW);   // gate off -> sonar loses its ground return
  delay(1500);                         // let the ~100uF discharge for a real power-down
  digitalWrite(PIN_SONAR_GATE, HIGH);  // gate on
  delay(400);                          // sonar power-up settle

  // 2) continuous: pin4 floats (D1 hi-Z) -> sonar internal pull-up -> free-run
  pinMode(PIN_STROBE, INPUT);
  while (Serial1.available()) Serial1.read();
  int cf, cb, cmm; read_window(1500, &cf, &cb, &cmm);

  // 3) commanded: drive pin4 high via D1 -> range while high
  pinMode(PIN_STROBE, OUTPUT); digitalWrite(PIN_STROBE, LOW);
  delay(100);
  while (Serial1.available()) Serial1.read();
  digitalWrite(PIN_STROBE, HIGH);
  int df, db, dmm; read_window(800, &df, &db, &dmm);
  digitalWrite(PIN_STROBE, LOW);

  Serial.print("batt="); Serial.print(mv); Serial.print("mV  ");
  Serial.print("CONT(float): frames="); Serial.print(cf); Serial.print(" bytes="); Serial.print(cb);
  if (cmm >= 0) { Serial.print(" last="); Serial.print(cmm); Serial.print("mm"); }
  Serial.print("   CMD(D1-high): frames="); Serial.print(df); Serial.print(" bytes="); Serial.print(db);
  if (dmm >= 0) { Serial.print(" last="); Serial.print(dmm); Serial.print("mm"); }
  Serial.println();

  delay(500);
}
