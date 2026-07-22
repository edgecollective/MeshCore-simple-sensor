// sonar_oled_demo -- live median-filtered MB7388 range on the SSD1306 OLED, ~1 Hz.
// Demo firmware: gate held ON (D5), commanded ranging (pin4/D1), median of N frames,
// shown big on the 128x64 OLED (0x3C, Wire SDA=8/SCL=7). Display stays on. No sleep.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define PIN_SONAR_GATE 5        // D5 = GPS_EN = Q2 gate (HIGH = sonar on)
#define PIN_STROBE     1        // D1 = MB7388 pin4 (reclaimed from Serial1 TX)
#define SONAR_BAUD     9600
#define N_SAMPLES      5
#define OLED_ADDR      0x3C
#define PIN_VBAT       17       // P0.31 = AIN7 (VBAT divider)
#define ADC_MULT       1.815f

Adafruit_SSD1306 oled(128, 64, &Wire, -1);   // PIN_OLED_RESET = -1 (none)

// median of n ints (n small); insertion sort, return the middle. 9999 (no-target)
// participates so a lone glitch is outvoted; a majority of 9999 stays "no target".
static int median_int(int *a, int n) {
  for (int i = 1; i < n; i++) {
    int v = a[i], j = i - 1;
    while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
    a[j + 1] = v;
  }
  return a[n / 2];
}

// battery mV via the VBAT divider (avg 8 reads x 1.815, per RookBoard.getBattMilliVolts).
static uint16_t batt_mv() {
  analogReadResolution(12);
  uint32_t raw = 0;
  for (int i = 0; i < 8; i++) raw += analogRead(PIN_VBAT);
  raw /= 8;
  return (uint16_t)(ADC_MULT * raw);
}

// hold pin4 high, collect up to N consecutive Rdddd frames, return their median (mm),
// or -1 if nothing arrived.
static int read_median() {
  while (Serial1.available()) Serial1.read();          // flush
  digitalWrite(PIN_STROBE, HIGH);                      // pin4 high -> ranging
  int samp[N_SAMPLES]; int ns = 0;
  uint32_t t0 = millis();
  bool cap = false; char d[8]; int di = 0;
  while (ns < N_SAMPLES && millis() - t0 < 1000) {
    while (Serial1.available()) {
      char c = (char)Serial1.read();
      if (c == 'R') { cap = true; di = 0; }
      else if (cap && c == '\r') { d[di] = 0; if (di >= 3) samp[ns++] = atoi(d); cap = false; di = 0; }
      else if (cap && di < 6 && c >= '0' && c <= '9') d[di++] = c;
      else if (cap) { cap = false; di = 0; }
    }
  }
  digitalWrite(PIN_STROBE, LOW);                       // pin4 low -> stop
  return ns ? median_int(samp, ns) : -1;
}

void setup() {
  pinMode(PIN_SONAR_GATE, OUTPUT); digitalWrite(PIN_SONAR_GATE, HIGH);   // sonar on
  Serial1.begin(SONAR_BAUD);
  NRF_UARTE0->PSEL.TXD = 0xFFFFFFFF;                   // reclaim D1 as a GPIO strobe
  pinMode(PIN_STROBE, OUTPUT); digitalWrite(PIN_STROBE, LOW);

  Wire.begin();                                        // SDA=8, SCL=7 (variant defaults)
  oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, true, false);
  oled.clearDisplay(); oled.display();
  delay(300);                                          // sonar power-up settle
}

void loop() {
  int mm = read_median();
  uint16_t mv = batt_mv();

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0); oled.print("SONAR");
  char bb[12]; snprintf(bb, sizeof(bb), "%u.%02uV", mv / 1000, (mv % 1000) / 10);
  oled.setCursor(128 - 6 * (int)strlen(bb), 0); oled.print(bb);   // battery, right-aligned

  char buf[16];
  if (mm < 0)          snprintf(buf, sizeof(buf), "--.--");
  else if (mm == 9999) snprintf(buf, sizeof(buf), "no tgt");
  else                 snprintf(buf, sizeof(buf), "%d.%02dm", mm / 1000, (mm % 1000) / 10);
  oled.setTextSize(3); oled.setCursor(0, 24);
  oled.print(buf);

  oled.setTextSize(1); oled.setCursor(0, 56);
  if (mm >= 0 && mm != 9999) { oled.print(mm); oled.print(" mm"); }
  else oled.print(mm < 0 ? "(no serial)" : "(9999)");
  oled.display();

  delay(1000);
}
