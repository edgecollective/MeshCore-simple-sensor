// Host unit test for the sonar_field_node mode machine (pure logic, no hardware).
// Build + run on the dev machine, not the board:
//   g++ -std=c++11 -Wall -o /tmp/test_modes test_modes.cpp && /tmp/test_modes
// Validates the transition table in isolation before the .cpp hangs sonar/radio
// off it -- the "plain control" for the state machine.

#include <cstdio>
#include "modes.h"

static int checks = 0, fails = 0;

static void expect(mode got, mode want, const char *what) {
  checks++;
  if (got != want) {
    fails++;
    printf("  FAIL  %-34s got %-9s want %s\n", what, mode_name(got), mode_name(want));
  }
}

// build a mode_event; woke defaults false (the common case)
static mode_event EV(int btn, bool elapsed, bool done, bool woke = false) {
  mode_event e; e.btn = btn; e.elapsed = elapsed; e.done = done; e.woke_by_button = woke;
  return e;
}

int main() {
  const mode_event IDLE = EV(BUTTON_EVENT_NONE, false, false);

  // --- deploy path: the whole low-power loop, no button ---
  expect(next_mode(MODE_POST,     IDLE),                              MODE_POST,     "POST holds until selftest");
  expect(next_mode(MODE_POST,     EV(0, false, true)),               MODE_STATUS,   "POST done -> STATUS");
  expect(next_mode(MODE_STATUS,   IDLE),                              MODE_STATUS,   "STATUS holds");
  expect(next_mode(MODE_STATUS,   EV(0, true, false)),               MODE_SLEEP,    "STATUS 30s -> SLEEP");
  expect(next_mode(MODE_SLEEP,    IDLE),                              MODE_SLEEP,    "SLEEP holds");
  expect(next_mode(MODE_SLEEP,    EV(0, true, false)),               MODE_WAKE,     "SLEEP RTC 5min -> WAKE");
  expect(next_mode(MODE_WAKE,     EV(0, false, true)),               MODE_MEASURE,  "WAKE settled(no gesture) -> MEASURE");
  expect(next_mode(MODE_MEASURE,  IDLE),                              MODE_MEASURE,  "MEASURE holds");
  expect(next_mode(MODE_MEASURE,  EV(0, false, true)),               MODE_TRANSMIT, "MEASURE done -> TRANSMIT");
  expect(next_mode(MODE_TRANSMIT, IDLE),                              MODE_TRANSMIT, "TRANSMIT holds (retrying)");
  expect(next_mode(MODE_TRANSMIT, EV(0, false, true)),               MODE_SLEEP,    "TRANSMIT settled -> SLEEP");

  // --- The race invariant: button-wake, gesture not yet resolved ---
  expect(next_mode(MODE_SLEEP,    EV(BUTTON_EVENT_NONE, false, false, true)), MODE_STATUS, "SLEEP + button edge -> STATUS");
  expect(next_mode(MODE_WAKE,     EV(BUTTON_EVENT_NONE, false, false, true)), MODE_WAKE, "WAKE holds while classifying (no race)");

  // --- WAKE routes once the classifier commits ---
  expect(next_mode(MODE_WAKE,     EV(BUTTON_EVENT_CLICK,        false, false, true)), MODE_STATUS,         "WAKE + CLICK -> STATUS");
  expect(next_mode(MODE_WAKE,     EV(BUTTON_EVENT_DOUBLE_CLICK, false, false, true)), MODE_DEMO,           "WAKE + DOUBLE -> DEMO");
  expect(next_mode(MODE_WAKE,     EV(BUTTON_EVENT_LONG_PRESS,   false, false, true)), MODE_TRANSMIT_DEBUG, "WAKE + LONG -> TX_DEBUG");

  // --- awake gestures (peripherals already on; no wake needed) ---
  expect(next_mode(MODE_STATUS,   EV(BUTTON_EVENT_CLICK,        false, false)), MODE_STATUS,         "STATUS + CLICK -> STATUS (keepalive)");
  expect(next_mode(MODE_STATUS,   EV(BUTTON_EVENT_DOUBLE_CLICK, false, false)), MODE_DEMO,           "STATUS + DOUBLE -> DEMO");
  expect(next_mode(MODE_STATUS,   EV(BUTTON_EVENT_LONG_PRESS,   false, false)), MODE_TRANSMIT_DEBUG, "STATUS + LONG -> TX_DEBUG");
  expect(next_mode(MODE_DEMO,     EV(BUTTON_EVENT_CLICK,        false, false)), MODE_STATUS,         "DEMO + CLICK -> STATUS");
  expect(next_mode(MODE_DEMO,     EV(BUTTON_EVENT_DOUBLE_CLICK, false, false)), MODE_DEMO,           "DEMO + DOUBLE -> DEMO");
  expect(next_mode(MODE_DEMO,     EV(BUTTON_EVENT_LONG_PRESS,   false, false)), MODE_TRANSMIT_DEBUG, "DEMO + LONG -> TX_DEBUG");
  expect(next_mode(MODE_DEMO,     EV(0, true, false)),                          MODE_SLEEP,          "DEMO cycle elapsed -> SLEEP");
  expect(next_mode(MODE_TRANSMIT_DEBUG, EV(0, false, true)),                    MODE_DEMO,           "TX_DEBUG done -> DEMO");

  // --- protected modes ignore the button (don't abort a live cycle) ---
  expect(next_mode(MODE_MEASURE,  EV(BUTTON_EVENT_LONG_PRESS,   false, false)), MODE_MEASURE,  "MEASURE ignores button");
  expect(next_mode(MODE_TRANSMIT, EV(BUTTON_EVENT_CLICK,        false, false)), MODE_TRANSMIT, "TRANSMIT ignores button");
  expect(next_mode(MODE_POST,     EV(BUTTON_EVENT_DOUBLE_CLICK, false, false)), MODE_POST,     "POST ignores button");
  expect(next_mode(MODE_TRANSMIT_DEBUG, EV(BUTTON_EVENT_CLICK, false, false)),  MODE_TRANSMIT_DEBUG, "TX_DEBUG ignores button mid-attempt");

  // --- WAKE is timer-only now; settle falls through to the measurement ---
  expect(next_mode(MODE_WAKE,     EV(BUTTON_EVENT_NONE, false, true, true)),    MODE_MEASURE,  "WAKE settle -> MEASURE");
  // --- button wake from SLEEP goes straight to STATUS (no gesture dwell) ---
  expect(next_mode(MODE_SLEEP,    EV(BUTTON_EVENT_NONE, false, false, true)),   MODE_STATUS,   "SLEEP button wake -> STATUS");
  expect(next_mode(MODE_SLEEP,    EV(BUTTON_EVENT_NONE, true, false, false)),   MODE_WAKE,     "SLEEP timer wake -> WAKE");

  printf("\n%d checks, %d failures\n", checks, fails);
  return fails ? 1 : 0;
}
