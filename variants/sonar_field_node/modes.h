#pragma once
// sonar_field_node mode machine -- pure state logic, no hardware.
// Kept hardware-free so it compiles and unit-tests on the host; the .cpp
// wires each mode to the sonar/OLED/mesh/RTC primitives.
//
// Deploy path (default, low power):
//   reset -> POST -> STATUS(30s) -> SLEEP -> (5min) -> WAKE -> MEASURE
//         -> TRANSMIT(ack, retry 5s x5) -> SLEEP -> ...
// Dev path (on demand, richer OLED): a button press in SLEEP just wakes the
// unit -- WAKE powers the peripherals back on and classifies the gesture there,
// then routes the dev mode:
//   B1 single (click)  -> STATUS         (peek at sensor/batt/radio, then back to sleep)
//   B1 double (click)  -> DEMO           (live OLED, runs one cycle time, then back to sleep)
//   B1 long press      -> TRANSMIT_DEBUG (Don's "send now" gesture; tx attempt + link debug)
//   reset              -> POST           (hard restart to self-test, any time)

enum mode {
  MODE_NONE = -1,      // no mode (invalid)
  MODE_POST = 0,       // power-on self test (entered on reset)
  MODE_STATUS,         // 30s OLED: sensor / battery / radio (RSSI, s since ACK)
  MODE_SLEEP,          // display off; deep sleep until timer or button wake
  MODE_WAKE,           // display off; brief power-up of sonar + radio
  MODE_MEASURE,        // take the median range reading
  MODE_TRANSMIT,       // send reading, wait ACK, retry 5s up to 5x (display off)
  MODE_DEMO,           // always-on OLED live demo; runs for one cycle time
  MODE_TRANSMIT_DEBUG, // OLED: tx attempt + radio-link debug info
  MODE_COUNT
};

// mirror of MomentaryButton.h so this header stays hardware-free / host-testable
#ifndef BUTTON_EVENT_NONE
#define BUTTON_EVENT_NONE         0
#define BUTTON_EVENT_CLICK        1
#define BUTTON_EVENT_LONG_PRESS   2
#define BUTTON_EVENT_DOUBLE_CLICK 3
#define BUTTON_EVENT_TRIPLE_CLICK 4
#endif

// what happened this tick that could drive a transition
struct mode_event {
  int  btn;            // BUTTON_EVENT_* this tick, once classified (NONE if none yet)
  bool elapsed;        // the current mode's dwell timer expired (STATUS 30s, SLEEP 5min, DEMO cycle)
  bool done;           // the current mode's action finished (POST selftest ok, MEASURE got a reading,
                       //   TRANSMIT settled = ACK or retries exhausted, TRANSMIT_DEBUG attempt done,
                       //   WAKE peripheral power-up settled on the plain timer path)
  bool woke_by_button; // SLEEP was interrupted by a button edge (vs the RTC dwell timer). The .cpp
                       //   only classifies the gesture (sets btn) on this path, in WAKE.
};

static inline const char *mode_name(mode m) {
  switch (m) {
    case MODE_POST:           return "POST";
    case MODE_STATUS:         return "STATUS";
    case MODE_SLEEP:          return "SLEEP";
    case MODE_WAKE:           return "WAKE";
    case MODE_MEASURE:        return "MEASURE";
    case MODE_TRANSMIT:       return "TRANSMIT";
    case MODE_DEMO:           return "DEMO";
    case MODE_TRANSMIT_DEBUG: return "TX_DEBUG";
    default:                  return "?";
  }
}


static inline mode handle_button_gesture(mode_event ev) {
  switch (ev.btn) {
    case BUTTON_EVENT_CLICK:
      return MODE_STATUS;
    case BUTTON_EVENT_DOUBLE_CLICK:
      return MODE_DEMO;
    case BUTTON_EVENT_LONG_PRESS:
      return MODE_TRANSMIT_DEBUG;
    default:
      return MODE_NONE;
  }
}
// Pure transition: given the current mode and what happened this tick, return
// the next mode. In the awake display modes (STATUS/DEMO) and in WAKE a
// classified gesture overrides the natural flow. SLEEP never interprets a
// gesture: any wake -- a button edge or the 5-min RTC -- routes to WAKE, which
// powers the peripherals back on and classifies the press there.
static inline mode next_mode(mode cur, mode_event ev) {
  mode next;

  switch (cur) {
    case MODE_POST:
      return ev.done ? MODE_STATUS : MODE_POST;
    case MODE_STATUS:
      next = handle_button_gesture(ev);
      if (next != MODE_NONE) {
        return next;
      }
      return ev.elapsed ? MODE_SLEEP : MODE_STATUS;
    case MODE_SLEEP:
      // a button wake goes STRAIGHT to STATUS -- no gesture-classification
      // dwell; STATUS is the gateway to the dev modes. Timer wakes take the
      // WAKE bring-up path into the deploy measurement.
      if (ev.woke_by_button) return MODE_STATUS;
      return ev.elapsed ? MODE_WAKE : MODE_SLEEP;
    case MODE_WAKE:
      // timer-wake bring-up; button wakes route to STATUS from SLEEP now, so
      // this settles the peripherals and falls through to the measurement.
      next = handle_button_gesture(ev);
      if (next != MODE_NONE) {
        return next;
      }
      return ev.done ? MODE_MEASURE : MODE_WAKE;
    case MODE_MEASURE:
      return ev.done ? MODE_TRANSMIT : MODE_MEASURE;
    case MODE_TRANSMIT:
      return ev.done ? MODE_SLEEP : MODE_TRANSMIT;
    case MODE_DEMO:
      next = handle_button_gesture(ev);
      if (next != MODE_NONE) {
        return next;
      }
      return ev.elapsed ? MODE_SLEEP : MODE_DEMO;
    case MODE_TRANSMIT_DEBUG:
      return ev.done ? MODE_DEMO : MODE_TRANSMIT_DEBUG;
    default:
      return cur;
  }
}