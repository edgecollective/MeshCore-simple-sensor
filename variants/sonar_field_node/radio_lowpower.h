#pragma once
// App-side low-power radio control for the duty-cycled field node.
//
// Keeps MeshCore PRISTINE: instead of patching the Dispatcher (which owns the radio
// and polls it every loop), we SPI-sleep the RadioLib radio object directly and have
// the app PAUSE the mesh -- loop() skips the_mesh.loop() while g_radio_slept is set,
// so nothing polls the slept radio (a poll would deadlock on the SX1262 BUSY line).
//
// Wake is standby + re-arm Rx only. radio.sleep() is a WARM sleep: it retains config
// AND the DIO/ISR mapping, so we do NOT call radio_driver.begin() on wake (begin()
// re-runs SPI setup and deadlocks against a still-slept radio -- that was the original hang).
// The MeshCore wrapper's internal state stays STATE_RX across this (sleep() never
// changes it), and we re-arm hardware Rx to match, so isInRecvMode() stays consistent
// and the dispatcher resumes cleanly when the app un-pauses it.
//
// MCU stays System-ON idle (delay()/waitForEvent) -- required for a timed wake with no
// external RTC. This module only governs the radio.

#include <SPI.h>              // SPI.end()/begin() for the tri-state path
#include <target.h>            // RADIO_CLASS + the shared radio object, SX126X_POWER_EN
extern RADIO_CLASS radio;

// SLEEP_RADIO_POWER_OFF: also drop the radio's supply gate during sleep.
//
// SX126X_POWER_EN (pin 21 / P0.13) is driven HIGH once in RookBoard::begin() and never
// touched again, so the TCXO + RF-switch domain stays powered even while the SX1262
// itself is SPI-slept. The sleep_test ladder measured what that domain costs:
//   POWER_EN high, radio in POR standby ... 2.82 mA
//   POWER_EN LOW ..................... 1.05 mA   (-1.77 mA: TCXO + RF switch)
//   + SPI sleep ...................... 146.6 uA  (-0.9 mA: digital core)
// We currently do the second half only, and sit at ~1.06 mA -- close to that 1.05 mA
// rung, which is what makes this the prime suspect for the remaining floor.
//
// COST: cutting power loses the chip's configuration, so warm sleep is off the table
// and wake must do a full cold re-init (radio_init -> std_init). That is deliberately
// NOT the same operation as the original hang: those were begin() against a chip still in
// SPI-sleep, whereas this is a cold power-up, i.e. the normal boot path.
// Default 0 so the measured 1.066 mA configuration stays the default until this is
// proven on the bench.
#ifndef SLEEP_RADIO_POWER_OFF
#define SLEEP_RADIO_POWER_OFF 0
#endif

// SLEEP_RADIO_TRISTATE: hi-Z every nRF52 line into the SX1262 before cutting its
// supply, and release SPIM so it stops driving SCK/MOSI.
//
// MEASURED 2026-07-18: dropping POWER_EN *without* this made sleep 3x WORSE
// (1.0595 mA -> 3.2120 mA, 300 sigma). With the radio's VCC gated off, the MCU was
// still driving NSS/SCK/MOSI/RXEN/DIO into a dead chip, so current flowed through
// the SX1262's input protection diodes into its unpowered rail -- back-powering it
// through its own pins, in an undefined state. Same physics as the sonar
// sneak path, where an active UART RX pad loaded the sensor while its ground floated.
//
// This is also why the early sleep_test result (POWER_EN low saves 1.77 mA) did not transfer: that was
// measured in the minimal sleep_test sketch, where these lines were not being driven.
// A power figure measured in a minimal sketch does not carry to full firmware if the
// pin states differ -- the pins are part of the measurement.
#ifndef SLEEP_RADIO_TRISTATE
#define SLEEP_RADIO_TRISTATE 0
#endif

// SLEEP_RADIO_PIN_DISCONNECT: use the nRF52's PIN_CNF INPUT=Disconnect state
// instead of Arduino INPUT for the eight radio lines while the supply is gated.
//
// The tri-state arm (2026-07-18) recovered only half the back-powering
// regression: 3.212 -> 2.102 mA, still +1.04 mA over the radio-powered floor.
// pinMode(INPUT) leaves the pin floating with its input buffer CONNECTED, and a
// floating CMOS input sitting near threshold conducts through the buffer. The
// chip's own reset state for every GPIO is buffer-disconnected (PIN_CNF=0x0002);
// nrf_gpio_cfg_default() returns a pin to it. This arm tests whether the
// residual +1.04 mA was buffer conduction. Wake needs no special restore: the
// cold re-init path (SPI.begin + radio_init) re-runs every pinMode, which
// rewrites PIN_CNF completely -- the same restore the tri-state arm proved out.
#ifndef SLEEP_RADIO_PIN_DISCONNECT
#define SLEEP_RADIO_PIN_DISCONNECT 0
#endif

#if SLEEP_RADIO_PIN_DISCONNECT
#include "nrf_gpio.h"
// Arduino pin -> nRF GPIO via the variant map (D0/D1-swapped Rook variant --
// never hand-translate pin numbers; the map is the authority).
static inline void radio_pin_silence(uint32_t arduino_pin) {
  nrf_gpio_cfg_default(g_ADigitalPinMap[arduino_pin]);
}
#endif

// SLEEP_RADIO_SEQ_FIX: enter sleep via standby -> clear IRQs -> sleep(warm),
// then park the RF switch and chip-select for the sleep window.
//
// The bare radio.sleep() below is issued straight from continuous Rx after
// TX/ACK traffic. RadioLib #1736 (measured on this same nRF52840 + Wio-SX1262
// combo) shows a pending radio interrupt at SetSleep can strand the die in
// STANDBY: 600 uA (RC) / 800 uA (XOSC) instead of ~1 uA -- which would account
// for most of our 1.06 mA floor by itself. Also per the module docs: RF_SW held
// high costs +55 uA, and NSS must stay a DRIVEN HIGH (a floating NSS measures
// 2.2 mA). Wake path is unchanged: standby() + startReceive(), the proven arc.
#ifndef SLEEP_RADIO_SEQ_FIX
#define SLEEP_RADIO_SEQ_FIX 0
#endif

static bool g_radio_slept = false;

// SPI-sleep the radio (~1 uA). Caller must then stop servicing the mesh
// (loop() gates on g_radio_slept) until radio_wake_lp().
static inline void radio_sleep_lp() {
#if SLEEP_RADIO_SEQ_FIX
  radio.standby();              // leave Rx cleanly before sleeping
  radio.clearIrqStatus();       // clear ALL pending radio IRQs so SetSleep sticks
  radio.sleep(true);            // warm sleep, config retained
  digitalWrite(SX126X_RXEN, LOW);       // RF switch off for the window (+55 uA if left high)
  digitalWrite(P_LORA_NSS, HIGH);       // chip-select parked driven-HIGH, never floating
#else
  radio.sleep();                // SX126x SetSleep (warm: config + DIO retained)
#endif
#if SLEEP_RADIO_POWER_OFF
#if SLEEP_RADIO_TRISTATE
  // Stop driving anything into the chip BEFORE removing its supply, or the MCU
  // back-powers it through the input protection diodes (measured: +2.15 mA).
  SPI.end();                            // release SPIM: stops driving SCK/MOSI
#if SLEEP_RADIO_PIN_DISCONNECT
  // Buffer-disconnected (chip reset state), not merely floating: a floating
  // input with its buffer connected still conducts near threshold.
  radio_pin_silence(P_LORA_NSS);        // P1.13
  radio_pin_silence(P_LORA_SCLK);       // P1.11
  radio_pin_silence(P_LORA_MOSI);       // P1.15
  radio_pin_silence(P_LORA_MISO);       // P0.02
  radio_pin_silence(P_LORA_RESET);      // P0.09
  radio_pin_silence(P_LORA_BUSY);       // P0.29
  radio_pin_silence(P_LORA_DIO_1);      // P0.10
  radio_pin_silence(SX126X_RXEN);       // P0.17
#else
  pinMode(P_LORA_NSS,   INPUT);
  pinMode(P_LORA_SCLK,  INPUT);
  pinMode(P_LORA_MOSI,  INPUT);
  pinMode(P_LORA_MISO,  INPUT);
  pinMode(P_LORA_RESET, INPUT);
  pinMode(P_LORA_BUSY,  INPUT);
  pinMode(P_LORA_DIO_1, INPUT);
  pinMode(SX126X_RXEN,  INPUT);
#endif
#endif
  digitalWrite(SX126X_POWER_EN, LOW);   // cut TCXO + RF-switch domain
#endif
  g_radio_slept = true;
}

// Wake the radio and re-arm continuous Rx, then let the mesh resume.
static inline void radio_wake_lp() {
#if SLEEP_RADIO_POWER_OFF
  digitalWrite(SX126X_POWER_EN, HIGH);
  delay(10);                    // TCXO settle -- same 10 ms RookBoard::begin() allows
#if SLEEP_RADIO_TRISTATE
  SPI.begin();                  // SPIM back up before std_init drives the bus
#endif
  radio_init();                 // cold re-init (std_init): config was lost with power
  radio.startReceive();         // re-arm continuous Rx
#else
  radio.standby();              // wake out of warm sleep to standby
  radio.startReceive();         // continuous Rx (DIO/ISR mapping retained through warm sleep)
#endif
  g_radio_slept = false;
}
