# Development log: the water-level field node

This documents the work, June-July 2026, of turning the v3-ultrasonic
companion sensor into a deployable, duty-cycled water-level node:
a MaxBotix MB7388 ultrasonic rangefinder measures the distance down to a water
surface, and the reading travels over LoRa (Long Range, a low-power radio scheme
that carries small packets for kilometers) through a mesh network to a gateway.

Upstream baseline for everything here:

```
615ebd4531bf6536e21f1316d887506f8917d086  "version 3+"
```

The v3-ultrasonic example already supplied the hard half: the mesh layer, the
acknowledgement state machine, the flood fallback and the non-forwarding leaf
behavior. None of that was re-derived. What this work adds is the sensor
integration, the power work needed to run the thing on a battery
in the field, and the bench validation that caught what the first dock-side
deployment would otherwise have found the hard way.

## What changed, and where

The changes live in the application and variant layer:

- `variants/sonar_field_node/` - the deployment firmware: a mode state machine
  (POST, STATUS, SLEEP, WAKE, MEASURE, TRANSMIT) with the sonar read, battery
  telemetry, transmit-with-retry and a link-health LED.
- `variants/sonar_oled_demo/` - radio-free sonar bring-up, useful as a known-good
  reference when the mesh is not the thing under test.
- `variants/sleep_test/` - a minimal sketch used to find where the sleep current
  actually goes.
- `examples/v3-ultrasonic/` - additive debug instrumentation, behind build flags,
  so the default build is unchanged.

**The MeshCore library source is deliberately untouched.** At one point a small
sleep/wake seam was written into the dispatcher and then reverted in favour of an
application-side module (`variants/sonar_field_node/radio_lowpower.h`) that
achieves the same result without patching the library. That separation is
intentional and worth preserving.

## How to read the commit history

**The history here is a reconstruction, not archaeology.** The firmware was
developed on the bench between June and July 2026 and was not committed
incrementally at the time. The branches and pull requests were authored
afterwards, from dated engineering notes and instrument captures, to
present the work as an ordered and reviewable sequence.

The commit dates are therefore not when the work happened, and the sequence is tidier
than the actual path was. Each pull request describes what was measured and when,
including the wrong turns, because several of the conclusions along the way were
wrong and had to be retracted. Those retractions are part of the record on
purpose. A clean narrative that hides them would be less useful to anyone
repeating this work.

## Status

Bench-verified end to end: sonar ranging, gated sensor power, radio sleep with
wake, a full measure/transmit/acknowledge/sleep cycle, and a validation pass
that ran the firmware the way the field will run it (battery-first cold boot,
sensor attached, buttons pressed by a human) and fixed the six defects that
pass exposed.

The power question is settled, though not the way we hoped. The sleep floor
came down from 8 mA to about 1.05 mA, and there it stops: a measurement
campaign that stripped the firmware to nothing and cut every subsystem in turn
showed the remaining current belongs to the microcontroller platform and this
board class, not to anything the application does. Reaching microamps needs
System-OFF (the chip's deep power-down state) plus an external wake source,
which is a hardware change, not a software setting. The pull requests state
the measured numbers and how they were obtained rather than claiming a target
that was not reached.

## Credit

Base firmware and the ultrasonic adaptation: Don Blair, Edge Collective.
The mesh protocol work belongs to the MeshCore project.

Development was assisted by Anthropic's Claude Code agentic platform, using
the Opus 4.8 and Fable 5 models.
