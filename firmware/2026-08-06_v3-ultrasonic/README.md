# Firmware snapshot — 2026-08-06 (v3-ultrasonic)

Complete build snapshot of the v3-ultrasonic pair (MaxBotix MB7388 sender +
Bayou-uploading receiver). This is the set currently shipping on the
micro-config setup page at `edgecollective.io/micro-config/`.

## Source commit

- Repo: `MeshCore-simple-sensor`
- SHA:  `3e9796ff` (branch `main`)
- Tag:  `snapshot/2026-08-06_v3-ultrasonic`

Re-check out with `git checkout snapshot/2026-08-06_v3-ultrasonic` to
reproduce these bins byte-for-byte (assuming the same PlatformIO/Arduino
toolchain versions).

## Files

| File | Board | Role | env |
|------|-------|------|-----|
| `rook_sender_ultrasonic_v3.uf2` | Rook v4 (nRF52840 + SX1262) | Sender | `Rook_companion_sensor_v3_ultrasonic` |
| `heltec_v3_receiver_ultrasonic_v3.bin` | Heltec WiFi LoRa 32 V3 | Receiver (app only) | `Heltec_v3_companion_sensor_receiver_v3_ultrasonic` |
| `heltec_v3_receiver_ultrasonic_v3_merged.bin` | Heltec V3 | Receiver (full flash) | same |
| `heltec_v4_receiver_ultrasonic_v3.bin` | Heltec WiFi LoRa 32 V4 (ESP32-S3) | Receiver (app only) | `heltec_v4_companion_sensor_receiver_v3_ultrasonic` |
| `heltec_v4_receiver_ultrasonic_v3_merged.bin` | Heltec V4 | Receiver (full flash) | same |

## Shared radio settings

```
-D LORA_FREQ=910.525
-D LORA_BW=62.5
-D LORA_SF=7
-D LORA_CR=5
```

All three envs share these — a Rook sender, V3 receiver, and V4 receiver can
all talk to each other on the same mesh.

## Rebuild

```bash
pio run -e Rook_companion_sensor_v3_ultrasonic
pio run -e Heltec_v3_companion_sensor_receiver_v3_ultrasonic
pio run -e heltec_v4_companion_sensor_receiver_v3_ultrasonic -t mergebin
```

Merged bins for the ESP32 receivers are produced by the `mergebin` target
(see `merge-bin.py`); PlatformIO's default `run` target only produces
`firmware.bin` (the app image without bootloader/partitions).

## Bayou defaults

The receivers ship with `bayou_public_key = mfupcqtx34ee` and
`bayou_private_key = t3w7v5zpk8ya` baked in. Override with:

```
set bayou_public_key <your-key>
set bayou_private_key <your-key>
```

## What's new vs the 2026-05-26 snapshot

- Adds a **Heltec V4 receiver** build (`heltec_v4_companion_sensor_receiver_v3_ultrasonic`).
  Functionally identical to the V3 receiver — same firmware, same LoRa params,
  same Bayou defaults — but built for the newer Heltec board.
- The 2026-05-26 snapshot was of a different `v3` (companion_sensor pair,
  not v3-ultrasonic). This snapshot is the first coherent bundle of the
  v3-ultrasonic pair with both V3 and V4 receiver options.
