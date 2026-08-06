# v3-ultrasonic — Heltec V4 sender / Heltec V3 receiver

This pairing runs the v3-ultrasonic firmware on two Heltec ESP32-S3 boards:

- **Sender**: Heltec WiFi LoRa 32 V4 — reads a MaxBotix MB7388 over UART and
  transmits `[SENSOR] dist=... batt=...` LoRa DMs to the receiver.
- **Receiver**: Heltec WiFi LoRa 32 V3 — receives, ACKs, and posts to Bayou
  over WiFi.

Firmware source is shared with the parent v3-ultrasonic setup — this directory
just holds the pairing docs and pre-built UF2/BIN snapshots.

## Envs

| Role     | Board       | PlatformIO env                                       |
|----------|-------------|------------------------------------------------------|
| Sender   | Heltec V4   | `heltec_v4_companion_sensor_v3_ultrasonic`           |
| Receiver | Heltec V3   | `Heltec_v3_companion_sensor_receiver_v3_ultrasonic`  |

## Wiring — sender side (Heltec V4)

| MB7388 pin | Heltec V4 pin | Notes                    |
|------------|---------------|--------------------------|
| 5 (TX)     | GPIO47        | TTL serial, 9600 baud    |
| 6 (V+)     | 3V3           | 3.0–5.5 V range          |
| 7 (GND)    | GND           |                          |

The `ULTRASONIC_RX_PIN=47` build flag routes `Serial1` RX to GPIO47 on ESP32.

## No-sensor behavior

If the MB7388 is disconnected at boot, the sender's boot probe times out after
500 ms (non-blocking, no freeze), flags `has_sensor = false` on the OLED, and
still sends `dist=0.000m` payloads on the normal schedule so the receiver keeps
seeing packets. `last_dist_m` remains at its init value (0.0) until a real
frame arrives.

## Build & flash

```
pio run -e heltec_v4_companion_sensor_v3_ultrasonic -t upload
pio run -e Heltec_v3_companion_sensor_receiver_v3_ultrasonic -t upload
```

Pre-built binaries live under `firmware/`.
