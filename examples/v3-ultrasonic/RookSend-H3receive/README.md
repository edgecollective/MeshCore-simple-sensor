# v3-ultrasonic — Rook sender / Heltec V3 receiver

This pairing runs the v3-ultrasonic firmware with a Rook (nRF52840) as the
sender and a Heltec WiFi LoRa 32 V3 (ESP32-S3) as the receiver.

- **Sender**: Rook v4 — reads a MaxBotix MB7388 over UART and transmits
  `[SENSOR] dist=... batt=...` LoRa DMs to the receiver.
- **Receiver**: Heltec V3 — receives, ACKs, and posts to Bayou over WiFi.

Firmware source is shared with the parent v3-ultrasonic setup — this
directory just holds the pairing docs and pre-built snapshots.

## Envs

| Role     | Board       | PlatformIO env                                       |
|----------|-------------|------------------------------------------------------|
| Sender   | Rook v4     | `Rook_companion_sensor_v3_ultrasonic`                |
| Receiver | Heltec V3   | `Heltec_v3_companion_sensor_receiver_v3_ultrasonic`  |

## Wiring — sender side (Rook)

| MB7388 pin | Rook pin                         | Notes                    |
|------------|----------------------------------|--------------------------|
| 5 (TX)     | D0 = P0.08 (silkscreen "RX1")    | TTL serial, 9600 baud    |
| 6 (V+)     | 3V3                              | 3.0–5.5 V range          |
| 7 (GND)    | GND                              |                          |
| 4 (RX)     | leave floating                   | continuous mode ~6 Hz    |

The Rook build uses `Serial1` default pins from the local variant
(`variants/rook/variant.h`): `PIN_SERIAL1_RX = 0`, `PIN_SERIAL1_TX = 1`.
Note the local variant swaps D0/D1 vs. the canonical Nice!Nano pinout.

## Build & flash

```
pio run -e Rook_companion_sensor_v3_ultrasonic -t upload
pio run -e Heltec_v3_companion_sensor_receiver_v3_ultrasonic -t upload
```

The Rook env auto-generates `firmware.uf2` via `create-uf2-post.py`; drag
it onto the `NICENANO` mass-storage volume that appears when the board is
in UF2 bootloader mode (double-tap reset).

## Pre-built snapshots (`firmware/`)

| File                          | Board      | Where it flashes                              |
|-------------------------------|------------|-----------------------------------------------|
| `rook_sender.uf2`             | Rook       | UF2 bootloader (drag-to-NICENANO)             |
| `heltec_v3_receiver.bin`      | Heltec V3  | app-only, `esptool write_flash 0x10000`       |
| `heltec_v3_receiver_merged.bin` | Heltec V3 | full image (bootloader+partitions+app), 0x0  |

For the MeshCore web flasher's "custom firmware" upload, use the app-only
`.bin` with **erase-flash off** (the stock bootloader stays put), or the
merged `.bin` if the flasher writes at offset 0.

## Bayou defaults (receiver)

The receiver ships with these default Bayou keys baked in (used on first
boot before any `set bayou_public_key ...` overwrites them):

- `bayou_public_key`  = `mfupcqtx34ee`
- `bayou_private_key` = `t3w7v5zpk8ya`

Override at build time with
`-D BAYOU_DEFAULT_PUBLIC_KEY='"..."' -D BAYOU_DEFAULT_PRIVATE_KEY='"..."'`.

## Pairing on first boot

Each device generates its own identity keypair on first boot; contacts
are learned live, not baked in. To pair:

1. On the **receiver**: `advert` (broadcasts identity).
2. On the **sender**: watch for `ADVERT from Receiver` in the serial
   console — it auto-adds the receiver to `/contacts`.
3. On the **sender**: set the send target to the receiver's key prefix
   (see the sender's `help` output for the exact verb).
