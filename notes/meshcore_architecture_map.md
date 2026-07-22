# MeshCore architecture map (Rook / SX1262 build)

Reverse-engineered July 2026 while working out how to sleep the radio for the
duty-cycled sonar field node. Read this before touching MeshCore internals; the
key trap is that the radio is owned by a base class, not composable away.

## The two inheritance chains

**Radio side (a `mesh::Radio` implementation):**
```
mesh::Radio  (abstract iface, src/Dispatcher.h)
  <- RadioLibWrapper            (src/helpers/radiolib/RadioLibWrappers.{h,cpp})
     <- CustomSX1262Wrapper     (src/helpers/radiolib/CustomSX1262Wrapper.h)  == WRAPPER_CLASS
```
The concrete instance is `radio_driver` (`WRAPPER_CLASS radio_driver(radio, board);`
in `variants/rook/target.cpp`), wrapping the RadioLib `radio` object (`extern RADIO_CLASS radio`).

**Mesh side (the "system" -- this is the trap):**
```
Dispatcher   (src/Dispatcher.{h,cpp})  -- OWNS the radio + the radio loop
  <- mesh::Mesh          (src/Mesh.{h,cpp})            -- routing / protocol
     <- BaseChatMesh     (src/helpers/BaseChatMesh.*)  -- contacts, chat, ACK tracking
        <- MyMesh        (the app, e.g. variants/sonar_field_node) == the_mesh
```
So **`Mesh : public Dispatcher`** -- the mesh protocol IS-A radio dispatcher. You
cannot peel "packet/routing logic" off "radio loop"; they are one object. There is
no fork-free way to run the radio yourself while reusing only the packet layer.

## Who owns what

- **Dispatcher** owns `mesh::Radio* _radio`, and its `loop()` services the radio every
  call: `triggerNoiseFloorCalibrate()`, `_radio->loop()`, `isInRecvMode()`, `resetAGC()`,
  send-complete handling, TX airtime/duty-cycle budget. It runs an **8s "radio stuck in
  non-Rx" watchdog** (`radio_nonrx_start` -> `ERR_EVENT_STARTRX_TIMEOUT`). It ASSUMES the
  radio is continuously live in a known state.
- **Mesh** adds packet routing/flood/direct + dedup.
- **BaseChatMesh** adds contacts, ACK-CRC tracking, the sensor/chat send helpers.
- **MainBoard** (`src/MeshCore.h`, impl e.g. `NRF52Board`) -- board/power/startup-reason;
  `Bluefruit.begin()` runs here (SoftDevice active -> use `waitForEvent()`/`sd_app_evt_wait`,
  never raw `__WFE`/RTC banging).
- **The app owns `loop()`** and chooses when to call `the_mesh.loop()`. So MeshCore is
  ALREADY driven as a subsystem -- the app controls the cadence. What's missing is a
  sleep seam.

## The `mesh::Radio` interface (what the wrapper must implement)

begin, recvRaw, getEstAirtimeFor, packetScore, startSendRaw, isSendComplete,
onSendFinished, loop, getNoiseFloor, triggerNoiseFloorCalibrate, resetAGC,
isInRecvMode, isReceiving, getLastRSSI, getLastSNR.
Wrapper (protected, not app-callable): `idle()` (= radio.standby()), `startRecv()`
(= radio.startReceive()). Public: `begin()`, `powerOff()` (= `_radio->sleep()`).

## The radio-sleep trap, and the seam that was written then reverted

Sleeping the radio at the app level deadlocks because:

1. `Dispatcher::loop()` keeps polling the slept radio over SPI (BUSY never clears).
2. The Dispatcher's radio-liveness state is **private** -- `prev_isrecv_mode`,
   `radio_nonrx_start`, `next_floor_calib_time`, `next_agc_reset_time`, `outbound` --
   so neither the app nor a subclass can re-sync it after a sleep.
3. `radio.sleep()` is a WARM sleep (config retained); wake = `radio.standby()` then
   `startReceive()`. Calling `begin()` on a slept radio deadlocks the same way the
   poll does. On nRF52 the USB serial console dies during all this and only a full
   battery+USB power cycle recovers it (bench-only concern; the field node is
   battery-only).

**The seam (minimal patch, would be contributable upstream):**

- Add `virtual void sleep()` / `virtual void wake()` to `mesh::Radio`; wrapper does
  `sleep(){ _radio->sleep(); }`, `wake(){ _radio->standby(); startReceive(); }`.
- Add `Dispatcher::radioSleep()` / `radioWake()`: set an `_radio_asleep` flag (loop()
  early-returns while set) and on wake reset `prev_isrecv_mode=true`,
  `radio_nonrx_start=now`, `next_floor_calib_time=next_agc_reset_time=0`.
- App calls `the_mesh.radioSleep()` on sleep entry, `the_mesh.radioWake()` on wake.

This seam was written and compiled, then **reverted**: the same result is
reachable without touching the library, because the app already controls when
`the_mesh.loop()` runs. The application-side module (`radio_lowpower.h`) sleeps
the RadioLib object directly and simply stops servicing the mesh while the radio
is asleep, so nothing polls it. The library-level seam remains the better
long-term answer and is worth proposing upstream as its own conversation.

## Files

- `src/Dispatcher.{h,cpp}` -- Radio iface + Dispatcher (radio loop, budget, watchdog).
- `src/Mesh.{h,cpp}` -- routing.
- `src/helpers/BaseChatMesh.*` -- contacts/chat/ACK.
- `src/helpers/radiolib/RadioLibWrappers.{h,cpp}` -- the wrapper.
- `variants/rook/target.cpp` -- `radio_driver`, `radio`, board wiring.
- `variants/sonar_field_node/` -- the app (mode machine, `the_mesh`).
