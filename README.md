# obd-idtool
A diagnostic tool to sniff live CAN traffic and test candidate frames, to discover the
CAN header + data bytes that lock/unlock a car's doors via the OBD-II port.

This is **not** a production lock/unlock controller — it's a bench/reverse-engineering
tool. Once you've confirmed the right frame(s), use them in your own trigger firmware.

## Hardware
- ESP32 DevKit (classic, dual-core, has Bluetooth Classic)
- vLinker FD (or other ELM327-compatible) Bluetooth Classic OBD2 adapter

## Firmware
Files:
- `obd-idtool.ino` — BT connection state machine + Serial command shell
- `config.h` — pins, ELM327 MAC address, timing constants

### Required libraries (Arduino IDE Library Manager)
- `ELMduino` (PowerBroker2/ELMduino) — ELM327 command/response handling
- `BluetoothSerial` — bundled with the ESP32 board core, no separate install needed

### Setup
1. Install the ESP32 board package and the `ELMduino` library in the Arduino IDE.
2. Find your adapter's MAC address (pair it with a phone/PC first, or scan with an ESP32
   BT scanner sketch) and set it in `ELM327_MAC` in `config.h`.
3. Select the correct ESP32 board/port in the Arduino IDE and flash `obd-idtool.ino`
   manually (no PlatformIO).
4. Open the Serial Monitor at 115200 baud, line ending "Newline".

## Usage
Once connected (see Serial log), these commands are available:

- `sniff [durationMs]` — enables CAN headers (`ATH1`), then runs the adapter's monitor-all
  mode (`ATMA`) for the given duration (default 15000 ms), printing every raw frame with a
  relative timestamp. **Toggle the physical interior door lock/unlock switch while this
  runs** (not the RF key fob — only bus traffic is visible). Run it once per direction and
  compare the captures to spot the frame(s) that differ.
- `filter <mask> <id>` — sets an ID mask/filter (`ATCM`/`ATCF`) so `sniff` only shows IDs
  matching `<id>` under `<mask>`, e.g. `filter 700 000` keeps only IDs `0x000`-`0x0FF`.
- `filter clear` — resets the mask to `000`/`000` (accept every ID again).
- `scan [durationMsPerBlock]` — automates the filter+sniff binary search: steps through the
  8 standard `0x100` ID blocks (`0x000`-`0x0FF`, `0x100`-`0x1FF`, ... `0x700`-`0x7FF`) one at
  a time. For each block it applies the filter, waits for you to press Enter (so you can get
  ready), then captures for `durationMsPerBlock` (default 5000 ms) while you toggle the door
  switch, then prompts again before moving to the next block. Type `stop` + Enter at any
  prompt to end the scan early; the filter is cleared automatically when it finishes.
- `send <header> <data>` — sets the outgoing arbitration ID via `ATSH <header>`, then
  transmits `<data>` as the raw frame payload, printing the adapter's response. Use this to
  replay candidate frames found via `sniff`/`scan` and confirm which one actually actuates
  the doors.
- `lock` / `unlock` — shortcuts that inject the candidate frame defined by `DOOR_CMD_HEADER`/
  `LOCK_CMD_DATA`/`UNLOCK_CMD_DATA` in `config.h` (currently header `750`, data
  `4005301100400000` / `4005301100800000`, sourced from
  [cydia2020/toyota-can-bus-multitool](https://github.com/cydia2020/toyota-can-bus-multitool)).
  Sent fire-and-forget (`ATR0`) since a broadcast body frame gets no reply. **Unverified for
  this vehicle/wiring** — that project taps the ADAS/Safety bus directly, not the OBD-II port.
- `uds <tx> <rx> <data>` — full diagnostic exchange to one ECU: sets the tx header (`ATSH`),
  the receive filter (`ATCRA`), and flow control, then sends `<data>` and prints the reply.
  ELM327 handles ISO-TP framing automatically. Example: `uds 750 758 1003` (enter extended
  session on the body ECU).
- `tryunlock` — steps through a list of candidate Toyota body-ECU (`0x750`/`0x758`) unlock
  sequences one at a time, pausing after each so you can see whether the doors moved and read
  the ECU's reply (a `7F` response means "rejected", anything else is worth noting).

Example:
```
sniff 20000
scan 5000
uds 750 758 1003
tryunlock
send 750 0227103601
```

## Unlocking the doors via OBD (Route B: blind UDS)
Your Vlinker FD only reaches the OBD-II port, and on a 2018 CT200h the door locks are driven
by the **Main Body ECU**, which responds to **diagnostic (UDS) requests** rather than plain
broadcast frames — that's how the dealer tool (Techstream) unlocks doors through the same
port. To try this without Techstream:

1. Confirm the body ECU answers at all: `uds 750 758 1003`. A non-`7F` reply means it's
   reachable and in an extended session. If you get no response, try other request/response
   pairs (`uds 7C0 7C8 1003`, etc.) or sniff for which IDs reply.
2. Run `tryunlock` and watch the doors after each candidate. The candidate list in
   `tryBodyEcuUnlock()` is a starting set of guesses — edit it as you learn which service/DID
   the ECU accepts.
3. If nothing works, the reliable fallback (Route A) is capturing Techstream's "Door Lock
   Control" Active Test on the DLC3 with a Y-splitter and replaying it with `uds`/`send`.

### Dealing with "BUFFER FULL"
On a busy bus (e.g. HS-CAN at idle), `ATMA` generates far more traffic than the Bluetooth
SPP link can drain, so the adapter's internal buffer overflows and it prints `BUFFER FULL`
mid-stream, drowning out the low-frequency frame you actually care about. If you see this,
narrow the bus with `filter` and binary-search: start with a wide mask (e.g.
`filter 700 000` for the low half of the ID range), `sniff` and toggle the switch, then
narrow further (`filter 780 000`, etc.) into whichever half keeps showing activity, until
the volume is low enough to read cleanly. Run `filter clear` before trying a different
range from scratch.

Repeated identical lines are collapsed to one entry with a `(xN)` count so a single
high-frequency frame doesn't bury the one that changes when you operate the switch. A
`<DATA ERROR` suffix on a line is the adapter itself flagging a bus reception error for
that frame (normal ELM327 behavior) — frequent occurrences alongside `BUFFER FULL` are
another sign the bus is too busy for the current filter and should be narrowed further.

## Notes
- Test only with the vehicle safely parked — sending an incorrect frame can trigger
  unintended ECU behavior.
- On some vehicles the OBD port only exposes the powertrain (HS-CAN) bus and the gateway
  may not forward general body-CAN broadcasts. If `sniff` shows nothing correlated with the
  door switch, capturing while a dealer diagnostic tool (e.g. Techstream) performs its own
  door-lock actuator test can reveal the addressed request/response frames instead.
# odb-tool
