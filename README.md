# obd-idtool

ESP32 firmware for controlling the door locks over a direct CAN connection via an MCP2515 SPI CAN controller. The `lock` and `unlock` commands are confirmed to operate the doors on a 2018 Lexus CT200h via the OBD-II port.

This is the ESP32 beta for the project's final target hardware (nRF52840 + MCP2515, see [../Pin-diagram.md](../Pin-diagram.md)). It replaces an earlier Bluetooth-ELM327-based version with a direct SPI connection to the CAN bus, and keeps only the verified lock/unlock feature.

## Hardware and setup

- ESP32 DevKit
- MCP2515 + SN65HVD230 combo CAN module (3.3V-native transceiver — do **not** substitute a 5V/TJA1050 board without re-checking power and logic levels)
- Arduino IDE with the ESP32 board package and the `MCP_CAN_lib` library (by coryjfowler)

### Wiring (ESP32 VSPI)

| MCP2515 pin | ESP32 pin | Notes |
| --- | --- | --- |
| INT | GPIO4 | Reserved, unused for TX-only |
| SCK | GPIO18 | VSPI clock |
| SI (MOSI) | GPIO23 | VSPI MOSI |
| SO (MISO) | GPIO19 | VSPI MISO |
| CS | GPIO5 | Chip select |
| GND | GND | Common ground with ESP32 |
| VCC | 3.3V | SN65HVD230 is 3.3V-native; do not feed it 5V |
| CANH / CANL | OBD-II pins 6 / 14 | Vehicle CAN bus |

The SN65HVD230 is a 3.3V-logic transceiver, same level as the ESP32's GPIOs, so no level-shifting is needed on SPI/INT and VCC goes straight to the ESP32's 3.3V pin. This is the opposite of the more common 5V/TJA1050 MCP2515 boards covered in [../Pin-diagram.md](../Pin-diagram.md) — if you ever swap modules, re-check VCC and logic levels before powering it up.

### OBD-II (J1962) connector pinout

Looking into the female OBD-II port (pin 1 top-left):

```text
 ____________________________________
 \  1   2   3   4   5   6   7   8   /
  \                                 /
   \  9  10  11  12  13  14  15  16/
    \_______________________________/
```

Only the pins this project uses:

| Pin | Signal | Wired to |
| --- | --- | --- |
| 4 | Chassis ground | ESP32/MCP2515 GND |
| 5 | Signal ground | ESP32/MCP2515 GND (tie with pin 4) |
| 6 | CAN-H (J2284, HS CAN) | MCP2515 CANH |
| 14 | CAN-L (J2284, HS CAN) | MCP2515 CANL |
| 16 | Battery+ (12V, always on) | Vehicle power, **not** ESP32 VCC — regulate down separately if powering the board from the OBD-II port |

Pins 6/14 carry the same powertrain/body CAN bus the `750` door-lock frame rides on; the other pins are unused by this firmware. If you power the ESP32 from pin 16, use a 12V→5V (or 12V→3.3V) regulator — don't feed 12V directly into the dev board's 5V/3.3V pins.

**Check your module's crystal frequency** (8MHz or 16MHz, printed on the crystal can) and set `MCP_OSC_FREQ` in `config.h` to match. This is the setting most likely to be wrong on a cheap breakout — `CAN0.begin()` will still report success with the wrong value, but frames won't ACK on the bus.

Flash `obd-idtool.ino` from the Arduino IDE. Open Serial Monitor at 115200 baud with Newline line endings. The firmware retries MCP2515 init automatically and prints the available commands once on-bus.

## Door controls

Enter either command in Serial Monitor:

```text
lock
unlock
```

The commands send a single raw CAN frame (standard 11-bit ID, no ISO-TP) to ID `0x750`. The frame data is configured in `config.h`:

| Command | Frame data |
| --- | --- |
| `lock` | `40 05 30 11 00 80 00 00` |
| `unlock` | `40 05 30 11 00 40 00 00` |

## Sniffing

The tool boots in **normal** mode, which on this car is the correct choice rather than a neutral one — see the measurement below. `listen` switches to listen-only for short observations and prints a warning; `lock` and `unlock` work in either mode.

| Command | Does |
| --- | --- |
| `sniff` | toggle printing every received frame |
| `stats` | per-ID frame counts and average period |
| `status` | bus state (active/SILENT), mode, error counters |
| `wakeseq` | replay the captured gateway startup sequence on `0x45A` |
| `clear` | reset stats and re-arm the wake capture |
| `listen` / `normal` | set bus mode by hand |

Two things happen automatically, without any command:

- **Silence detection.** After `SILENCE_THRESHOLD_MS` with no frame, the tool prints `*** BUS SILENT ***`. That is the car going to sleep.
- **Wake capture.** The first `WAKE_CAPTURE_FRAMES` frames after a silence are captured with timestamps and deltas, then printed as a table. Whatever wakes the network transmits first, so this is the data worth having.

Note that a busy bus at 500 kbps outruns a 115200 baud serial port, and the MCP2515 only has two receive buffers, so `sniff` will drop frames. The wake capture is unaffected — it only needs the first few.

Watch for replies on `758` (`DOOR_CMD_ID + 8`). A frame there means the body ECU actually *processed* the door command; a successful `sendMsgBuf` only proves some node ACKed it.

## Hunting a wake-up frame

A few minutes after the car is parked its bus goes silent and the door frame fails with `code=7` / `REC=0` — nothing is awake to ACK it. Transmitting to wake the bus does not work on the 2018 CT200h, because the main body ECU wakes on its hardwired door-switch inputs rather than on CAN activity (see `../PKE/README.md`).

But there is a path that isn't ruled out. The smart key ECU must stay awake listening for the fob, and when it hears one it has to tell the body ECU — plausibly over CAN. If so, a replayable wake path exists. This is what the capture settles:

1. Park, connect the tool, leave it in listen-only mode, and wait for `*** BUS SILENT ***`.
2. Trigger a wake **without opening a door** — press the fob, or touch the door handle.
3. Read the capture table.

| Outcome | Means |
| --- | --- |
| Frames appear on the fob press, before any door opens | a CAN wake path exists; the first frames are replay candidates |
| Nothing until a door physically opens | hardwired wake confirmed; the door-switch line is the only way in |
| The bus never goes fully silent | it isn't sleep — look at bitrate and wiring instead |

`status` also reports how long the bus has been silent, which measures how long a wake lasts — the number the keepalive trade-off in `../PKE/README.md` depends on.

### Result: CAN injection does not wake this car

`wakeseq` replayed the sequence above frame for frame and gap for gap:

```text
[wakeseq] replaying 11 frames on 45A, then 5 ticks
[wakeseq] 16 sent, 0 acked; bus=SILENT EFLG=0x15 TEC=128
[wakeseq] nothing acknowledged - the sequence did not wake anything
```

Opening a door moments later produced that same sequence verbatim, with gaps of 80/100/260 ms against our 81/99/260 — so the replay was faithful, and it still woke nothing.

**`0 acked` is the finding.** A module that was awake but idle would still ACK. Zero means nothing on the segment is listening at all, so no frame of any content can reach one. These frames are what the gateway emits *after* something else wakes it. Selective wake (ISO 11898-6) is evidently not implemented here, or the diagnostic segment isn't where it would apply.

Attempts that failed, for the record: `0x7DF` diagnostic request, a burst of the steady-state payload `5A 04 ...`, and the full captured sequence. Don't spend more time on frame-guessing — see `../PKE/README.md` for what to do instead.

### Injecting frames

```text
tx 45A 5A 00 80 42 01 0F 0F 0F      one frame
burst 45A 5A 00 80 42 01 0F 0F 0F   the same frame repeated (TX_BURST_COUNT / TX_BURST_GAP_MS)
```

`burst` exists because network management is periodic: each node transmits its own NM message, and the network sleeps when those stop arriving. A single frame therefore proves nothing — a wake attempt has to be sustained traffic at a plausible period. Burst sends in one-shot TX mode so unacknowledged attempts aren't retried in hardware and the period stays even, and it keeps draining RX throughout, so if the injection does wake the network you see it in the same output.

### What the capture found

On the 2018 CT200h, pressing the **fob** — with no door opened — woke the bus after 149 s of silence. So a CAN wake path does exist, and the "hardwired door switches only" conclusion was wrong. The first frame was:

```text
45A  [8] 5A 00 80 42 01 0F 0F 0F
```

In opendbc's `toyota_2017_ref_pt.dbc`, `0x45A` (1114) is `CGW1N02`, transmitted by **CGW — the central gateway**. That file defines exactly ten `*1N0x` messages, one per ECU (gateway, ACC, AFS, BSR, CSR, FCM, FRD, KSS, MAV, SCS), which is the signature of a network-management scheme rather than ordinary signal traffic.

That matters because the earlier wake attempts used `0x7DF`, an OBD-II **diagnostic request**. A network governed by NM messages has no reason to respond to that. Sending the NM frame instead is the experiment that hasn't been run yet — hence `burst`.

Caveat: `toyota_2017_ref_pt.dbc` is a Toyota 2017 reference file, not CT200h-specific, so the ID match is strong evidence but not proof that it means the same thing on this car.

### Listen-only is not as neutral as it looks

The full capture turned out to be the *same* frame fourteen times inside 3 ms. At 500 kbps an 8-byte frame takes about 250 µs, so that is back-to-back transmission at line rate — the signature of a frame being **retransmitted because nothing acknowledged it**, not of periodic network management.

CAN requires at least one other node to ACK. In listen-only mode this tool deliberately never ACKs, and the gateway is the only other node on the diagnostic segment, so its NM frame is never acknowledged and it retransmits. Its TEC climbs to 128 — and because an error-passive transmitter stops incrementing TEC on ACK errors, it never reaches bus-off. It just retransmits forever.

Measured in listen-only: **5803 copies of `0x45A` in 3999 ms**, about 1400 frames/s, versus 1 Hz in normal mode. `stats` showed 2898 frames across exactly 1 distinct ID.

So listen-only, chosen to avoid disturbing the measurement, both floods the bus and aborts wakes. On a segment where this tool is the only other participant, *not* acknowledging is the intervention. Hence the normal-mode default.

### Confirmed: ACKing is what lets the wake complete

Repeating the fob-press experiment in **`normal` mode** settled it:

```text
*** BUS AWAKE after 47791 ms of silence ***
[door] unlock
[door] sendMsgBuf -> OK
[door-reply]   144031  758  [8] 40 02 70 11 00 00 00 00

     t(ms)   +dt  xN   ID  len  data
    143975     0   1  45A  [8] 5A 04 00 42 01 00 00 00
    144031    56   1  758  [8] 40 02 70 11 00 00 00 00
    144974   943   4  45A  [8] 5A 04 00 42 01 00 00 00  (repeated 4 x over 2998 ms)
```

Three things confirm the diagnosis:

- **The door command worked**, and `758` carries `40 02 70 11`: `0x70` is the positive response to service `0x30`, echoing LID `0x11`. The body ECU *processed* the command — far stronger evidence than an ACK, which only says some node heard the frame.
- **`0x45A` settled into a ~1 Hz period** instead of hammering at line rate. That is what network management is supposed to look like, and it confirms the earlier line-rate burst was unacknowledged retransmission.
- **Its payload changed** from `5A 00 80 42 01 0F 0F 0F` (failed wake) to `5A 04 00 42 01 00 00 00` (successful wake), so byte 1/2 and the trailing bytes appear to carry network state.

The practical rule: **this tool must run in normal mode on a bus where it may be the only other node.** Listen-only is for pure observation only, and it will abort a wake.

Still unsolved for passive entry: the wake above was triggered by a **fob press**. Keeping the network up by participating in NM is now implementable — the ID and cadence are known — but `0x45A` is actively transmitted by the gateway at ~1 Hz, so injecting that ID while the car is awake would collide with the real sender. Measure how long a wake lasts first; `*** BUS SILENT ***` now reports the awake duration.

Replay captured frames one at a time, parked, noting what each does. Unlike the door frame, a newly captured frame is unverified, and network-management or diagnostic-session frames can have side effects.

## Safety

Use only while the vehicle is safely parked. The raw frame values are vehicle-specific; verify them before using this firmware with another vehicle.
