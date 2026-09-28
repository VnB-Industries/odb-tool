# obd-idtool

ESP32 firmware for controlling the door locks and inspecting CAN traffic through a Bluetooth ELM327 adapter. The `lock` and `unlock` commands are confirmed to operate the doors on a 2018 Lexus CT200h via the OBD-II port.

## Hardware and setup

- ESP32 DevKit with Bluetooth Classic
- vLinker FD or another Bluetooth Classic ELM327-compatible adapter
- Arduino IDE with the ESP32 board package and the `ELMduino` library

Set the adapter's Bluetooth address in `ELM327_MAC` in `config.h`, then flash `obd-idtool.ino` from the Arduino IDE. Open Serial Monitor at 115200 baud with Newline line endings. The firmware reconnects to the adapter automatically and prints the available commands when connected.

## Door controls

Enter either command in Serial Monitor:

```text
lock
unlock
```

The commands send a raw CAN frame to header `750`. The frame data is configured in `config.h`:

| Command | Frame data |
| --- | --- |
| `lock` | `4005301100800000` |
| `unlock` | `4005301100400000` |

## Other commands

- `sniff [durationMs]` captures CAN traffic for 15 seconds by default. Operate the physical door switch during capture to inspect related traffic.
- `filter <mask> <id>` limits captured CAN IDs; `filter clear` removes the filter.
- `scan [durationMsPerBlock]` captures each of eight 0x100-ID blocks, 5 seconds per block by default. Press Enter to begin each capture; enter `stop` at a prompt to end early.
- `send <header> <data>` sends a custom raw CAN frame.
- `uds <tx> <rx> <data>` sends a diagnostic request and displays the response.
- `ecuscan` probes diagnostic IDs from `0x700` to `0x7EF`; `idscan` reads identification data from the responders found by `ecuscan`.
- `tryunlock [tx] [rx]` tries diagnostic unlock candidates, using `750`/`758` by default.

On a busy CAN bus, `sniff` may report `BUFFER FULL`. Use `filter` to narrow the captured ID range.

## Safety

Use only while the vehicle is safely parked. The raw frame values are vehicle-specific; verify them before using this firmware with another vehicle.