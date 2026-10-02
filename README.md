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

## Safety

Use only while the vehicle is safely parked. The raw frame values are vehicle-specific; verify them before using this firmware with another vehicle.
