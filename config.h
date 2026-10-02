#pragma once

#include <mcp_can.h>

// ---- Debug ----
#define DEBUG_ENABLED 1 // set to 0 to silence Serial debug output

#if DEBUG_ENABLED
  #define DEBUG_PRINT(...)   Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

// ---- MCP2515 CAN controller (SPI) ----
// Module: MCP2515 + SN65HVD230 combo board. SN65HVD230 is 3.3V-native, same logic level as the
// ESP32, so VCC wires to 3.3V (not 5V) and no level-shifting is needed - see README.md.
// ESP32 VSPI default pins: SCK=18, MISO=19, MOSI=23. Only CS and INT are configurable here.
static const uint8_t MCP_CS_PIN  = 5; // MCP2515 CS
static const uint8_t MCP_INT_PIN = 4; // MCP2515 INT (reserved; not needed for TX-only use)

// Crystal on the MCP2515 module - this is the setting most likely wrong on a cheap breakout.
// Common blue modules: 8MHz. Some modules (e.g. with a CAN transceiver breakout): 16MHz.
// Wrong value: begin() still returns CAN_OK, but bit timing is off and frames won't ACK on the bus.
#define MCP_OSC_FREQ MCP_8MHZ // MCP_8MHZ or MCP_16MHZ

// '6' on the old ELM327 build = ISO 15765-4 CAN, 11-bit ID, 500 kbps. Match that here.
#define MCP_CAN_SPEED CAN_500KBPS

// ---- GPIO ----
static const uint8_t LED_PIN = 2; // onboard LED, connection status indicator

// ---- Timing ----
static const uint32_t MCP_RETRY_INTERVAL_MS = 1000; // delay between MCP2515 init attempts

// ---- Door lock/unlock frame (VERIFIED working on 2018 Lexus CT200h via OBD-II) ----
// Standard 11-bit CAN frame, DLC 8. Toyota body active test: sub-addr 0x40, len 0x05,
// service 0x30 (inputOutputControlByLocalIdentifier), LID 0x11 (door lock), control 0x00 XX.
// Frame origin: cydia2020/toyota-can-bus-multitool. Control byte 0x80 = lock, 0x40 = unlock on this car.
static const uint32_t DOOR_CMD_ID = 0x750;
static const uint8_t LOCK_CMD_DATA[8]   = {0x40, 0x05, 0x30, 0x11, 0x00, 0x80, 0x00, 0x00};
static const uint8_t UNLOCK_CMD_DATA[8] = {0x40, 0x05, 0x30, 0x11, 0x00, 0x40, 0x00, 0x00};
