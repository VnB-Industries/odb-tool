#pragma once

#include <mcp_can.h>

// ---- Debug ----
#define DEBUG_ENABLED 1 // set to 0 to silence Serial debug output

#if DEBUG_ENABLED
  #define DEBUG_PRINT(...)   Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTF(...)  Serial.printf(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTF(...)
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

// Diagnostic responses come back on the request ID + 8. Seeing a frame here means the body ECU
// actually processed the door command, which an ACK alone does not prove.
static const uint32_t DOOR_RESPONSE_ID = 0x758;

// ---- Sniffer ----
// No frame for this long means the bus has gone to sleep. Toyota modules take a few minutes after
// the car is parked, so this only has to be long enough to not trip on normal idle gaps.
static const uint32_t SILENCE_THRESHOLD_MS = 2000;

// Frames captured and printed when the bus comes back after a silence. This is the interesting
// data: whatever wakes the network transmits first, and that frame is the candidate to replay.
//
// Consecutive identical frames collapse into one entry with a repeat count. Without that, an
// unacknowledged transmitter retrying at line rate fills the whole buffer in a couple of
// milliseconds and hides everything that follows.
static const uint8_t WAKE_CAPTURE_FRAMES = 32;

// The capture also flushes after this long, so a wake that produces only a handful of distinct
// frames still gets printed instead of waiting forever for the buffer to fill.
static const uint32_t WAKE_CAPTURE_MS = 4000;

// ---- Network-management watch ----
// The gateway's NM frame is the only traffic on an otherwise idle bus, and its payload changes as
// the network approaches sleep (byte 1 observed stepping 04 -> 14, and 00 with byte 2 = 80 while a
// wake was failing). Logging only the changes exposes that state machine without flooding serial.
static const uint32_t NM_WATCH_ID = 0x45A;

// ---- Wake sequence replay (`wakeseq`) ----
// Captured from a real wake (door opened from inside). The gateway doesn't just repeat one frame -
// it runs a startup sequence on 0x45A with byte 1 stepping and byte 2 carrying a flag, so replaying
// a single frame may not be enough. delayMs is the gap *before* that frame, from the capture.
struct WakeStep {
  uint16_t delayMs;
  uint8_t data[8];
};

static const uint32_t WAKESEQ_ID = 0x45A;

static const WakeStep WAKE_SEQUENCE[] = {
  {   0, {0x5A, 0x00, 0x80, 0x42, 0x01, 0x0F, 0x0F, 0x0F} }, // first frame of the wake
  {  81, {0x5A, 0x01, 0x80, 0x42, 0x01, 0x00, 0x00, 0x00} },
  {  99, {0x5A, 0x02, 0x00, 0x42, 0x01, 0x00, 0x00, 0x00} },
  { 260, {0x5A, 0x01, 0x00, 0x42, 0x01, 0x00, 0x00, 0x00} }, // handshake alternates from here
  { 100, {0x5A, 0x02, 0x00, 0x42, 0x01, 0x00, 0x00, 0x00} },
  { 260, {0x5A, 0x01, 0x00, 0x42, 0x01, 0x00, 0x00, 0x00} },
  { 100, {0x5A, 0x02, 0x00, 0x42, 0x01, 0x00, 0x00, 0x00} },
  { 260, {0x5A, 0x01, 0x00, 0x42, 0x01, 0x00, 0x00, 0x00} },
  { 100, {0x5A, 0x02, 0x00, 0x42, 0x01, 0x00, 0x00, 0x00} },
  { 260, {0x5A, 0x01, 0x00, 0x42, 0x01, 0x00, 0x00, 0x00} },
  { 999, {0x5A, 0x04, 0x00, 0x42, 0x01, 0x00, 0x00, 0x00} }, // steady state, 1 Hz
};
static const uint8_t WAKE_SEQUENCE_LEN = sizeof(WAKE_SEQUENCE) / sizeof(WakeStep);

// After the sequence, keep ticking the steady-state frame so the network has time to join in.
static const uint8_t  WAKESEQ_TICKS   = 5;
static const uint16_t WAKESEQ_TICK_MS = 1000;

// ---- Frame injection (`tx` / `burst`) ----
// Network management works by each node periodically transmitting its own NM message; the network
// sleeps when those stop. So a wake attempt has to be a *sustained periodic* NM frame, not one
// shot. Burst sends in one-shot TX mode, so an unacknowledged attempt isn't retried in hardware
// and the period stays even.
static const uint16_t TX_BURST_COUNT  = 40;
static const uint16_t TX_BURST_GAP_MS = 100;

// Distinct IDs tracked by the `stats` command. Periodic network-management frames show up here as
// high counts with a steady period.
static const uint8_t STATS_MAX_IDS = 48;
