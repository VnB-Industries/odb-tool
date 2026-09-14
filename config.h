#pragma once

// ---- Debug ----
#define DEBUG_ENABLED 1 // set to 0 to silence Serial debug output

#if DEBUG_ENABLED
  #define DEBUG_PRINT(...)   Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

// ---- ELM327 Bluetooth Classic (SPP) adapter ----
// MAC address of the paired ELM327 dongle, e.g. from `hcitool scan` or the ESP32 BT scanner sketch.
static const uint8_t ELM327_MAC[6] = {0xDC, 0x0D, 0x30, 0xE2, 0x4B, 0xE1}; // vLinker FD
static const char* const BT_PIN = "1234"; // legacy pairing PIN for the vLinker FD

// ---- GPIO ----
static const uint8_t LED_PIN = 2; // onboard LED, connection status indicator

// ---- Timing ----
static const uint32_t ELM_TIMEOUT_MS           = 1000; // ELM327 response timeout per AT command
static const uint32_t BT_RECONNECT_INTERVAL_MS = 3000; // delay between connection attempts
static const uint32_t DEFAULT_SNIFF_DURATION_MS = 15000; // default "sniff" capture window
static const uint32_t DEFAULT_SCAN_BLOCK_DURATION_MS = 5000; // default per-block capture window for "scan"
static const uint32_t UDS_TIMEOUT_MS           = 3000; // longer window for diagnostic-session responses

// ---- CAN protocol ----
// '6' = ISO 15765-4 CAN, 11-bit ID, 500 kbps (Toyota/Lexus powertrain + body). Locking this avoids "SEARCHING...".
static const char CAN_PROTOCOL = '6';

// ---- Door lock/unlock frame (VERIFIED working on 2018 Lexus CT200h via OBD-II) ----
// Raw single CAN frame (send with ATCAF0). Toyota body active test: sub-addr 0x40, len 0x05,
// service 0x30 (inputOutputControlByLocalIdentifier), LID 0x11 (door lock), control 0x00 XX.
// Frame origin: cydia2020/toyota-can-bus-multitool. Control byte 0x80 = lock, 0x40 = unlock on this car.
static const char* const DOOR_CMD_HEADER = "750";
static const char* const LOCK_CMD_DATA   = "4005301100800000";
static const char* const UNLOCK_CMD_DATA = "4005301100400000";
