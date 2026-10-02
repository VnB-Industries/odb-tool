#include <SPI.h>
#include <mcp_can.h>

#include "config.h"

MCP_CAN CAN0(MCP_CS_PIN);

bool canReady = false;
uint32_t lastInitAttempt = 0;

uint32_t lastLedToggle = 0;
bool ledOn = false;

// non-blocking blink: fast while the MCP2515 isn't initialized, slow heartbeat once on-bus
void updateLed() {
  uint32_t interval = canReady ? 1000 : 150;
  uint32_t now = millis();
  if (now - lastLedToggle >= interval) {
    lastLedToggle = now;
    ledOn = !ledOn;
    digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
  }
}

bool initCan() {
  DEBUG_PRINTLN("[can] initializing MCP2515...");
  if (CAN0.begin(MCP_ANY, MCP_CAN_SPEED, MCP_OSC_FREQ) != CAN_OK) {
    DEBUG_PRINTLN("[can] MCP2515 init failed");
    return false;
  }
  CAN0.setMode(MCP_NORMAL); // leave config mode, go on-bus
  DEBUG_PRINTLN("[can] MCP2515 init OK, on-bus");
  return true;
}

// sends the door command frame as a single standard 11-bit CAN frame (no ISO-TP, fire-and-forget)
void sendDoorFrame(const uint8_t* data) {
  if (!canReady) {
    DEBUG_PRINTLN(F("[door] not ready"));
    return;
  }
  byte result = CAN0.sendMsgBuf(DOOR_CMD_ID, 0 /* standard frame */, 8, const_cast<uint8_t*>(data));
  DEBUG_PRINT(F("[door] sendMsgBuf -> "));
  DEBUG_PRINTLN(result == CAN_OK ? F("OK") : F("ERROR"));
}

void printUsage() {
  DEBUG_PRINTLN(F("Commands: lock, unlock"));
}

void processSerialCommands() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;
  line.toLowerCase();

  if (line == "lock") {
    DEBUG_PRINTLN(F("[door] lock"));
    sendDoorFrame(LOCK_CMD_DATA);
  } else if (line == "unlock") {
    DEBUG_PRINTLN(F("[door] unlock"));
    sendDoorFrame(UNLOCK_CMD_DATA);
  } else {
    printUsage();
  }
}

void handleNotReady() {
  uint32_t now = millis();
  if (now - lastInitAttempt >= MCP_RETRY_INTERVAL_MS) {
    lastInitAttempt = now;
    canReady = initCan();
    if (canReady) printUsage();
  }
}

void setup() {
  Serial.begin(115200);
  DEBUG_PRINTLN("[boot] obd-idtool starting (MCP2515)");

  pinMode(LED_PIN, OUTPUT);
  pinMode(MCP_INT_PIN, INPUT);

  canReady = initCan();
  if (canReady) printUsage();
}

void loop() {
  if (!canReady) {
    handleNotReady();
  } else {
    processSerialCommands();
  }
  updateLed();
}
