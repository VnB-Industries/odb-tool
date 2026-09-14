#include <BluetoothSerial.h>
#include <ELMduino.h>
#include <string.h>

#include "config.h"

enum LinkState { DISCONNECTED, CONNECTING, CONNECTED };

BluetoothSerial SerialBT;
ELM327 elm327;

LinkState linkState = DISCONNECTED;
uint32_t lastConnectAttempt = 0;

uint16_t responderTx[32]; // diagnostic request IDs that answered the last ecuscan
uint8_t responderCount = 0;

uint32_t lastLedToggle = 0;
bool ledOn = false;

// non-blocking blink: fast while searching for the adapter, slow heartbeat once connected
void updateLed() {
  uint32_t interval = (linkState == CONNECTED) ? 1000 : 150;
  uint32_t now = millis();
  if (now - lastLedToggle >= interval) {
    lastLedToggle = now;
    ledOn = !ledOn;
    digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
  }
}

bool connectToELM327() {
  DEBUG_PRINTLN("[bt] connecting to ELM327...");
  if (!SerialBT.connect(const_cast<uint8_t*>(ELM327_MAC))) {
    DEBUG_PRINTLN("[bt] SPP connect failed");
    return false;
  }
  bool ok = elm327.begin(SerialBT, false, ELM_TIMEOUT_MS, CAN_PROTOCOL); // lock protocol so it never re-searches
  if (ok) {
    // Warm up with an unfiltered known-good query so the protocol is confirmed/locked before any ATCRA filter;
    // otherwise a later receive filter hides the only responder and the adapter searches forever.
    elm327.sendCommand_Blocking("ATSP6"); // set, no auto-search
    elm327.sendCommand_Blocking("ATCRA"); // ensure no leftover receive filter
    elm327.sendCommand_Blocking("0100");  // powertrain answers -> protocol becomes active
  }
  DEBUG_PRINTLN(ok ? "[bt] ELM327 init OK" : "[bt] ELM327 init failed");
  return ok;
}

void printUsage() {
  DEBUG_PRINTLN(F("Commands:"));
  DEBUG_PRINTLN(F("  sniff [durationMs]       - monitor all CAN traffic (ATMA) for durationMs"));
  DEBUG_PRINTLN(F("  send <header> <data>     - set header via ATSH and transmit <data> hex bytes"));
  DEBUG_PRINTLN(F("  filter <mask> <id>       - narrow sniff to IDs matching <id> under <mask> (ATCM/ATCF)"));
  DEBUG_PRINTLN(F("  filter clear             - remove filtering, sniff sees every ID again"));
  DEBUG_PRINTLN(F("  scan [durationMsPerBlock]- step through the 8 standard 0x100 ID blocks, pausing between each"));
  DEBUG_PRINTLN(F("  lock / unlock            - inject the candidate body frame from config.h (fire-and-forget)"));
  DEBUG_PRINTLN(F("  uds <tx> <rx> <data>     - diagnostic request to one ECU with rx filter + flow control"));
  DEBUG_PRINTLN(F("  ecuscan                  - probe 0x700-0x7EF and list which ECUs answer (rx = tx+8)"));
  DEBUG_PRINTLN(F("  idscan                   - read part-number/VIN/version DIDs from ecuscan responders to name them"));
  DEBUG_PRINTLN(F("  tryunlock [tx] [rx]      - step through candidate body-ECU UDS unlock sequences (default 750/758)"));
  DEBUG_PRINTLN(F("  if sniff reports BUFFER FULL, the bus is too busy for ATMA - narrow it with 'filter' first"));
}

// blocks until a full line is available on Serial, so 'scan' can pause between blocks for the user
String waitForLine() {
  while (!Serial.available()) {
    delay(10);
  }
  String line = Serial.readStringUntil('\n');
  line.trim();
  return line;
}

// restricts ATMA to IDs where (id & mask) == (filterHex & mask), to keep busy buses under the BT link's throughput
void setCanFilter(const char* maskHex, const char* filterHex) {
  if (linkState != CONNECTED) {
    DEBUG_PRINTLN(F("[filter] not connected"));
    return;
  }

  char cmd[16];
  snprintf(cmd, sizeof(cmd), "ATCM%s", maskHex);
  elm327.sendCommand_Blocking(cmd);
  DEBUG_PRINT(F("[filter] ")); DEBUG_PRINT(cmd); DEBUG_PRINT(F(" -> ")); DEBUG_PRINTLN(elm327.payload);

  snprintf(cmd, sizeof(cmd), "ATCF%s", filterHex);
  elm327.sendCommand_Blocking(cmd);
  DEBUG_PRINT(F("[filter] ")); DEBUG_PRINT(cmd); DEBUG_PRINT(F(" -> ")); DEBUG_PRINTLN(elm327.payload);
}

void clearCanFilter() {
  setCanFilter("000", "000"); // mask 000 = don't-care on every bit = accept all IDs
  DEBUG_PRINTLN(F("[filter] cleared (accepting all IDs)"));
}

// prints a collapsed run of identical lines once, so repeated noise doesn't bury the frame that changes
void flushDedupedLine(const char* line, uint32_t repeatCount, uint32_t timestampMs) {
  if (line[0] == '\0') return;
  DEBUG_PRINT(timestampMs);
  DEBUG_PRINT(F(" ms  "));
  DEBUG_PRINT(line);
  if (repeatCount > 1) {
    DEBUG_PRINT(F("  (x"));
    DEBUG_PRINT(repeatCount);
    DEBUG_PRINT(F(")"));
  }
  DEBUG_PRINTLN();
}

// dumps raw bus traffic to Serial so lock/unlock frames can be spotted while toggling the door switch
void sniffCanTraffic(uint32_t durationMs) {
  if (linkState != CONNECTED) {
    DEBUG_PRINTLN(F("[sniff] not connected"));
    return;
  }

  DEBUG_PRINTLN(F("[sniff] enabling headers (ATH1)"));
  elm327.sendCommand_Blocking("ATH1");

  DEBUG_PRINT(F("[sniff] starting monitor for "));
  DEBUG_PRINT(durationMs);
  DEBUG_PRINTLN(F(" ms - toggle the door lock switch now"));

  while (SerialBT.available()) SerialBT.read(); // clear stale bytes before starting
  SerialBT.print(F("ATMA\r"));

  char lineBuf[64];
  uint8_t lineLen = 0;
  char lastLine[64] = {0};
  uint32_t repeatCount = 0;
  uint32_t lastLineTime = 0;
  uint32_t start = millis();
  while (millis() - start < durationMs) {
    while (SerialBT.available()) {
      char c = SerialBT.read();
      if (c == '\r' || c == '\n') {
        if (lineLen > 0) {
          lineBuf[lineLen] = '\0';
          if (strcmp(lineBuf, lastLine) == 0) {
            repeatCount++;
          } else {
            flushDedupedLine(lastLine, repeatCount, lastLineTime);
            strncpy(lastLine, lineBuf, sizeof(lastLine) - 1);
            lastLine[sizeof(lastLine) - 1] = '\0';
            repeatCount = 1;
            lastLineTime = millis() - start;
          }
          lineLen = 0;
        }
      } else if (lineLen < sizeof(lineBuf) - 1) {
        lineBuf[lineLen++] = c;
      }
    }
  }
  flushDedupedLine(lastLine, repeatCount, lastLineTime); // flush the final run

  DEBUG_PRINTLN(F("[sniff] stopping monitor mode"));
  SerialBT.write(' '); // any character halts ATMA per the ELM327 datasheet
  delay(300);
  while (SerialBT.available()) SerialBT.read(); // discard trailing bytes/prompt

  DEBUG_PRINTLN(F("[sniff] resyncing adapter"));
  elm327.sendCommand_Blocking("AT");
  DEBUG_PRINTLN(F("[sniff] done"));
}

// automates the manual filter+sniff binary search: walks the 8 standard 0x100 ID blocks one at a time,
// pausing for the user to get ready (and toggle the switch) before each block's capture window
void scanIdBlocks(uint32_t durationMsPerBlock) {
  if (linkState != CONNECTED) {
    DEBUG_PRINTLN(F("[scan] not connected"));
    return;
  }

  static const char* const BLOCK_IDS[] = {"000", "100", "200", "300", "400", "500", "600", "700"};
  for (uint8_t i = 0; i < 8; i++) {
    DEBUG_PRINT(F("[scan] block ")); DEBUG_PRINT(i + 1); DEBUG_PRINT(F("/8: filter 700 "));
    DEBUG_PRINTLN(BLOCK_IDS[i]);
    setCanFilter("700", BLOCK_IDS[i]);

    DEBUG_PRINTLN(F("[scan] get ready, then press Enter to start this block's capture"));
    waitForLine();

    sniffCanTraffic(durationMsPerBlock);

    DEBUG_PRINTLN(F("[scan] press Enter for the next block, or type 'stop' + Enter to end the scan"));
    String resp = waitForLine();
    resp.toLowerCase();
    if (resp == "stop") {
      DEBUG_PRINTLN(F("[scan] stopped early"));
      clearCanFilter();
      return;
    }
  }

  clearCanFilter();
  DEBUG_PRINTLN(F("[scan] complete, filter cleared"));
}

// fire-and-forget raw CAN frame injection: CAF0 sends the bytes as one literal frame (no ISO-TP/flow control),
// ATR0 stops the ELM327 blocking for a reply the ECU never sends. Restores CAF1/ATR1 afterward for UDS.
void sendTestFrame(const char* headerHex, const char* dataHex) {
  if (linkState != CONNECTED) {
    DEBUG_PRINTLN(F("[send] not connected"));
    return;
  }

  char cmd[24];
  elm327.sendCommand_Blocking("ATCAF0"); // CAN auto-formatting off: raw 8-byte frame, no ISO-TP framing
  snprintf(cmd, sizeof(cmd), "ATSH%s", headerHex);
  DEBUG_PRINT(F("[send] ")); DEBUG_PRINTLN(cmd);
  elm327.sendCommand_Blocking(cmd);

  elm327.sendCommand_Blocking("ATR0"); // responses off: send one-way and return immediately
  DEBUG_PRINT(F("[send] ")); DEBUG_PRINTLN(dataHex);
  elm327.sendCommand_Blocking(dataHex);
  DEBUG_PRINT(F("[send] response: ")); DEBUG_PRINTLN(elm327.payload);
  elm327.sendCommand_Blocking("ATR1");   // restore normal request/response behavior
  elm327.sendCommand_Blocking("ATCAF1"); // restore auto-formatting for UDS/PID commands
}

// full UDS/diagnostic exchange to one ECU: sets tx header + rx filter, keeps responses on, prints the reply.
// ELM327 handles ISO-TP (multi-frame + flow control) automatically once the headers are set.
void sendUdsRequest(const char* txHeader, const char* rxHeader, const char* dataHex) {
  if (linkState != CONNECTED) {
    DEBUG_PRINTLN(F("[uds] not connected"));
    return;
  }

  char cmd[24];
  elm327.timeout_ms = UDS_TIMEOUT_MS; // diagnostic-session replies can be slower than a PID read
  elm327.sendCommand_Blocking("ATR1"); // ensure we wait for the ECU's response

  snprintf(cmd, sizeof(cmd), "ATSH%s", txHeader);
  DEBUG_PRINT(F("[uds] ")); DEBUG_PRINTLN(cmd);
  elm327.sendCommand_Blocking(cmd);

  snprintf(cmd, sizeof(cmd), "ATCRA%s", rxHeader);
  DEBUG_PRINT(F("[uds] ")); DEBUG_PRINTLN(cmd);
  elm327.sendCommand_Blocking(cmd);

  snprintf(cmd, sizeof(cmd), "ATFCSH%s", txHeader); // flow-control uses the same tx header
  elm327.sendCommand_Blocking(cmd);
  elm327.sendCommand_Blocking("ATFCSD300000"); // flow control data: clear-to-send, no block/separation limit
  elm327.sendCommand_Blocking("ATFCSM1");       // use our configured flow control

  DEBUG_PRINT(F("[uds] req ")); DEBUG_PRINTLN(dataHex);
  elm327.sendCommand_Blocking(dataHex);
  DEBUG_PRINT(F("[uds] resp: ")); DEBUG_PRINTLN(elm327.payload);
  elm327.timeout_ms = ELM_TIMEOUT_MS; // restore default for other commands

  elm327.sendCommand_Blocking("ATCRA"); // clear the receive filter for subsequent commands
}

// probes the 11-bit diagnostic ID range and reports which ECUs answer, to locate the one that owns the doors.
// Any reply (positive 7E.. or negative 7F..) proves an ECU is present at that address.
void scanEcus() {
  if (linkState != CONNECTED) {
    DEBUG_PRINTLN(F("[ecuscan] not connected"));
    return;
  }

  DEBUG_PRINTLN(F("[ecuscan] probing tx 0x700-0x7EF with tester-present (3E00), rx = tx+8..."));
  uint16_t prevTimeout = elm327.timeout_ms;
  elm327.timeout_ms = 300; // short per-probe timeout so the sweep isn't glacial
  char cmd[16];
  responderCount = 0;

  for (uint16_t tx = 0x700; tx <= 0x7EF; tx++) {
    uint16_t rx = tx + 8;
    snprintf(cmd, sizeof(cmd), "ATSH%03X", tx);
    elm327.sendCommand_Blocking(cmd);
    snprintf(cmd, sizeof(cmd), "ATCRA%03X", rx);
    elm327.sendCommand_Blocking(cmd);
    elm327.sendCommand_Blocking("3E00");
    if (elm327.nb_rx_state == ELM_SUCCESS) {
      DEBUG_PRINT(F("[ecuscan] responder tx 0x"));
      DEBUG_PRINT(String(tx, HEX)); DEBUG_PRINT(F(" rx 0x")); DEBUG_PRINT(String(rx, HEX));
      DEBUG_PRINT(F("  resp: ")); DEBUG_PRINTLN(elm327.payload);
      if (responderCount < sizeof(responderTx) / sizeof(responderTx[0])) {
        responderTx[responderCount++] = tx;
      }
    }
  }

  elm327.sendCommand_Blocking("ATCRA");
  elm327.timeout_ms = prevTimeout;
  DEBUG_PRINT(F("[ecuscan] done, responders found: ")); DEBUG_PRINTLN(responderCount);
}

// prints the hex payload as ASCII (non-printable bytes shown as '.'), to read Toyota part-number strings
void printAsciiOfHex(const char* hex) {
  size_t len = strlen(hex);
  DEBUG_PRINT(F("  ascii: "));
  for (size_t i = 0; i + 1 < len; i += 2) {
    char h[3] = {hex[i], hex[i + 1], '\0'};
    uint8_t b = (uint8_t)strtol(h, NULL, 16);
    DEBUG_PRINT((b >= 0x20 && b < 0x7F) ? (char)b : '.');
  }
  DEBUG_PRINTLN();
}

// reads identification DIDs from each responder found by ecuscan, to name ECUs (e.g. body-ECU part number)
void readEcuIds() {
  if (linkState != CONNECTED) {
    DEBUG_PRINTLN(F("[idscan] not connected"));
    return;
  }
  if (responderCount == 0) {
    DEBUG_PRINTLN(F("[idscan] run 'ecuscan' first to find responders"));
    return;
  }

  uint16_t prevTimeout = elm327.timeout_ms;
  elm327.timeout_ms = 500;
  elm327.sendCommand_Blocking("ATH0"); // headers off so payload is just the data bytes

  // Mix of legacy Toyota/KWP identification (0x1A, positive reply 0x5A) and modern UDS (0x22, positive 0x62).
  // Older cars answer 1A88/1A80; newer ones answer 22F187/22F190. Print every reply so 7F negatives show too.
  static const char* const ID_REQS[] = {"1A88", "1A80", "1A87", "22F187", "22F190"};
  char cmd[16];

  for (uint8_t i = 0; i < responderCount; i++) {
    uint16_t tx = responderTx[i];
    uint16_t rx = tx + 8;
    DEBUG_PRINT(F("[idscan] ECU tx 0x")); DEBUG_PRINTLN(String(tx, HEX));
    snprintf(cmd, sizeof(cmd), "ATSH%03X", tx);
    elm327.sendCommand_Blocking(cmd);
    snprintf(cmd, sizeof(cmd), "ATCRA%03X", rx);
    elm327.sendCommand_Blocking(cmd);

    for (uint8_t d = 0; d < sizeof(ID_REQS) / sizeof(ID_REQS[0]); d++) {
      elm327.sendCommand_Blocking(ID_REQS[d]);
      if (elm327.nb_rx_state == ELM_SUCCESS && strlen(elm327.payload) > 0) {
        DEBUG_PRINT(F("  ")); DEBUG_PRINT(ID_REQS[d]);
        DEBUG_PRINT(F(" -> ")); DEBUG_PRINTLN(elm327.payload);
        if (strncmp(elm327.payload, "7F", 2) != 0) {
          printAsciiOfHex(elm327.payload); // only worth decoding a positive reply
        }
      }
    }
  }

  elm327.sendCommand_Blocking("ATCRA");
  elm327.timeout_ms = prevTimeout;
  DEBUG_PRINTLN(F("[idscan] done - a body-ECU part number identifies the door-lock controller"));
}

// walks documented Toyota body-ECU diagnostic candidates so the user can watch which one the doors react to.
// Sequences are best-guesses (physical addr, extended session, then an IO-control/active-test unlock) - the
// point is to observe the ECU replies (positive vs 7F negative) and any door movement, not a guaranteed hit.
void tryBodyEcuUnlock(const char* tx, const char* rx) {
  if (linkState != CONNECTED) {
    DEBUG_PRINTLN(F("[tryunlock] not connected"));
    return;
  }

  DEBUG_PRINT(F("[tryunlock] target ECU tx ")); DEBUG_PRINT(tx);
  DEBUG_PRINT(F(" rx ")); DEBUG_PRINTLN(rx);

  // This car is legacy Toyota (service 0x1A works, 0x22 rejected), so try Toyota/KWP sessions first.
  static const char* const SESSIONS[] = {"1081", "1085", "1003", "1001"};
  for (uint8_t s = 0; s < sizeof(SESSIONS) / sizeof(SESSIONS[0]); s++) {
    DEBUG_PRINT(F("[tryunlock] session ")); DEBUG_PRINTLN(SESSIONS[s]);
    sendUdsRequest(tx, rx, SESSIONS[s]);
    delay(150);
  }

  // 30 = inputOutputControlByLocalIdentifier (legacy active test). 30 11 00 80 = door-lock LID 0x11, unlock;
  // 30 11 00 40 = lock (decoded from the cydia2020 body frame). The 2F/31 variants are UDS fallbacks.
  static const char* const CANDIDATES[] = {
    "30110080",     // legacy active-test: door lock LID 0x11, unlock
    "30110040",     // legacy active-test: door lock LID 0x11, lock
    "2FF530030001", // UDS IO control fallback (unlikely on this car)
    "31010F5001",   // UDS routineControl fallback
  };

  for (uint8_t i = 0; i < sizeof(CANDIDATES) / sizeof(CANDIDATES[0]); i++) {
    DEBUG_PRINT(F("[tryunlock] candidate ")); DEBUG_PRINT(i + 1);
    DEBUG_PRINT(F("/")); DEBUG_PRINT(sizeof(CANDIDATES) / sizeof(CANDIDATES[0]));
    DEBUG_PRINT(F(": ")); DEBUG_PRINTLN(CANDIDATES[i]);
    sendUdsRequest(tx, rx, CANDIDATES[i]);
    DEBUG_PRINTLN(F("[tryunlock] did the doors move? press Enter for next, 'stop' + Enter to end"));
    String resp = waitForLine();
    resp.toLowerCase();
    if (resp == "stop") {
      DEBUG_PRINTLN(F("[tryunlock] stopped"));
      return;
    }
  }
  DEBUG_PRINTLN(F("[tryunlock] all candidates tried - if none worked, capture Techstream (Route A)"));
}

void processSerialCommands() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  int sp = line.indexOf(' ');
  String cmd = (sp == -1) ? line : line.substring(0, sp);
  String rest = (sp == -1) ? "" : line.substring(sp + 1);
  cmd.toLowerCase();
  rest.trim();

  if (cmd == "sniff") {
    uint32_t duration = (rest.length() > 0) ? (uint32_t)rest.toInt() : DEFAULT_SNIFF_DURATION_MS;
    sniffCanTraffic(duration);
  } else if (cmd == "scan") {
    uint32_t duration = (rest.length() > 0) ? (uint32_t)rest.toInt() : DEFAULT_SCAN_BLOCK_DURATION_MS;
    scanIdBlocks(duration);
  } else if (cmd == "lock") {
    sendTestFrame(DOOR_CMD_HEADER, LOCK_CMD_DATA);
  } else if (cmd == "unlock") {
    sendTestFrame(DOOR_CMD_HEADER, UNLOCK_CMD_DATA);
  } else if (cmd == "filter") {
    if (rest == "clear") {
      clearCanFilter();
      return;
    }
    int sp2 = rest.indexOf(' ');
    if (sp2 == -1) {
      printUsage();
      return;
    }
    String mask = rest.substring(0, sp2);
    String id = rest.substring(sp2 + 1);
    mask.trim();
    id.trim();
    setCanFilter(mask.c_str(), id.c_str());
  } else if (cmd == "send") {
    int sp2 = rest.indexOf(' ');
    if (sp2 == -1) {
      printUsage();
      return;
    }
    String header = rest.substring(0, sp2);
    String data = rest.substring(sp2 + 1);
    header.trim();
    data.trim();
    sendTestFrame(header.c_str(), data.c_str());
  } else if (cmd == "uds") {
    int sp2 = rest.indexOf(' ');
    int sp3 = (sp2 == -1) ? -1 : rest.indexOf(' ', sp2 + 1);
    if (sp3 == -1) {
      DEBUG_PRINTLN(F("usage: uds <txHeader> <rxHeader> <dataHex>"));
      return;
    }
    String tx = rest.substring(0, sp2);
    String rx = rest.substring(sp2 + 1, sp3);
    String data = rest.substring(sp3 + 1);
    tx.trim();
    rx.trim();
    data.trim();
    sendUdsRequest(tx.c_str(), rx.c_str(), data.c_str());
  } else if (cmd == "ecuscan") {
    scanEcus();
  } else if (cmd == "idscan") {
    readEcuIds();
  } else if (cmd == "tryunlock") {
    int sp2 = rest.indexOf(' ');
    if (sp2 == -1) {
      tryBodyEcuUnlock("750", "758"); // default guess if no address given
    } else {
      String tx = rest.substring(0, sp2);
      String rx = rest.substring(sp2 + 1);
      tx.trim();
      rx.trim();
      tryBodyEcuUnlock(tx.c_str(), rx.c_str());
    }
  } else {
    printUsage();
  }
}

void handleDisconnected() {
  uint32_t now = millis();
  if (now - lastConnectAttempt >= BT_RECONNECT_INTERVAL_MS) {
    lastConnectAttempt = now;
    linkState = CONNECTING;
    if (connectToELM327()) {
      linkState = CONNECTED;
      printUsage();
    } else {
      DEBUG_PRINTLN("[bt] connection attempt failed, will retry");
      SerialBT.disconnect();
      linkState = DISCONNECTED;
    }
  }
}

void handleConnected() {
  if (!SerialBT.connected()) {
    DEBUG_PRINTLN("[bt] link dropped");
    linkState = DISCONNECTED;
    return;
  }
  processSerialCommands();
}

void setup() {
  Serial.begin(115200);
  DEBUG_PRINTLN("[boot] obd-idtool starting");

  pinMode(LED_PIN, OUTPUT);

  SerialBT.begin("obd-idtool", true); // master mode, to connect out to the ELM327
  SerialBT.setPin(BT_PIN, strlen(BT_PIN)); // legacy pairing PIN required by the vLinker FD
}

void loop() {
  switch (linkState) {
    case DISCONNECTED:
      handleDisconnected();
      break;
    case CONNECTING:
      // connectToELM327() is synchronous; this state is only transient
      break;
    case CONNECTED:
      handleConnected();
      break;
  }
  updateLed();
}
