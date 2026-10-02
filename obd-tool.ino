#include <SPI.h>
#include <mcp_can.h>

#include "config.h"

MCP_CAN CAN0(MCP_CS_PIN);

bool canReady = false;
uint32_t lastInitAttempt = 0;

uint32_t lastLedToggle = 0;
bool ledOn = false;

// ---------------------------------------------------------------------------
// Bus mode
//
// Normal mode is the default, and on this car that is not a neutral choice but the correct one.
// The gateway is the only other node on the diagnostic segment, so in listen-only - where the
// MCP2515 never ACKs - its NM frame is never acknowledged. It then retransmits forever: the TEC
// climbs to 128, and an error-passive transmitter stops incrementing TEC on ACK errors, so it never
// reaches bus-off and hammers the bus at ~1400 frames/s instead of the normal 1 Hz. Measured.
//
// Listen-only therefore both floods the bus and aborts wakes. Keep it for short observations only.
// ---------------------------------------------------------------------------
bool listenOnly = false;

bool setBusMode(bool listen) {
  if (CAN0.setMode(listen ? MCP_LISTENONLY : MCP_NORMAL) != MCP2515_OK) return false;
  listenOnly = listen;
  return true;
}

bool initCan() {
  DEBUG_PRINTLN(F("[can] initializing MCP2515..."));
  if (CAN0.begin(MCP_ANY, MCP_CAN_SPEED, MCP_OSC_FREQ) != CAN_OK) {
    DEBUG_PRINTLN(F("[can] MCP2515 init failed"));
    return false;
  }
  setBusMode(false);
  DEBUG_PRINTLN(F("[can] MCP2515 init OK, on-bus (normal - we ACK, as this bus needs)"));
  return true;
}

// ---------------------------------------------------------------------------
// Sniffer state
// ---------------------------------------------------------------------------
struct CapturedFrame {
  uint32_t t;
  uint32_t tLast;   // timestamp of the last repeat
  uint32_t id;
  uint16_t repeat;  // consecutive identical frames collapsed into this entry
  uint8_t len;
  uint8_t data[8];
};

struct IdStat {
  uint32_t id;
  uint32_t count;
  uint32_t firstMs;
  uint32_t lastMs;
};

CapturedFrame wakeCapture[WAKE_CAPTURE_FRAMES];
uint8_t wakeCaptured = 0;
uint32_t wakeCaptureStartMs = 0;
bool wakeArmed = true; // also capture the very first frames heard after boot

IdStat idStats[STATS_MAX_IDS];
uint8_t idStatCount = 0;

bool sniffStream = false;   // print every frame as it arrives
bool busSilent = true;      // assume asleep until something is heard
uint32_t lastRxMs = 0;
uint32_t silenceSinceMs = 0;
uint32_t awakeSinceMs = 0;  // when the current awake period began, to measure how long a wake lasts
uint32_t totalFrames = 0;

void printFrame(const char* tag, uint32_t t, uint32_t id, uint8_t len, const uint8_t* data) {
  Serial.printf("%s %8lu  %03lX  [%u] ", tag, (unsigned long)t, (unsigned long)id, len);
  for (uint8_t i = 0; i < len; i++) Serial.printf("%02X ", data[i]);
  Serial.println();
}

void recordStat(uint32_t id, uint32_t now) {
  for (uint8_t i = 0; i < idStatCount; i++) {
    if (idStats[i].id == id) {
      idStats[i].count++;
      idStats[i].lastMs = now;
      return;
    }
  }
  if (idStatCount < STATS_MAX_IDS) {
    idStats[idStatCount++] = {id, 1, now, now};
  }
}

void printWakeCapture() {
  DEBUG_PRINTF("\n=== first %u distinct frames after silence ===\n", wakeCaptured);
  DEBUG_PRINTLN(F("     t(ms)   +dt  xN   ID  len  data"));
  for (uint8_t i = 0; i < wakeCaptured; i++) {
    const CapturedFrame &f = wakeCapture[i];
    uint32_t dt = (i == 0) ? 0 : f.t - wakeCapture[i - 1].tLast;
    Serial.printf("  %8lu %5lu %3u  %03lX  [%u] ",
                  (unsigned long)f.t, (unsigned long)dt, f.repeat,
                  (unsigned long)f.id, f.len);
    for (uint8_t b = 0; b < f.len; b++) Serial.printf("%02X ", f.data[b]);
    if (f.repeat > 1) Serial.printf(" (repeated %u x over %lu ms)", f.repeat,
                                    (unsigned long)(f.tLast - f.t));
    Serial.println();
  }
  DEBUG_PRINTLN(F("=== end of capture ===\n"));
}

void handleFrame(uint32_t id, uint8_t len, const uint8_t* data, uint32_t now) {
  totalFrames++;
  lastRxMs = now;
  recordStat(id, now);

  // transition from silence back to traffic: this is the wake event
  if (busSilent) {
    busSilent = false;
    awakeSinceMs = now;
    DEBUG_PRINTF("\n*** BUS AWAKE after %lu ms of silence ***\n",
                 (unsigned long)(now - silenceSinceMs));
    wakeArmed = true;
    wakeCaptured = 0;
  }

  if (wakeArmed) {
    uint8_t n = len > 8 ? 8 : len;

    // collapse a repeat of the previous frame instead of consuming a slot: a transmitter whose
    // frame goes unacknowledged retries at line rate and would otherwise fill the whole buffer
    bool sameAsLast = false;
    if (wakeCaptured > 0) {
      CapturedFrame &prev = wakeCapture[wakeCaptured - 1];
      sameAsLast = (prev.id == id && prev.len == n);
      for (uint8_t i = 0; sameAsLast && i < n; i++) sameAsLast = (prev.data[i] == data[i]);
      if (sameAsLast) {
        prev.repeat++;
        prev.tLast = now;
      }
    }

    if (!sameAsLast && wakeCaptured < WAKE_CAPTURE_FRAMES) {
      CapturedFrame &f = wakeCapture[wakeCaptured++];
      f.t = now;
      f.tLast = now;
      f.id = id;
      f.repeat = 1;
      f.len = n;
      for (uint8_t i = 0; i < n; i++) f.data[i] = data[i];
      if (wakeCaptured == 1) wakeCaptureStartMs = now;
    }

    if (wakeCaptured == WAKE_CAPTURE_FRAMES) {
      wakeArmed = false;
      printWakeCapture();
    }
  }

  if (id == DOOR_RESPONSE_ID) {
    printFrame("[door-reply]", now, id, len, data);
  } else if (id == NM_WATCH_ID) {
    // log only when the payload changes, so a long observation stays readable
    static uint8_t lastNm[8];
    static uint8_t lastNmLen = 0;
    bool changed = (lastNmLen != len);
    for (uint8_t i = 0; !changed && i < len; i++) changed = (lastNm[i] != data[i]);
    if (changed) {
      printFrame("[nm]", now, id, len, data);
      lastNmLen = len > 8 ? 8 : len;
      for (uint8_t i = 0; i < lastNmLen; i++) lastNm[i] = data[i];
    }
    if (sniffStream) printFrame("[rx]", now, id, len, data);
  } else if (sniffStream) {
    printFrame("[rx]", now, id, len, data);
  }
}

// The MCP2515 has only two receive buffers, so drain them every pass. On a busy bus frames will
// still be missed; the wake capture is unaffected because it only needs the first few.
void pollCanRx() {
  uint32_t now = millis();
  while (CAN0.checkReceive() == CAN_MSGAVAIL) {
    uint32_t id;
    uint8_t len;
    uint8_t buf[8];
    if (CAN0.readMsgBuf(&id, &len, buf) != CAN_OK) break;
    handleFrame(id, len, buf, now);
  }
}

// Flush a partly-filled capture once the window is up, so a wake that produces only a few distinct
// frames gets printed instead of waiting indefinitely for the buffer to fill.
void checkWakeCaptureTimeout(uint32_t now) {
  if (!wakeArmed || wakeCaptured == 0) return;
  if (now - wakeCaptureStartMs >= WAKE_CAPTURE_MS) {
    wakeArmed = false;
    printWakeCapture();
  }
}

void checkSilence(uint32_t now) {
  if (busSilent) return;
  if (now - lastRxMs >= SILENCE_THRESHOLD_MS) {
    busSilent = true;
    silenceSinceMs = lastRxMs;
    // how long the wake lasted: the number the keepalive trade-off depends on
    DEBUG_PRINTF("\n*** BUS SILENT - asleep (was awake %lu ms) ***\n",
                 (unsigned long)(lastRxMs - awakeSinceMs));
    // arm so the next frame, whenever it comes, starts a fresh capture
    wakeArmed = true;
    wakeCaptured = 0;
  }
}

// ---------------------------------------------------------------------------
// Transmit
// ---------------------------------------------------------------------------
// Sends the door command as a single standard 11-bit CAN frame (no ISO-TP, fire-and-forget).
// Transmitting needs normal mode, so drop out of listen-only for the send and go straight back:
// staying in normal mode would let this tool ACK bus traffic and skew the sleep measurements.
void sendDoorFrame(const uint8_t* data) {
  if (!canReady) {
    DEBUG_PRINTLN(F("[door] not ready"));
    return;
  }

  bool wasListening = listenOnly;
  if (wasListening && !setBusMode(false)) {
    DEBUG_PRINTLN(F("[door] could not enter normal mode"));
    return;
  }

  byte result = CAN0.sendMsgBuf(DOOR_CMD_ID, 0 /* standard frame */, 8, const_cast<uint8_t*>(data));
  if (result == CAN_OK) {
    DEBUG_PRINTLN(F("[door] sendMsgBuf -> OK (watch for a reply on 758)"));
  } else {
    // code 7 = send timeout (nobody ACKed). EFLG bit4 = error-passive, bit5 = bus-off.
    DEBUG_PRINTF("[door] sendMsgBuf -> ERROR code=%u EFLG=0x%02X TEC=%u REC=%u\n",
                 result, CAN0.getError(), CAN0.errorCountTX(), CAN0.errorCountRX());
    if (busSilent) DEBUG_PRINTLN(F("[door] bus is silent - the car is asleep"));
  }

  if (wasListening) setBusMode(true);
}

// Parses "tx <id> <b0> .. <b7>" / "burst <id> ...", all hex. Returns false on a malformed line.
bool parseFrameArgs(const String &args, uint32_t &id, uint8_t *data, uint8_t &len) {
  len = 0;
  int pos = 0;
  bool haveId = false;

  while (pos < (int)args.length()) {
    while (pos < (int)args.length() && args[pos] == ' ') pos++;
    int end = args.indexOf(' ', pos);
    if (end < 0) end = args.length();
    if (end == pos) break;

    String tok = args.substring(pos, end);
    pos = end;

    char *endp = nullptr;
    long v = strtol(tok.c_str(), &endp, 16);
    if (endp == tok.c_str() || *endp != '\0' || v < 0) return false;

    if (!haveId) {
      if (v > 0x7FF) return false; // standard 11-bit only
      id = (uint32_t)v;
      haveId = true;
    } else {
      if (len >= 8 || v > 0xFF) return false;
      data[len++] = (uint8_t)v;
    }
  }
  return haveId && len > 0;
}

// Injects a frame, optionally as a periodic burst, while still watching for the bus to come alive.
void cmdTx(const String &args, bool burst) {
  uint32_t id;
  uint8_t data[8];
  uint8_t len;

  if (!parseFrameArgs(args, id, data, len)) {
    DEBUG_PRINTLN(F("usage: tx <id> <b0> [.. b7]   (hex, e.g. tx 45A 5A 00 80 42 01 0F 0F 0F)"));
    return;
  }
  if (!canReady) {
    DEBUG_PRINTLN(F("[tx] not ready"));
    return;
  }

  bool wasListening = listenOnly;
  if (wasListening && !setBusMode(false)) {
    DEBUG_PRINTLN(F("[tx] could not enter normal mode"));
    return;
  }

  uint16_t reps = burst ? TX_BURST_COUNT : 1;
  DEBUG_PRINTF("[tx] %03lX x%u", (unsigned long)id, reps);
  for (uint8_t i = 0; i < len; i++) DEBUG_PRINTF(" %02X", data[i]);
  Serial.println();

  if (burst) CAN0.enOneShotTX(); // keep the period even instead of retrying failed attempts

  // If we started on a silent bus, stop as soon as it wakes. Injecting an ID that a real ECU also
  // transmits is fine while that ECU is asleep, but once it starts sending the same ID we would be
  // colliding with it - duplicate transmitters on one ID cause error frames, not just noise.
  bool startedSilent = busSilent;
  uint16_t okCount = 0;
  uint16_t ackedCount = 0;
  uint16_t sent = 0;

  for (uint16_t i = 0; i < reps; i++) {
    // In one-shot mode TXREQ clears after a single attempt whether or not the frame was
    // acknowledged, so sendMsgBuf returns CAN_OK either way. A rising transmit error counter is
    // what actually distinguishes them (until it pins at 128 in error-passive).
    uint8_t tecBefore = CAN0.errorCountTX();
    if (CAN0.sendMsgBuf(id, 0, len, data) == CAN_OK) {
      okCount++;
      if (CAN0.errorCountTX() <= tecBefore) ackedCount++;
    }
    sent++;
    // keep draining RX: if the injection wakes the network we want to see it happen
    pollCanRx();
    if (startedSilent && !busSilent) {
      DEBUG_PRINTLN(F("[tx] bus woke - stopping burst so we don't collide with the real sender"));
      break;
    }
    if (burst && i + 1 < reps) {
      uint32_t until = millis() + TX_BURST_GAP_MS;
      while ((int32_t)(until - millis()) > 0) pollCanRx();
      if (startedSilent && !busSilent) {
        DEBUG_PRINTLN(F("[tx] bus woke - stopping burst so we don't collide with the real sender"));
        break;
      }
    }
  }
  reps = sent;

  if (burst) CAN0.disOneShotTX();
  DEBUG_PRINTF("[tx] %u sent, %u completed, %u acked; bus=%s EFLG=0x%02X TEC=%u\n",
               sent, okCount, ackedCount, busSilent ? "SILENT" : "ACTIVE",
               CAN0.getError(), CAN0.errorCountTX());
  if (ackedCount == 0 && sent > 0) {
    DEBUG_PRINTLN(F("[tx] nothing acknowledged - no other node is listening"));
  }
  if (wasListening) setBusMode(true);
}

// Replays the captured gateway startup sequence with its original timing, then holds the steady
// state for a few seconds. One-shot TX throughout so unacknowledged frames aren't retried in
// hardware, and it stops the moment the bus comes up - past that point the real gateway is sending
// this same ID and we must not transmit it too.
void cmdWakeSeq() {
  if (!canReady) {
    DEBUG_PRINTLN(F("[wakeseq] not ready"));
    return;
  }

  bool wasListening = listenOnly;
  if (wasListening && !setBusMode(false)) {
    DEBUG_PRINTLN(F("[wakeseq] could not enter normal mode"));
    return;
  }

  bool startedSilent = busSilent;
  bool woke = false;
  uint16_t sent = 0, acked = 0;

  DEBUG_PRINTF("[wakeseq] replaying %u frames on %03lX, then %u ticks\n",
               WAKE_SEQUENCE_LEN, (unsigned long)WAKESEQ_ID, WAKESEQ_TICKS);
  CAN0.enOneShotTX();

  // wait out a gap while still draining RX; returns true if the bus came up
  auto gap = [&](uint32_t ms) {
    uint32_t until = millis() + ms;
    while ((int32_t)(until - millis()) > 0) pollCanRx();
    return startedSilent && !busSilent;
  };

  auto send = [&](const uint8_t *data) {
    uint8_t tecBefore = CAN0.errorCountTX();
    if (CAN0.sendMsgBuf(WAKESEQ_ID, 0, 8, const_cast<uint8_t *>(data)) == CAN_OK) {
      sent++;
      if (CAN0.errorCountTX() <= tecBefore) acked++;
    }
    pollCanRx();
    return startedSilent && !busSilent;
  };

  for (uint8_t i = 0; i < WAKE_SEQUENCE_LEN && !woke; i++) {
    if (gap(WAKE_SEQUENCE[i].delayMs)) { woke = true; break; }
    if (send(WAKE_SEQUENCE[i].data))   { woke = true; break; }
  }

  const uint8_t *steady = WAKE_SEQUENCE[WAKE_SEQUENCE_LEN - 1].data;
  for (uint8_t t = 0; t < WAKESEQ_TICKS && !woke; t++) {
    if (gap(WAKESEQ_TICK_MS)) { woke = true; break; }
    if (send(steady))         { woke = true; break; }
  }

  CAN0.disOneShotTX();
  DEBUG_PRINTF("[wakeseq] %u sent, %u acked; bus=%s EFLG=0x%02X TEC=%u\n",
               sent, acked, busSilent ? "SILENT" : "ACTIVE",
               CAN0.getError(), CAN0.errorCountTX());
  if (woke) {
    DEBUG_PRINTLN(F("[wakeseq] bus came up - stopped so we don't collide with the gateway"));
  } else if (acked == 0) {
    DEBUG_PRINTLN(F("[wakeseq] nothing acknowledged - the sequence did not wake anything"));
  }

  if (wasListening) setBusMode(true);
}

// ---------------------------------------------------------------------------
// Serial interface
// ---------------------------------------------------------------------------
void printUsage() {
  DEBUG_PRINTLN(F("Commands:"));
  DEBUG_PRINTLN(F("  lock / unlock  send the door frame (briefly switches to normal mode)"));
  DEBUG_PRINTLN(F("  tx <id> <b..>  inject one frame, hex (tx 45A 5A 00 80 42 01 0F 0F 0F)"));
  DEBUG_PRINTLN(F("  burst <id> ..  same frame sent periodically - use this for NM wake attempts"));
  DEBUG_PRINTLN(F("  wakeseq        replay the captured gateway startup sequence on 45A"));
  DEBUG_PRINTLN(F("  sniff          toggle printing every received frame"));
  DEBUG_PRINTLN(F("  stats          per-ID frame counts and average period"));
  DEBUG_PRINTLN(F("  clear          reset stats and re-arm the wake capture"));
  DEBUG_PRINTLN(F("  status         bus state, mode, error counters"));
  DEBUG_PRINTLN(F("  listen/normal  set bus mode manually"));
}

void printStats() {
  uint32_t now = millis();
  DEBUG_PRINTF("\n=== %lu frames, %u distinct IDs ===\n", (unsigned long)totalFrames, idStatCount);
  DEBUG_PRINTLN(F("   ID   count   period(ms)"));
  for (uint8_t i = 0; i < idStatCount; i++) {
    uint32_t span = idStats[i].lastMs - idStats[i].firstMs;
    uint32_t period = (idStats[i].count > 1) ? span / (idStats[i].count - 1) : 0;
    Serial.printf("  %03lX  %6lu   %lu%s\n", (unsigned long)idStats[i].id,
                  (unsigned long)idStats[i].count, (unsigned long)period,
                  (now - idStats[i].lastMs > SILENCE_THRESHOLD_MS) ? "  (stopped)" : "");
  }
  if (idStatCount == STATS_MAX_IDS) DEBUG_PRINTLN(F("  (ID table full - raise STATS_MAX_IDS)"));
  Serial.println();
}

void printStatus() {
  uint32_t now = millis();
  DEBUG_PRINTF("[status] mode=%s bus=%s frames=%lu EFLG=0x%02X TEC=%u REC=%u\n",
               listenOnly ? "listen-only" : "normal",
               busSilent ? "SILENT" : "active",
               (unsigned long)totalFrames, CAN0.getError(),
               CAN0.errorCountTX(), CAN0.errorCountRX());
  if (busSilent && lastRxMs) {
    DEBUG_PRINTF("[status] silent for %lu ms\n", (unsigned long)(now - lastRxMs));
  }
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
  } else if (line.startsWith("tx ")) {
    cmdTx(line.substring(3), false);
  } else if (line.startsWith("burst ")) {
    cmdTx(line.substring(6), true);
  } else if (line == "wakeseq") {
    cmdWakeSeq();
  } else if (line == "sniff") {
    sniffStream = !sniffStream;
    DEBUG_PRINTF("[sniff] %s\n", sniffStream ? "on (serial may drop frames on a busy bus)" : "off");
  } else if (line == "stats") {
    printStats();
  } else if (line == "clear") {
    idStatCount = 0;
    totalFrames = 0;
    wakeCaptured = 0;
    wakeArmed = true;
    DEBUG_PRINTLN(F("[sniff] cleared, wake capture re-armed"));
  } else if (line == "status") {
    printStatus();
  } else if (line == "listen") {
    if (setBusMode(true)) {
      DEBUG_PRINTLN(F("[can] listen-only - WARNING: we stop ACKing, so the gateway's NM frame"));
      DEBUG_PRINTLN(F("[can] retransmits at line rate (~1400/s) and wakes will abort. Short use only."));
    } else {
      DEBUG_PRINTLN(F("[can] mode change failed"));
    }
  } else if (line == "normal") {
    DEBUG_PRINTLN(setBusMode(false) ? F("[can] normal (we ACK bus traffic - correct for this bus)")
                                    : F("[can] mode change failed"));
  } else {
    printUsage();
  }
}

// ---------------------------------------------------------------------------
// non-blocking blink: fast while the MCP2515 isn't initialized, slow heartbeat once on-bus
// ---------------------------------------------------------------------------
void updateLed() {
  uint32_t interval = canReady ? 1000 : 150;
  uint32_t now = millis();
  if (now - lastLedToggle >= interval) {
    lastLedToggle = now;
    ledOn = !ledOn;
    digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
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
  DEBUG_PRINTLN(F("[boot] obd-idtool starting (MCP2515, sniffer)"));

  pinMode(LED_PIN, OUTPUT);
  pinMode(MCP_INT_PIN, INPUT);

  canReady = initCan();
  if (canReady) printUsage();
  silenceSinceMs = millis(); // so the first "BUS AWAKE" reports silence measured from boot
}

void loop() {
  if (!canReady) {
    handleNotReady();
  } else {
    pollCanRx();
    uint32_t now = millis();
    checkWakeCaptureTimeout(now);
    checkSilence(now);
    processSerialCommands();
  }
  updateLed();
}
