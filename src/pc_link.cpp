/**
 * @file pc_link.cpp
 * @brief Companion commands for the ZGW clamp switches, plus a log stream.
 *
 * Commands (one JSON object per line):
 *   {"cmd":"ping"}
 *   {"cmd":"get"}
 *   {"cmd":"kl","kl30":1,"kl15":1}
 *   {"cmd":"fa","hex":"03463031..."}
 *   {"cmd":"vcm","istufe":"F025-18-03-520","werk":"...","ho":"..."}
 *
 * Reply:
 *   {"ok":1,"kl30":1,"kl15":1}
 *   {"ok":1,"fa":"G020 5V51 1119"}
 *   {"ok":1,"vcm":1}
 *
 * Unsolicited lines to the last UDP peer:
 *   {"ev":"eth","msg":"..."}
 *   {"ev":"can","id":"12F","dlc":8,"data":"45 FF ..."}
 */

#include "pc_link.h"
#include "bench_vin.h"
#include "bmw_frames.h"
#include "config.h"
#include "vehicle_fa.h"

#include <Arduino.h>
#include <ETH.h>
#include <WiFiUdp.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace pc_link {

namespace {

WiFiUDP g_udp;
bool g_udpStarted = false;
char g_serialBuf[256];
size_t g_serialLen = 0;

IPAddress g_peerIp;
uint16_t g_peerPort = 0;

constexpr uint16_t kCompanionUdpPort = 13401;
constexpr int kEthHist = 16;

char g_ethHist[kEthHist][140];
int g_ethHistN = 0;

struct NoteItem {
  char text[140];
  uint8_t eth;
};

QueueHandle_t g_notes = nullptr;

float parseFloatField(const char* json, const char* key, bool* found) {
  *found = false;
  char pat[32];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char* p = strstr(json, pat);
  if (!p) return 0.f;
  p = strchr(p + strlen(pat), ':');
  if (!p) return 0.f;
  p++;
  while (*p == ' ' || *p == '\t') p++;
  *found = true;
  return strtof(p, nullptr);
}

int parseIntField(const char* json, const char* key, bool* found) {
  return (int)lroundf(parseFloatField(json, key, found));
}

bool parseStringField(const char* json, const char* key, char* out, size_t outLen) {
  if (!out || outLen == 0) return false;
  out[0] = '\0';
  char pat[32];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char* p = strstr(json, pat);
  if (!p) return false;
  p = strchr(p + strlen(pat), ':');
  if (!p) return false;
  p = strchr(p, '"');
  if (!p) return false;
  ++p;
  size_t n = 0;
  while (p[n] && p[n] != '"' && n + 1 < outLen) {
    out[n] = p[n];
    n++;
  }
  if (p[n] != '"') return false;
  out[n] = '\0';
  return true;
}

const char* parseCmd(const char* json) {
  const char* p = strstr(json, "\"cmd\"");
  if (!p) return nullptr;
  p = strchr(p, ':');
  if (!p) return nullptr;
  p = strchr(p, '"');
  if (!p) return nullptr;
  return p + 1;
}

int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool cmdEquals(const char* cmdStart, const char* name) {
  if (!cmdStart) return false;
  const size_t n = strlen(name);
  return strncmp(cmdStart, name, n) == 0 &&
         (cmdStart[n] == '"' || cmdStart[n] == '\0');
}

void rememberEth(const char* line) {
  if (g_ethHistN < kEthHist) {
    snprintf(g_ethHist[g_ethHistN], sizeof(g_ethHist[0]), "%s", line);
    g_ethHistN++;
    return;
  }
  memmove(g_ethHist[0], g_ethHist[1], sizeof(g_ethHist[0]) * (kEthHist - 1));
  snprintf(g_ethHist[kEthHist - 1], sizeof(g_ethHist[0]), "%s", line);
}

void enqueue(const char* line, bool eth) {
  if (!line || !line[0]) return;
  if (eth) rememberEth(line);
  if (!g_notes) return;
  NoteItem item = {};
  snprintf(item.text, sizeof(item.text), "%s", line);
  item.eth = eth ? 1 : 0;
  xQueueSend(g_notes, &item, 0);
}

void replyStatus(Print& out) {
  const bmw::LiveSignals s = bmw::getSignals();
  char vin[18];
  bench_vin::copy(vin);
  out.printf("{\"ok\":1,\"kl30\":%d,\"kl15\":%d,\"vin\":\"%s\"}\n",
             s.kl30On ? 1 : 0, s.ignitionOn ? 1 : 0, vin);
}

void sendUdpLine(const char* line) {
  if (!g_udpStarted || g_peerPort == 0 || !line) return;
  g_udp.beginPacket(g_peerIp, g_peerPort);
  g_udp.write((const uint8_t*)line, strlen(line));
  g_udp.endPacket();
}

void sendEvent(const char* kind, const char* msg) {
  char line[200];
  snprintf(line, sizeof(line), "{\"ev\":\"%s\",\"msg\":\"%s\"}\n", kind, msg);
  sendUdpLine(line);
}

void replayEth() {
  for (int i = 0; i < g_ethHistN; i++) sendEvent("eth", g_ethHist[i]);
}

void handleLine(const char* line, Print& out) {
  if (!line || !line[0]) return;

  const char* cmd = parseCmd(line);
  if (!cmd) {
    out.println(F("{\"ok\":0,\"err\":\"no_cmd\"}"));
    return;
  }

  if (cmdEquals(cmd, "ping") || cmdEquals(cmd, "get")) {
    replyStatus(out);
    return;
  }

  if (cmdEquals(cmd, "vin")) {
    char vin[20];
    if (!parseStringField(line, "vin", vin, sizeof(vin)) || !bench_vin::set(vin)) {
      out.println(F("{\"ok\":0,\"err\":\"bad_vin\"}"));
      return;
    }
    replyStatus(out);
    return;
  }

  if (cmdEquals(cmd, "kl") || cmdEquals(cmd, "ign")) {
    const bmw::LiveSignals cur = bmw::getSignals();
    bool found = false;
    bool kl30 = cur.kl30On;
    bool kl15 = cur.ignitionOn;
    if (strstr(line, "\"kl30\"")) kl30 = parseIntField(line, "kl30", &found) != 0;
    if (strstr(line, "\"kl15\"")) kl15 = parseIntField(line, "kl15", &found) != 0;
    if (strstr(line, "\"on\"")) kl15 = parseIntField(line, "on", &found) != 0;
    if (!strstr(line, "\"kl30\"") && !strstr(line, "\"kl15\"") && !strstr(line, "\"on\"")) {
      out.println(F("{\"ok\":0,\"err\":\"missing_clamp\"}"));
      return;
    }
    bmw::setClamps(kl30, kl15);
    replyStatus(out);
    return;
  }

  if (cmdEquals(cmd, "fa")) {
    static char hex[vehicle_fa::kMaxFa * 2 + 2];
    if (!parseStringField(line, "hex", hex, sizeof(hex))) {
      out.println(F("{\"ok\":0,\"err\":\"bad_fa\"}"));
      return;
    }
    const size_t hexLen = strlen(hex);
    if (hexLen == 0 || (hexLen & 1) || hexLen / 2 > vehicle_fa::kMaxFa) {
      out.println(F("{\"ok\":0,\"err\":\"bad_fa\"}"));
      return;
    }
    static uint8_t blob[vehicle_fa::kMaxFa];
    for (size_t i = 0; i < hexLen; i += 2) {
      const int hi = hexNibble(hex[i]);
      const int lo = hexNibble(hex[i + 1]);
      if (hi < 0 || lo < 0) {
        out.println(F("{\"ok\":0,\"err\":\"bad_fa\"}"));
        return;
      }
      blob[i / 2] = (uint8_t)((hi << 4) | lo);
    }
    if (!vehicle_fa::store(blob, hexLen / 2)) {
      out.println(F("{\"ok\":0,\"err\":\"bad_fa\"}"));
      return;
    }
    char sum[20];
    vehicle_fa::summary(sum);
    out.printf("{\"ok\":1,\"fa\":\"%s\"}\n", sum);
    return;
  }

  if (cmdEquals(cmd, "vcm")) {
    char istufe[20];
    char werk[20];
    char ho[20];
    const bool hasI = parseStringField(line, "istufe", istufe, sizeof(istufe));
    const bool hasW = parseStringField(line, "werk", werk, sizeof(werk));
    const bool hasH = parseStringField(line, "ho", ho, sizeof(ho));
    if (!hasI && !hasW && !hasH) {
      out.println(F("{\"ok\":0,\"err\":\"bad_vcm\"}"));
      return;
    }
    if (!vehicle_fa::setIStufe(hasI ? istufe : nullptr,
                               hasW ? werk : nullptr,
                               hasH ? ho : nullptr)) {
      out.println(F("{\"ok\":0,\"err\":\"bad_vcm\"}"));
      return;
    }
    out.println(F("{\"ok\":1,\"vcm\":1}"));
    return;
  }

  out.println(F("{\"ok\":0,\"err\":\"unknown_cmd\"}"));
}

class UdpReplyPrinter : public Print {
 public:
  IPAddress ip;
  uint16_t port = 0;
  char buf[240];
  size_t len = 0;

  size_t write(uint8_t c) override {
    if (len + 1 >= sizeof(buf)) return 0;
    buf[len++] = (char)c;
    if (c == '\n' && port) {
      g_udp.beginPacket(ip, port);
      g_udp.write((const uint8_t*)buf, len);
      g_udp.endPacket();
      len = 0;
    }
    return 1;
  }
};

void pollSerial() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      g_serialBuf[g_serialLen] = '\0';
      handleLine(g_serialBuf, Serial);
      g_serialLen = 0;
      continue;
    }
    if (g_serialLen + 1 < sizeof(g_serialBuf)) {
      g_serialBuf[g_serialLen++] = c;
    } else {
      g_serialLen = 0;
    }
  }
}

void pollUdp() {
  if (!g_udpStarted) {
    if (ETH.linkUp() && ETH.localIP()[0] != 0) {
      if (g_udp.begin(kCompanionUdpPort)) {
        g_udpStarted = true;
        Serial.printf("[PC] Companion UDP :%u\n", kCompanionUdpPort);
        enqueue("[PC] Companion UDP :13401", true);
      }
    }
    return;
  }

  int n = g_udp.parsePacket();
  while (n > 0) {
    // FA JSON is about a kilobyte. This buffer is static: the task stack
    // is too small for a 1500-byte local plus the reply printer.
    static char buf[1500];
    const int room = (int)sizeof(buf) - 1;
    if (n > room) {
      uint8_t dump[64];
      while (g_udp.read(dump, sizeof(dump)) > 0) {
      }
      g_peerIp = g_udp.remoteIP();
      g_peerPort = g_udp.remotePort();
      UdpReplyPrinter reply;
      reply.ip = g_peerIp;
      reply.port = g_peerPort;
      reply.println(F("{\"ok\":0,\"err\":\"too_big\"}"));
      n = g_udp.parsePacket();
      continue;
    }
    const int r = g_udp.read(buf, room);
    if (r > 0) {
      buf[r] = '\0';
      size_t L = (size_t)r;
      while (L > 0 && (buf[L - 1] == '\n' || buf[L - 1] == '\r')) {
        buf[--L] = '\0';
      }
      const bool newPeer = g_udp.remotePort() != g_peerPort || g_udp.remoteIP() != g_peerIp;
      g_peerIp = g_udp.remoteIP();
      g_peerPort = g_udp.remotePort();
      UdpReplyPrinter reply;
      reply.ip = g_peerIp;
      reply.port = g_peerPort;
      handleLine(buf, reply);
      if (reply.len > 0 && reply.port) {
        g_udp.beginPacket(reply.ip, reply.port);
        g_udp.write((const uint8_t*)reply.buf, reply.len);
        g_udp.endPacket();
        reply.len = 0;
      }
      if (newPeer) {
        NoteItem drop;
        while (g_notes && xQueueReceive(g_notes, &drop, 0) == pdTRUE) {
        }
        replayEth();
      }
    }
    n = g_udp.parsePacket();
  }
}

void drainNotes() {
  if (!g_notes || g_peerPort == 0) return;
  NoteItem item;
  int budget = 8;
  while (budget-- > 0 && xQueueReceive(g_notes, &item, 0) == pdTRUE) {
    if (item.eth) sendEvent("eth", item.text);
    else sendEvent("can", item.text);
  }
}

}  // namespace

void noteLine(const char* line) {
  Serial.println(line);
  enqueue(line, true);
}

void noteCan(uint32_t id, const uint8_t* data, uint8_t dlc) {
  if (!data) return;
  if (dlc > 8) dlc = 8;
  char msg[80];
  int n = snprintf(msg, sizeof(msg), "RX %03X", (unsigned)id);
  for (uint8_t i = 0; i < dlc && n > 0 && n < (int)sizeof(msg) - 4; i++) {
    n += snprintf(msg + n, sizeof(msg) - (size_t)n, " %02X", data[i]);
  }
  Serial.printf("[MCP] %s\n", msg);
  if (g_peerPort == 0) return;
  enqueue(msg, false);
}

bool init() {
  if (!g_notes) g_notes = xQueueCreate(48, sizeof(NoteItem));
  Serial.println(F("[PC] Link ready (UDP :13401, KL30/KL15)"));
  return true;
}

void task(void* /*arg*/) {
  for (;;) {
    pollSerial();
    pollUdp();
    drainNotes();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

}  // namespace pc_link
