/**
 * @file pc_link.cpp
 * @brief Parse companion JSON commands from Serial and UDP port 13401.
 *
 * Protocol (one JSON object per line, \\n terminated):
 *   {"cmd":"ping"}
 *   {"cmd":"get"}
 *   {"cmd":"ign","on":1}
 *   {"cmd":"sig","rpm":1500,"spd":60.5,"fuel":75,"clt":90}
 *   {"cmd":"ign","on":0}   // Terminal 15 OFF
 *
 * Responses (also JSON line):
 *   {"ok":1,"ign":1,"rpm":1500,"spd":60.5,"fuel":75,"clt":90}
 *   {"ok":0,"err":"..."}
 */

#include "pc_link.h"
#include "bmw_frames.h"
#include "config.h"

#include <Arduino.h>
#include <ETH.h>
#include <WiFiUdp.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace pc_link {

namespace {

WiFiUDP g_udp;
bool g_udpStarted = false;
char g_serialBuf[256];
size_t g_serialLen = 0;

constexpr uint16_t kCompanionUdpPort = 13401;

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

const char* parseCmd(const char* json) {
  const char* p = strstr(json, "\"cmd\"");
  if (!p) return nullptr;
  p = strchr(p, ':');
  if (!p) return nullptr;
  p = strchr(p, '"');
  if (!p) return nullptr;
  return p + 1;  // start of command token
}

bool cmdEquals(const char* cmdStart, const char* name) {
  if (!cmdStart) return false;
  const size_t n = strlen(name);
  return strncmp(cmdStart, name, n) == 0 &&
         (cmdStart[n] == '"' || cmdStart[n] == '\0');
}

void replyStatus(Print& out) {
  const bmw::LiveSignals s = bmw::getSignals();
  out.printf("{\"ok\":1,\"ign\":%d,\"rpm\":%u,\"spd\":%.1f,\"fuel\":%.1f,\"clt\":%d}\n",
             s.ignitionOn ? 1 : 0, (unsigned)s.rpm, (double)s.speedKmh,
             (double)s.fuelPct, (int)s.coolantC);
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

  if (cmdEquals(cmd, "ign")) {
    bool found = false;
    const int on = parseIntField(line, "on", &found);
    if (!found) {
      out.println(F("{\"ok\":0,\"err\":\"missing_on\"}"));
      return;
    }
    bmw::setIgnition(on != 0);
    replyStatus(out);
    return;
  }

  if (cmdEquals(cmd, "sig")) {
    bool f = false;
    if (strstr(line, "\"rpm\"")) {
      bmw::setRpm((uint16_t)parseIntField(line, "rpm", &f));
    }
    if (strstr(line, "\"spd\"")) {
      bmw::setSpeedKmh(parseFloatField(line, "spd", &f));
    }
    if (strstr(line, "\"fuel\"")) {
      bmw::setFuelPct(parseFloatField(line, "fuel", &f));
    }
    if (strstr(line, "\"clt\"")) {
      bmw::setCoolantC((int16_t)parseIntField(line, "clt", &f));
    }
    replyStatus(out);
    return;
  }

  out.println(F("{\"ok\":0,\"err\":\"unknown_cmd\"}"));
}

/** Lightweight Print adapter that sends one UDP datagram to last peer. */
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
      g_serialLen = 0;  // overflow — resync
    }
  }
}

void pollUdp() {
  if (!g_udpStarted) {
    // Start once Ethernet has an IP (DoIP task brings ETH up)
    if (ETH.linkUp() && ETH.localIP()[0] != 0) {
      if (g_udp.begin(kCompanionUdpPort)) {
        g_udpStarted = true;
        Serial.printf("[PC] Companion UDP :%u\n", kCompanionUdpPort);
      }
    }
    return;
  }

  int n = g_udp.parsePacket();
  while (n > 0) {
    char buf[256];
    const int r = g_udp.read(buf, sizeof(buf) - 1);
    if (r > 0) {
      buf[r] = '\0';
      // Trim trailing CR/LF
      size_t L = (size_t)r;
      while (L > 0 && (buf[L - 1] == '\n' || buf[L - 1] == '\r')) {
        buf[--L] = '\0';
      }
      UdpReplyPrinter reply;
      reply.ip = g_udp.remoteIP();
      reply.port = g_udp.remotePort();
      handleLine(buf, reply);
      if (reply.len > 0 && reply.port) {
        g_udp.beginPacket(reply.ip, reply.port);
        g_udp.write((const uint8_t*)reply.buf, reply.len);
        g_udp.endPacket();
        reply.len = 0;
      }
    }
    n = g_udp.parsePacket();
  }
}

}  // namespace

bool init() {
  Serial.println(F("[PC] Link ready (USB-Serial JSON + UDP :13401)"));
  return true;
}

void task(void* /*arg*/) {
  for (;;) {
    pollSerial();
    pollUdp();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

}  // namespace pc_link
