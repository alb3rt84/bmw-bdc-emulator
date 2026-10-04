/**
 * @file doip_server.cpp
 * @brief DoIP UDP discovery + TCP sessions → shared uds_bdc handler.
 */

#include "doip_server.h"
#include "bench_vin.h"
#include "bmw_frames.h"
#include "config.h"
#include "kcan_gw.h"
#include "pc_link.h"
#include "uds_bdc.h"

#include <ETH.h>
#include <Preferences.h>
#include <WiFi.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <driver/gpio.h>
#include <esp_system.h>
#include <lwip/netif.h>
#include <lwip/sockets.h>
#include <stdio.h>
#include <stdarg.h>

// Set after a full PHY scan fails, so one power cycle does not reboot forever.
// Cleared by a real power loss.
RTC_DATA_ATTR static uint8_t g_ethScanDone = 0;

namespace doip {

void notef(const char* fmt, ...) {
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  pc_link::noteLine(buf);
}

constexpr uint8_t kDoipVersion    = 0x02;
constexpr uint8_t kDoipInvVersion = 0xFD;

constexpr uint16_t kPtVehicleIdentReq      = 0x0001;
constexpr uint16_t kPtVehicleIdentReqEin   = 0x0002;
constexpr uint16_t kPtVehicleIdentReqVin   = 0x0003;
constexpr uint16_t kPtVehicleAnnounce      = 0x0004;
constexpr uint16_t kPtRoutingActivationReq = 0x0005;
constexpr uint16_t kPtRoutingActivationRes = 0x0006;
constexpr uint16_t kPtDiagnosticMessage    = 0x8001;
constexpr uint16_t kPtDiagnosticMessageAck = 0x8002;
constexpr uint16_t kPtDiagnosticMessageNack = 0x8003;
constexpr uint16_t kPtPowerModeReq         = 0x4003;
constexpr uint16_t kPtPowerModeRes         = 0x4004;

// ISO 13400 announcement is 33 bytes once the VIN/GID sync byte is included.
// EDIABAS drops a shorter DoIP reply and then has no VIN.
constexpr size_t kAnnounceLen = 33;

constexpr uint16_t kLaGateway = uds_bdc::kLogicalAddress;
constexpr uint16_t kLaTester  = 0x0E00;

bool g_ethReady = false;
int  g_udpSock  = -1;
int  g_tcpSock  = -1;
int  g_enetSock = -1;
int  g_hsfzUdp  = -1;

// ZGW Search parses this exact 50-byte layout:
// DIAGADR10 + BMWMAC + 12 hex digits + BMWVIN + 17-character VIN.
void buildVehicleIdent(uint8_t out[50]) {
  char vin[18];
  bench_vin::copy(vin);
  memcpy(out, "DIAGADR10BMWMAC", 15);
  const String mac = ETH.macAddress();
  size_t hex = 0;
  for (unsigned i = 0; i < mac.length() && hex < 12; ++i) {
    char c = mac[i];
    if (c == ':' || c == '-') continue;
    if (c >= 'a' && c <= 'f') c = (char)(c - 'a' + 'A');
    out[15 + hex] = (uint8_t)c;
    ++hex;
  }
  while (hex < 12) out[15 + hex++] = '0';
  memcpy(out + 27, "BMWVIN", 6);
  memcpy(out + 33, vin, 17);
}

void writeU16Be(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v >> 8);
  p[1] = (uint8_t)(v & 0xFF);
}

void writeU32Be(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v >> 24);
  p[1] = (uint8_t)(v >> 16);
  p[2] = (uint8_t)(v >> 8);
  p[3] = (uint8_t)(v & 0xFF);
}

uint16_t readU16Be(const uint8_t* p) {
  return (uint16_t)((p[0] << 8) | p[1]);
}

uint32_t readU32Be(const uint8_t* p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
         ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

size_t buildHeader(uint8_t* out, uint16_t payloadType, uint32_t payloadLen) {
  out[0] = kDoipVersion;
  out[1] = kDoipInvVersion;
  writeU16Be(out + 2, payloadType);
  writeU32Be(out + 4, payloadLen);
  return 8;
}

size_t buildVehicleAnnounce(uint8_t* payload) {
  char vin[18];
  bench_vin::copy(vin);
  memcpy(payload, vin, 17);
  writeU16Be(payload + 17, kLaGateway);
  uint8_t mac[6] = {};
  ETH.macAddress(mac);
  memcpy(payload + 19, mac, 6);
  memcpy(payload + 25, mac, 6);
  payload[31] = 0x00;  // further action: none
  payload[32] = 0x00;  // VIN and GID are synchronized
  return kAnnounceLen;
}

struct netif* ethNetif() {
  struct netif* n = netif_find("en0");
  return n != nullptr ? n : netif_default;
}

// Send as if every IPv4 address were on this cable. EDIABAS HostIdentService
// 255.255.255.255 delivers the probe from whatever address the laptop has.
int sendOnCable(int fd, const uint8_t* data, size_t len, const sockaddr_in& to) {
  const uint32_t dest = to.sin_addr.s_addr;
  const bool broadcast = dest == htonl(INADDR_BROADCAST) ||
                         dest == inet_addr("169.254.255.255");
  struct netif* nif = ethNetif();
  ip4_addr_t savedMask = {};
  bool opened = false;
  if (!broadcast && nif != nullptr) {
    savedMask = *netif_ip4_netmask(nif);
    ip4_addr_t openMask = {};
    openMask.addr = 0;
    netif_set_netmask(nif, &openMask);
    opened = true;
  }
  const int sent = sendto(fd, data, len, 0, (const sockaddr*)&to, sizeof(to));
  if (opened) netif_set_netmask(nif, &savedMask);
  return sent;
}

void sendUdp(const uint8_t* data, size_t len, const sockaddr_in& to) {
  if (g_udpSock < 0) return;
  sendOnCable(g_udpSock, data, len, to);
}

// Tools poll this often. Log only when the clamp byte changes.
void logClamp(uint8_t st, const char* via) {
  static int last = -1;
  if (last == (int)st) return;
  last = (int)st;
  notef("[ENET] clamps %02X KL30=%d KL15=%d via %s", st,
        (st & 0x01) ? 1 : 0, (st & 0x04) ? 1 : 0, via);
}

void handleUdpDiscovery() {
  uint8_t buf[256];
  sockaddr_in from = {};
  socklen_t fromLen = sizeof(from);
  const int n = recvfrom(g_udpSock, buf, sizeof(buf), MSG_DONTWAIT,
                         (sockaddr*)&from, &fromLen);
  if (n < 8) return;

  const uint16_t ptype = readU16Be(buf + 2);
  if (ptype == kPtPowerModeReq) {
    uint8_t resp[9];
    buildHeader(resp, kPtPowerModeRes, 1);
    resp[8] = (bmw::terminalStatusByte() & 0x04) ? 0x01 : 0x00;
    sendUdp(resp, sizeof(resp), from);
    logClamp(bmw::terminalStatusByte(), "DoIP");
    return;
  }
  if (ptype != kPtVehicleIdentReq && ptype != kPtVehicleIdentReqEin &&
      ptype != kPtVehicleIdentReqVin) {
    return;
  }

  uint8_t resp[8 + kAnnounceLen];
  buildHeader(resp, kPtVehicleAnnounce, kAnnounceLen);
  buildVehicleAnnounce(resp + 8);
  sendUdp(resp, sizeof(resp), from);
  Serial.println(F("[DoIP] Vehicle Identification Response sent"));
}

void sendDiagnosticResponse(int client, uint16_t sa, uint16_t ta,
                            const uint8_t* uds, size_t udsLen) {
  // DoIP diagnostic message: SA(2)+TA(2)+UDS
  uint8_t packet[8 + 4 + 256];
  if (udsLen > 256) udsLen = 256;
  const uint32_t plen = (uint32_t)(4 + udsLen);
  buildHeader(packet, kPtDiagnosticMessage, plen);
  writeU16Be(packet + 8, sa);   // our LA (BDC) as source
  writeU16Be(packet + 10, ta);  // tester as target
  memcpy(packet + 12, uds, udsLen);
  send(client, packet, 8 + plen, 0);
}

bool recvFull(int fd, uint8_t* dst, size_t n) {
  size_t got = 0;
  while (got < n) {
    const int r = recv(fd, dst + got, n - got, 0);
    if (r == 0) return false;
    if (r < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    got += (size_t)r;
  }
  return true;
}

bool sendFull(int fd, const uint8_t* src, size_t n) {
  size_t sent = 0;
  while (sent < n) {
    const int r = send(fd, src + sent, n - sent, 0);
    if (r <= 0) return false;
    sent += (size_t)r;
  }
  return true;
}

bool sendHsfz(int fd, uint16_t ctrl, const uint8_t* body, size_t bodyLen) {
  uint8_t hdr[6];
  hdr[0] = (uint8_t)((bodyLen >> 24) & 0xFF);
  hdr[1] = (uint8_t)((bodyLen >> 16) & 0xFF);
  hdr[2] = (uint8_t)((bodyLen >> 8) & 0xFF);
  hdr[3] = (uint8_t)(bodyLen & 0xFF);
  hdr[4] = (uint8_t)(ctrl >> 8);
  hdr[5] = (uint8_t)(ctrl & 0xFF);
  if (!sendFull(fd, hdr, 6)) return false;
  if (bodyLen == 0) return true;
  return sendFull(fd, body, bodyLen);
}

// HSFZ control words that are not a diagnostic request.
// 0x0010 is how EDIABAS reads ignition: it opens TCP on the control port
// (6811, the same number as the UDP ident port) and sends 00 00 00 00 00 10.
// The 7-byte reply is 00 00 00 01 00 10 plus one status byte. Bits 3–2 equal
// to 01 (value 0x04) mean KL15 on.
bool serveHsfzControl(int client, uint16_t ctrl, uint32_t len) {
  if (ctrl == 0x0012) {
    const uint8_t alive[2] = {uds_bdc::kCanEcuAddr, 0xF4};
    sendHsfz(client, 0x0012, alive, sizeof(alive));
    return true;
  }
  if (ctrl == 0x0011 && len == 0) {
    uint8_t ident[50];
    buildVehicleIdent(ident);
    sendHsfz(client, 0x0011, ident, sizeof(ident));
    char vin[18];
    bench_vin::copy(vin);
    notef("[ENET] ident VIN %s", vin);
    return true;
  }
  if (ctrl == 0x0010) {
    // Same byte as CAN 0x12F: bit0 KL30, bits3-2 KL15 (0x04 = on).
    const uint8_t st = bmw::terminalStatusByte();
    sendHsfz(client, 0x0010, &st, 1);
    logClamp(st, "HSFZ");
    return true;
  }
  return false;
}

// BMW ENET: TCP 6801, HSFZ. Tester (usually 0xF4) sends control 0x0001.
// Gateway echoes control 0x0002, then answers with control 0x0001 and
// source/target swapped. Address byte is the same one used on CAN 0x6F1.
void handleHsfzClient(int client) {
  timeval tv = {};
  tv.tv_sec = 30;
  setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  const int flags = fcntl(client, F_GETFL, 0);
  if (flags >= 0) fcntl(client, F_SETFL, flags & ~O_NONBLOCK);

  uint8_t body[258];
  pc_link::noteLine("[ENET] HSFZ session on port 6801");
  for (;;) {
    uint8_t hdr[6];
    if (!recvFull(client, hdr, 6)) break;
    const uint32_t len = ((uint32_t)hdr[0] << 24) | ((uint32_t)hdr[1] << 16) |
                         ((uint32_t)hdr[2] << 8) | hdr[3];
    const uint16_t ctrl = (uint16_t)((hdr[4] << 8) | hdr[5]);
    if (len > sizeof(body)) break;
    if (len > 0 && !recvFull(client, body, len)) break;

    if (serveHsfzControl(client, ctrl, len)) continue;
    if (ctrl != 0x0001 || len < 2) {
      Serial.printf("[ENET] ctrl 0x%04X len %u\n", ctrl, (unsigned)len);
      continue;
    }

    const uint8_t src = body[0];
    const uint8_t dst = body[1];
    const uint8_t* uds = body + 2;
    const size_t udsLen = len - 2;
    if (udsLen > 256) {
      sendHsfz(client, 0x0044, nullptr, 0);
      continue;
    }

    sendHsfz(client, 0x0002, body, len);

    uint8_t resp[256];
    size_t respLen = uds_bdc::answerVin(uds, udsLen, resp, sizeof(resp));
    uint8_t respSrc = dst;
    if (respLen > 0) {
      // Functional 0xDF has no address of its own. Answer as the gateway.
      if (dst == 0xDF) respSrc = uds_bdc::kCanEcuAddr;
      char vin[18];
      bench_vin::copy(vin);
      notef("[UDS] VIN %s for 0x%02X", vin, dst);
    } else if (dst == uds_bdc::kCanEcuAddr || (udsLen >= 1 && uds[0] == 0x3E)) {
      respLen = uds_bdc::handleRequest(uds, udsLen, resp, sizeof(resp));
      if (dst == 0xDF) respSrc = uds_bdc::kCanEcuAddr;
    } else {
      uint8_t from = dst;
      respLen = kcan_gw::transact(dst, uds, udsLen, resp, sizeof(resp), &from);
      if (respLen > 0) respSrc = from;
      if (respLen == 0 && udsLen > 0) {
        resp[0] = 0x7F;
        resp[1] = uds[0];
        resp[2] = 0x25;
        respLen = 3;
      }
    }
    if (respLen == 0) continue;

    uint8_t out[2 + 256];
    out[0] = respSrc;
    out[1] = src;
    memcpy(out + 2, resp, respLen);
    sendHsfz(client, 0x0001, out, 2 + respLen);
    Serial.printf("[ENET] 0x%02X -> 0x%02X  %u bytes\n", src, dst, (unsigned)respLen);
  }
  close(client);
  pc_link::noteLine("[ENET] session closed");
}

void handleTcpClient(int client) {
  uint8_t buf[1100];
  for (;;) {
    const int n = recv(client, buf, sizeof(buf), MSG_DONTWAIT);
    if (n == 0) break;
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }
      break;
    }
    if (n < 8) continue;

    const uint16_t ptype = readU16Be(buf + 2);
    const uint32_t plen  = readU32Be(buf + 4);

    if (ptype == kPtPowerModeReq) {
      uint8_t resp[9];
      buildHeader(resp, kPtPowerModeRes, 1);
      resp[8] = (bmw::terminalStatusByte() & 0x04) ? 0x01 : 0x00;
      send(client, resp, sizeof(resp), 0);
      logClamp(bmw::terminalStatusByte(), "DoIP");
      continue;
    }

    if (ptype == kPtRoutingActivationReq) {
      uint8_t resp[8 + 9] = {};
      buildHeader(resp, kPtRoutingActivationRes, 9);
      if (plen >= 2) {
        resp[8] = buf[8];
        resp[9] = buf[9];
      } else {
        writeU16Be(resp + 8, kLaTester);
      }
      writeU16Be(resp + 10, kLaGateway);
      resp[12] = 0x10;
      send(client, resp, sizeof(resp), 0);
      Serial.println(F("[DoIP] Routing activation OK"));
      continue;
    }

    if (ptype == kPtDiagnosticMessage && plen >= 4 && (uint32_t)n >= 8 + plen) {
      const uint16_t sa = readU16Be(buf + 8);   // tester
      const uint16_t ta = readU16Be(buf + 10);  // target ECU

      const uint8_t* uds = buf + 12;
      const size_t udsLen = plen - 4;
      const bool toBdc = (ta == kLaGateway);
      const bool functional = (ta == 0xE400);
      const bool toModule = (ta >= 0x0001 && ta <= 0x00FF && !toBdc);

      if (!toBdc && !toModule && !functional) {
        uint8_t nack[8 + 5] = {};
        buildHeader(nack, kPtDiagnosticMessageNack, 5);
        memcpy(nack + 8, buf + 8, 4);
        nack[12] = 0x03;  // unknown target address
        send(client, nack, sizeof(nack), 0);
        continue;
      }

      uint8_t ack[8 + 5] = {};
      buildHeader(ack, kPtDiagnosticMessageAck, 5);
      memcpy(ack + 8, buf + 8, 4);
      ack[12] = 0x00;
      send(client, ack, sizeof(ack), 0);

      uint8_t resp[256];
      size_t respLen = uds_bdc::answerVin(uds, udsLen, resp, sizeof(resp));
      uint16_t respSa = ta;

      if (respLen > 0) {
        respSa = (toBdc || functional) ? kLaGateway : ta;
        char vin[18];
        bench_vin::copy(vin);
        notef("[UDS] VIN %s for LA 0x%04X", vin, ta);
      } else if (toBdc || (udsLen >= 1 && uds[0] == 0x3E)) {
        respLen = uds_bdc::handleRequest(uds, udsLen, resp, sizeof(resp));
        respSa = kLaGateway;
      } else {
        const uint8_t ecu = functional ? 0xDF : (uint8_t)ta;
        uint8_t fromEcu = ecu;
        respLen = kcan_gw::transact(ecu, uds, udsLen, resp, sizeof(resp), &fromEcu);
        respSa = functional ? fromEcu : ta;
        if (respLen == 0 && udsLen > 0) {
          // ISO 14229 NRC 0x25 — gateway did not get an answer from the module.
          resp[0] = 0x7F;
          resp[1] = uds[0];
          resp[2] = 0x25;
          respLen = 3;
          respSa = ta;
          Serial.printf("[DoIP] LA 0x%04X no answer on K-CAN\n", ta);
        }
      }

      if (respLen > 0) {
        sendDiagnosticResponse(client, respSa, sa, resp, respLen);
      }
    }
  }
  close(client);
  Serial.println(F("[DoIP] TCP client disconnected"));
}

bool applyStaticIp() {
  const bool ok = ETH.config(ETH_LOCAL_IP, ETH_GATEWAY, ETH_SUBNET);
  if (ETH.linkUp() && ETH.localIP() == ETH_LOCAL_IP) g_ethReady = true;
  return ok;
}

void logEthAddress(const char* why) {
  char msg[140];
  snprintf(msg, sizeof(msg), "[ETH] %s IP %s mask %s link %s", why,
           ETH.localIP().toString().c_str(),
           ETH.subnetMask().toString().c_str(),
           ETH.linkUp() ? "up" : "down");
  pc_link::noteLine(msg);
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onEthEvent(arduino_event_id_t event, arduino_event_info_t info) {
  (void)info;
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      ETH.setHostname("bmw-bdc-emu");
      applyStaticIp();
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      pc_link::noteLine("[ETH] Link up");
      applyStaticIp();
      logEthAddress("link");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      logEthAddress("got ip");
      applyStaticIp();
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      g_ethReady = false;
      pc_link::noteLine("[ETH] Link down");
      break;
    default:
      break;
  }
}
#else
void onEthEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      ETH.setHostname("bmw-bdc-emu");
      applyStaticIp();
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      pc_link::noteLine("[ETH] Link up");
      applyStaticIp();
      logEthAddress("link");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      logEthAddress("got ip");
      applyStaticIp();
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      g_ethReady = false;
      pc_link::noteLine("[ETH] Link down");
      break;
    default:
      break;
  }
}
#endif

void hsfzUdpTask(void* arg);
void hsfzControlTask(void* arg);

bool beginEthernet(int phyAddr, int powerPin, eth_clock_mode_t clock) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ETH.begin(ETH_PHY_TYPE, phyAddr, ETH_PHY_MDC, ETH_PHY_MDIO, powerPin, clock);
#else
  return ETH.begin((uint8_t)phyAddr, powerPin, ETH_PHY_MDC, ETH_PHY_MDIO,
                    ETH_PHY_TYPE, clock);
#endif
}

// WT32-ETH01: GPIO16 enables the 50 MHz oscillator, GPIO0 receives that clock.
// GPIO0 is a strapping pin and boots with its pull-up still on. That load
// stops the oscillator, MDIO reads 0xFFFF and lan87xx reports power up timeout.
void releaseRmiiClockPin() {
  gpio_reset_pin(GPIO_NUM_0);
  gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
  gpio_pullup_dis(GPIO_NUM_0);
  gpio_pulldown_dis(GPIO_NUM_0);
  gpio_set_pull_mode(GPIO_NUM_0, GPIO_FLOATING);
}

void enableOscillator() {
  gpio_reset_pin(GPIO_NUM_16);
  gpio_set_direction(GPIO_NUM_16, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_16, 1);
  delay(300);
}

bool startEthernet() {
  struct Attempt {
    int addr;
    int power;
    eth_clock_mode_t clock;
    const char* name;
  };
  // Address 1 is the Wireless-Tag WT32-ETH01. Plenty of clones answer at 0.
  // GPIO17-out is the boards that have no oscillator of their own.
  // A failed ETH.begin() keeps the EMAC interrupt and the "ETH_DEF" netif,
  // so the next begin() in the same boot cannot run. One attempt per boot.
  const Attempt attempts[] = {
      {1, -1, ETH_CLOCK_GPIO0_IN, "PHY 1, clock in GPIO0"},
      {0, -1, ETH_CLOCK_GPIO0_IN, "PHY 0, clock in GPIO0"},
      {1, 16, ETH_CLOCK_GPIO0_IN, "PHY 1, enable GPIO16, clock in GPIO0"},
      {0, 16, ETH_CLOCK_GPIO0_IN, "PHY 0, enable GPIO16, clock in GPIO0"},
      {1, -1, ETH_CLOCK_GPIO17_OUT, "PHY 1, clock out GPIO17"},
      {0, -1, ETH_CLOCK_GPIO17_OUT, "PHY 0, clock out GPIO17"},
  };
  const int nAttempts = (int)(sizeof(attempts) / sizeof(attempts[0]));

  if (g_ethScanDone) return false;

  Preferences prefs;
  prefs.begin("bdceth", false);
  int idx = prefs.getInt("try", 0);
  if (idx < 0 || idx >= nAttempts) idx = 0;

  const Attempt& attempt = attempts[idx];
  releaseRmiiClockPin();
  enableOscillator();
  Serial.printf("[ETH] try %d/%d %s\n", idx + 1, nAttempts, attempt.name);
  if (beginEthernet(attempt.addr, attempt.power, attempt.clock)) {
    prefs.putInt("try", idx);
    prefs.end();
    notef("[ETH] PHY up: %s", attempt.name);
    return true;
  }
  Serial.printf("[ETH] no PHY: %s\n", attempt.name);
  const int next = idx + 1;
  if (next < nAttempts) {
    prefs.putInt("try", next);
    prefs.end();
    Serial.printf("[ETH] reboot to try %d/%d\n", next + 1, nAttempts);
    Serial.flush();
    delay(200);
    esp_restart();
  }
  prefs.putInt("try", 0);
  prefs.end();
  g_ethScanDone = 1;
  return false;
}

bool init() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  Network.onEvent(onEthEvent);
#else
  WiFi.onEvent(onEthEvent);
#endif
  const bool ok = startEthernet();

  if (!ok) {
    Serial.println(F("[ETH] LAN8720 did not answer. Disconnect the USB adapter from IO0"));
    Serial.println(F("[ETH] (a pull-up on IO0 stops the 50 MHz clock) and power-cycle the board."));
  }

  if (!applyStaticIp()) {
    Serial.println(F("[ETH] Static IP config failed"));
  }
  logEthAddress("init");
  // Own task: ZGW Search waits only 100 ms for the UDP 6811 reply.
  // The diagnostic TCP session must not delay that answer.
  xTaskCreatePinnedToCore(hsfzUdpTask, "zgw_udp", 8192, nullptr, TASK_PRIO_CAN_RX,
                          nullptr, TASK_CORE_NET);
  xTaskCreatePinnedToCore(hsfzControlTask, "zgw_kl15", 8192, nullptr, TASK_PRIO_DOIP,
                          nullptr, TASK_CORE_NET);
  return true;
}

// EDIABAS keeps this TCP session open and repeats the ignition request on it.
// It must not run inside the diagnostic task, or port 6801 stops accepting.
void handleHsfzControlClient(int client) {
  timeval tv = {};
  tv.tv_sec = 120;
  setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  const int flags = fcntl(client, F_GETFL, 0);
  if (flags >= 0) fcntl(client, F_SETFL, flags & ~O_NONBLOCK);

  uint8_t body[64];
  for (;;) {
    uint8_t hdr[6];
    if (!recvFull(client, hdr, 6)) break;
    const uint32_t len = ((uint32_t)hdr[0] << 24) | ((uint32_t)hdr[1] << 16) |
                         ((uint32_t)hdr[2] << 8) | hdr[3];
    const uint16_t ctrl = (uint16_t)((hdr[4] << 8) | hdr[5]);
    if (len > sizeof(body)) break;
    if (len > 0 && !recvFull(client, body, len)) break;
    if (!serveHsfzControl(client, ctrl, len)) {
      Serial.printf("[ENET] control 0x%04X len %u\n", ctrl, (unsigned)len);
    }
  }
  close(client);
  pc_link::noteLine("[ENET] control session closed");
}

void hsfzControlTask(void* /*arg*/) {
  for (;;) {
    if (!ETH.linkUp()) {
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }
    const int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(ENET_HSFZ_UDP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(fd, (sockaddr*)&addr, sizeof(addr)) != 0 || listen(fd, 2) != 0) {
      notef("[ENET] TCP %u bind failed", (unsigned)ENET_HSFZ_UDP_PORT);
      close(fd);
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    notef("[ENET] KL15 listening TCP :%u", (unsigned)ENET_HSFZ_UDP_PORT);
    for (;;) {
      if (!ETH.linkUp()) break;
      fd_set rfds;
      FD_ZERO(&rfds);
      FD_SET(fd, &rfds);
      timeval wait = {};
      wait.tv_sec = 1;
      if (select(fd + 1, &rfds, nullptr, nullptr, &wait) <= 0) continue;
      sockaddr_in ca = {};
      socklen_t cal = sizeof(ca);
      const int client = accept(fd, (sockaddr*)&ca, &cal);
      if (client < 0) continue;
      notef("[ENET] control %s", inet_ntoa(ca.sin_addr));
      handleHsfzControlClient(client);
    }
    close(fd);
  }
}

void sendVehicleIdent(const sockaddr_in& to) {
  if (g_hsfzUdp < 0) return;
  uint8_t ident[50];
  buildVehicleIdent(ident);
  uint8_t pkt[56];
  pkt[0] = 0;
  pkt[1] = 0;
  pkt[2] = 0;
  pkt[3] = 50;
  pkt[4] = 0x00;
  pkt[5] = 0x11;
  memcpy(pkt + 6, ident, 50);
  if (sendOnCable(g_hsfzUdp, pkt, sizeof(pkt), to) < 0) {
    Serial.printf("[ENET] ident send failed errno %d\n", errno);
  }
}

void announceVehicle() {
  if (g_hsfzUdp < 0) return;
  sockaddr_in to = {};
  to.sin_family = AF_INET;
  to.sin_port = htons(ENET_HSFZ_UDP_PORT);
  to.sin_addr.s_addr = htonl(INADDR_BROADCAST);
  sendVehicleIdent(to);
  to.sin_addr.s_addr = inet_addr("169.254.255.255");
  sendVehicleIdent(to);
}

// Probe used by ZGW_SEARCH and by Viaszx/BMW_ZGW_Search: 00 00 00 00 00 11
// to the adapter broadcast, UDP 6811. The tool reads one reply and gives up
// after 100 ms. The text must contain DIAGADR, BMWMAC and BMWVIN in that order.
void hsfzUdpTask(void* /*arg*/) {
  Serial.println(F("[ENET] ZGW search task started"));
  uint32_t lastAnnounce = 0;
  for (;;) {
    if (!ETH.linkUp()) {
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }
    if (ETH.localIP() != ETH_LOCAL_IP || ETH.subnetMask() != ETH_SUBNET) {
      applyStaticIp();
      logEthAddress("udp");
    }
    if (g_hsfzUdp < 0) {
      g_hsfzUdp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
      int reuse = 1;
      setsockopt(g_hsfzUdp, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
      int bcast = 1;
      setsockopt(g_hsfzUdp, SOL_SOCKET, SO_BROADCAST, &bcast, sizeof(bcast));
      timeval tv = {};
      tv.tv_sec = 0;
      tv.tv_usec = 200000;
      setsockopt(g_hsfzUdp, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
      sockaddr_in ha = {};
      ha.sin_family = AF_INET;
      ha.sin_port = htons(ENET_HSFZ_UDP_PORT);
      ha.sin_addr.s_addr = htonl(INADDR_ANY);
      if (bind(g_hsfzUdp, (sockaddr*)&ha, sizeof(ha)) != 0) {
        Serial.println(F("[ENET] UDP 6811 bind failed"));
        close(g_hsfzUdp);
        g_hsfzUdp = -1;
        vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }
      char vin[18];
      bench_vin::copy(vin);
      notef("[ENET] ZGW search listening UDP :%u VIN %s",
            ENET_HSFZ_UDP_PORT, vin);
    }

    uint8_t buf[128];
    sockaddr_in from = {};
    socklen_t fl = sizeof(from);
    const int n = recvfrom(g_hsfzUdp, buf, sizeof(buf), 0, (sockaddr*)&from, &fl);
    if (n >= 6) {
      const uint32_t own = (uint32_t)ETH.localIP();
      const uint32_t len = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                           ((uint32_t)buf[2] << 8) | buf[3];
      const uint16_t ctrl = (uint16_t)((buf[4] << 8) | buf[5]);
      const bool request =
          from.sin_addr.s_addr != own &&
          ((ctrl == 0x0011 && len == 0) || ctrl == 0x0012 || n <= 8);
      if (request) {
        sendVehicleIdent(from);
        char vin[18];
        bench_vin::copy(vin);
        notef("[ENET] ZGW search from %s:%u -> VIN %s",
              inet_ntoa(from.sin_addr), (unsigned)ntohs(from.sin_port),
              vin);
      }
    }
    if (millis() - lastAnnounce > 2000) {
      lastAnnounce = millis();
      announceVehicle();
    }
  }
}

void serverTask(void* /*arg*/) {
  Serial.println(F("[DoIP] Server task waiting for Ethernet..."));
  for (int i = 0; i < 30 && !g_ethReady; ++i) {
    if (ETH.linkUp()) applyStaticIp();
    if (!g_ethReady) vTaskDelay(pdMS_TO_TICKS(200));
  }
  if (!g_ethReady) {
    logEthAddress("still down");
    pc_link::noteLine("[ETH] No link. Check the cable and that GPIO16 stays the PHY enable.");
    while (!ETH.linkUp()) {
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    applyStaticIp();
    logEthAddress("late link");
  }

  g_udpSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  sockaddr_in ua = {};
  ua.sin_family      = AF_INET;
  ua.sin_port        = htons(DOIP_UDP_DISCOVERY_PORT);
  ua.sin_addr.s_addr = htonl(INADDR_ANY);
  int reuse = 1;
  setsockopt(g_udpSock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  bind(g_udpSock, (sockaddr*)&ua, sizeof(ua));

  g_tcpSock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  setsockopt(g_tcpSock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  sockaddr_in ta = {};
  ta.sin_family      = AF_INET;
  ta.sin_port        = htons(DOIP_TCP_DATA_PORT);
  ta.sin_addr.s_addr = htonl(INADDR_ANY);
  bind(g_tcpSock, (sockaddr*)&ta, sizeof(ta));
  listen(g_tcpSock, 2);
  fcntl(g_tcpSock, F_SETFL, O_NONBLOCK);

  g_enetSock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  setsockopt(g_enetSock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  sockaddr_in ea = {};
  ea.sin_family      = AF_INET;
  ea.sin_port        = htons(ENET_HSFZ_TCP_PORT);
  ea.sin_addr.s_addr = htonl(INADDR_ANY);
  bind(g_enetSock, (sockaddr*)&ea, sizeof(ea));
  listen(g_enetSock, 2);
  fcntl(g_enetSock, F_SETFL, O_NONBLOCK);

  notef("[DoIP] BDC LA=0x%04X listening UDP/TCP :%u (IP %s)",
        kLaGateway, DOIP_TCP_DATA_PORT,
        ETH.localIP().toString().c_str());
  notef("[ENET] HSFZ listening TCP :%u", ENET_HSFZ_TCP_PORT);

  for (;;) {
    handleUdpDiscovery();

    sockaddr_in ca = {};
    socklen_t cal = sizeof(ca);
    const int client = accept(g_tcpSock, (sockaddr*)&ca, &cal);
    if (client >= 0) {
      notef("[DoIP] TCP client %s", inet_ntoa(ca.sin_addr));
      fcntl(client, F_SETFL, O_NONBLOCK);
      handleTcpClient(client);
    }

    cal = sizeof(ca);
    const int enet = accept(g_enetSock, (sockaddr*)&ca, &cal);
    if (enet >= 0) {
      notef("[ENET] TCP client %s", inet_ntoa(ca.sin_addr));
      handleHsfzClient(enet);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

}  // namespace doip
