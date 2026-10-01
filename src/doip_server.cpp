/**
 * @file doip_server.cpp
 * @brief DoIP UDP discovery + TCP sessions → shared uds_bdc handler.
 */

#include "doip_server.h"
#include "config.h"
#include "kcan_gw.h"
#include "uds_bdc.h"

#include <ETH.h>
#include <WiFi.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <lwip/sockets.h>

namespace doip {

namespace {

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

constexpr uint16_t kLaGateway = uds_bdc::kLogicalAddress;
constexpr uint16_t kLaTester  = 0x0E00;

bool g_ethReady = false;
int  g_udpSock  = -1;
int  g_tcpSock  = -1;
int  g_enetSock = -1;

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
  const char vin[17] = {'W','B','A','D','E','M','O','G','C','H','A','S','S','I','S','0','1'};
  memcpy(payload + 0, vin, 17);
  writeU16Be(payload + 17, kLaGateway);
  memset(payload + 19, 0x00, 12);
  payload[31] = 0x00;
  return 32;
}

void sendUdp(const uint8_t* data, size_t len, const sockaddr_in& to) {
  if (g_udpSock < 0) return;
  sendto(g_udpSock, data, len, 0, (const sockaddr*)&to, sizeof(to));
}

void handleUdpDiscovery() {
  uint8_t buf[256];
  sockaddr_in from = {};
  socklen_t fromLen = sizeof(from);
  const int n = recvfrom(g_udpSock, buf, sizeof(buf), MSG_DONTWAIT,
                         (sockaddr*)&from, &fromLen);
  if (n < 8) return;

  const uint16_t ptype = readU16Be(buf + 2);
  if (ptype != kPtVehicleIdentReq && ptype != kPtVehicleIdentReqEin &&
      ptype != kPtVehicleIdentReqVin) {
    return;
  }

  uint8_t resp[8 + 32];
  buildHeader(resp, kPtVehicleAnnounce, 32);
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
  Serial.println(F("[ENET] HSFZ session on port 6801"));
  for (;;) {
    uint8_t hdr[6];
    if (!recvFull(client, hdr, 6)) break;
    const uint32_t len = ((uint32_t)hdr[0] << 24) | ((uint32_t)hdr[1] << 16) |
                         ((uint32_t)hdr[2] << 8) | hdr[3];
    const uint16_t ctrl = (uint16_t)((hdr[4] << 8) | hdr[5]);
    if (len > sizeof(body)) break;
    if (len > 0 && !recvFull(client, body, len)) break;

    if (ctrl == 0x0012) {
      const uint8_t alive[2] = {uds_bdc::kCanEcuAddr, 0xF4};
      sendHsfz(client, 0x0012, alive, sizeof(alive));
      continue;
    }
    if (ctrl == 0x0011 && len == 0) {
      static const char vin[] = "WBADEMOGCHASSIS01";
      sendHsfz(client, 0x0011, reinterpret_cast<const uint8_t*>(vin), 17);
      continue;
    }
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
    size_t respLen = 0;
    uint8_t respSrc = dst;
    if (dst == uds_bdc::kCanEcuAddr) {
      respLen = uds_bdc::handleRequest(uds, udsLen, resp, sizeof(resp));
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
  Serial.println(F("[ENET] session closed"));
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
      size_t respLen = 0;
      uint16_t respSa = ta;

      if (toBdc) {
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

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onEthEvent(arduino_event_id_t event, arduino_event_info_t info) {
  (void)info;
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      ETH.setHostname("bmw-bdc-emu");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println(F("[ETH] Link up"));
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print(F("[ETH] IP: "));
      Serial.println(ETH.localIP());
      g_ethReady = true;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      g_ethReady = false;
      Serial.println(F("[ETH] Link down"));
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
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println(F("[ETH] Link up"));
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print(F("[ETH] IP: "));
      Serial.println(ETH.localIP());
      g_ethReady = true;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      g_ethReady = false;
      Serial.println(F("[ETH] Link down"));
      break;
    default:
      break;
  }
}
#endif

}  // namespace

bool init() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  Network.onEvent(onEthEvent);
  const bool ok = ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO,
                            ETH_PHY_POWER, ETH_CLK_MODE);
#else
  WiFi.onEvent(onEthEvent);
  const bool ok = ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER, ETH_PHY_MDC, ETH_PHY_MDIO,
                            ETH_PHY_TYPE, ETH_CLK_MODE);
#endif

  if (!ok) {
    Serial.println(F("[ETH] begin() failed — check LAN8720A wiring / 50 MHz clock"));
  }

  if (!ETH.config(ETH_LOCAL_IP, ETH_GATEWAY, ETH_SUBNET)) {
    Serial.println(F("[ETH] Static IP config failed (DHCP may still work)"));
  }
  return true;
}

void serverTask(void* /*arg*/) {
  Serial.println(F("[DoIP] Server task waiting for Ethernet..."));
  while (!g_ethReady) {
    vTaskDelay(pdMS_TO_TICKS(200));
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

  Serial.printf("[DoIP] BDC LA=0x%04X listening UDP/TCP :%u (IP %s)\n",
                kLaGateway, DOIP_TCP_DATA_PORT,
                ETH.localIP().toString().c_str());
  Serial.printf("[ENET] HSFZ listening TCP :%u\n", ENET_HSFZ_TCP_PORT);

  for (;;) {
    handleUdpDiscovery();

    sockaddr_in ca = {};
    socklen_t cal = sizeof(ca);
    const int client = accept(g_tcpSock, (sockaddr*)&ca, &cal);
    if (client >= 0) {
      Serial.printf("[DoIP] TCP client %s\n", inet_ntoa(ca.sin_addr));
      fcntl(client, F_SETFL, O_NONBLOCK);
      handleTcpClient(client);
    }

    cal = sizeof(ca);
    const int enet = accept(g_enetSock, (sockaddr*)&ca, &cal);
    if (enet >= 0) {
      Serial.printf("[ENET] TCP client %s\n", inet_ntoa(ca.sin_addr));
      handleHsfzClient(enet);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

}  // namespace doip
