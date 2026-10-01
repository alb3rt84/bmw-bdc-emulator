/**
 * @file kcan_gw.cpp
 * @brief Tester-side ISO-TP on the module K-CAN, driven from the DoIP task.
 */

#include "kcan_gw.h"

#include "config.h"

#include <Arduino.h>
#include <string.h>

namespace kcan_gw {

namespace {

constexpr uint32_t kReqId = 0x6F1;
constexpr uint8_t kTester = 0xF1;
constexpr size_t kMaxPayload = 256;
constexpr CanChannel kBus = CanChannel::Can1_Twai;

QueueHandle_t g_q = nullptr;
SemaphoreHandle_t g_lock = nullptr;
volatile bool g_active = false;
volatile bool g_functional = false;
volatile uint32_t g_rxId = 0;

bool sendRaw(uint32_t id, const uint8_t* data, uint8_t dlc) {
  CanFrame f = {};
  f.id = id;
  f.dlc = dlc;
  f.extended = false;
  memcpy(f.data, data, dlc);
  return canBusSend(kBus, f);
}

bool pop(CanFrame& out, uint32_t timeoutMs) {
  if (!g_q) return false;
  return xQueueReceive(g_q, &out, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

bool sendRequest(uint8_t ecu, const uint8_t* uds, size_t len, uint32_t deadline) {
  if (len == 0 || len > kMaxPayload) return false;

  if (len <= 6) {
    uint8_t frame[8] = {};
    frame[0] = ecu;
    frame[1] = (uint8_t)len;
    memcpy(&frame[2], uds, len);
    return sendRaw(kReqId, frame, 8);
  }

  uint8_t ff[8] = {};
  ff[0] = ecu;
  ff[1] = (uint8_t)(0x10 | ((len >> 8) & 0x0F));
  ff[2] = (uint8_t)(len & 0xFF);
  memcpy(&ff[3], uds, 5);
  if (!sendRaw(kReqId, ff, 8)) return false;

  uint8_t stMin = 0;
  CanFrame fc;
  while (millis() < deadline) {
    if (!pop(fc, 20)) continue;
    if (!g_functional && fc.id != g_rxId) continue;
    if (fc.dlc < 2 || fc.data[0] != kTester) continue;
    if ((fc.data[1] >> 4) != 0x3) continue;
    if (fc.dlc >= 4 && fc.data[3] <= 0x7F) stMin = fc.data[3];
    break;
  }
  if (millis() >= deadline) return false;

  size_t sent = 5;
  uint8_t seq = 1;
  while (sent < len) {
    if (millis() >= deadline) return false;
    uint8_t cf[8] = {};
    cf[0] = ecu;
    cf[1] = (uint8_t)(0x20 | (seq & 0x0F));
    seq = (uint8_t)((seq + 1) & 0x0F);
    const size_t chunk = (len - sent) > 6 ? 6 : (len - sent);
    memcpy(&cf[2], uds + sent, chunk);
    if (!sendRaw(kReqId, cf, 8)) return false;
    sent += chunk;
    vTaskDelay(pdMS_TO_TICKS(stMin == 0 ? 1 : stMin));
  }
  return true;
}

bool takeResponse(uint8_t* resp, size_t respMax, size_t& outLen, uint8_t& fromEcu,
                  uint32_t deadline) {
  size_t expected = 0;
  size_t filled = 0;
  uint8_t nextSeq = 1;
  bool multi = false;

  while (millis() < deadline) {
    CanFrame f;
    const uint32_t left = deadline - millis();
    if (!pop(f, left > 50 ? 50 : left)) continue;
    if (f.dlc < 2 || f.data[0] != kTester) continue;
    if (g_functional) {
      if (f.id < 0x600 || f.id > 0x67F) continue;
    } else if (f.id != g_rxId) {
      continue;
    }

    const uint8_t pci = f.data[1];
    const uint8_t ft = (uint8_t)(pci >> 4);

    if (!multi && ft == 0x0) {
      const uint8_t n = (uint8_t)(pci & 0x0F);
      if (n == 0 || f.dlc < (uint8_t)(2 + n) || n > respMax) return false;
      memcpy(resp, &f.data[2], n);
      outLen = n;
      fromEcu = (uint8_t)(f.id & 0xFF);
      g_functional = false;
      g_rxId = f.id;
      return true;
    }

    if (!multi && ft == 0x1) {
      if (f.dlc < 4) return false;
      expected = ((size_t)(pci & 0x0F) << 8) | f.data[2];
      if (expected == 0 || expected > respMax || expected > kMaxPayload) return false;
      const uint8_t copy = (uint8_t)(f.dlc - 3);
      memcpy(resp, &f.data[3], copy);
      filled = copy;
      multi = true;
      nextSeq = 1;
      fromEcu = (uint8_t)(f.id & 0xFF);
      g_functional = false;
      g_rxId = f.id;

      uint8_t cts[8] = {};
      cts[0] = fromEcu;
      cts[1] = 0x30;
      if (!sendRaw(kReqId, cts, 8)) return false;
      continue;
    }

    if (multi && ft == 0x2) {
      if ((pci & 0x0F) != (nextSeq & 0x0F)) return false;
      nextSeq = (uint8_t)((nextSeq + 1) & 0x0F);
      uint8_t copy = (uint8_t)(f.dlc - 2);
      if (filled + copy > expected) copy = (uint8_t)(expected - filled);
      if (filled + copy > respMax) return false;
      memcpy(resp + filled, &f.data[2], copy);
      filled += copy;
      if (filled >= expected) {
        outLen = expected;
        return true;
      }
    }
  }
  return false;
}

}  // namespace

void init() {
  if (!g_q) g_q = xQueueCreate(48, sizeof(CanFrame));
  if (!g_lock) g_lock = xSemaphoreCreateMutex();
  Serial.printf("[KCAN] gateway on TWAI @ %d kbit/s  req=0x6F1\n", TWAI_BITRATE_KBPS);
}

bool onCanFrame(const CanFrame& frame) {
  if (!g_active || !g_q || frame.extended) return false;
  if (g_functional) {
    if (frame.id < 0x600 || frame.id > 0x67F) return false;
  } else if (frame.id != g_rxId) {
    return false;
  }
  xQueueSend(g_q, &frame, 0);
  return true;
}

size_t transact(uint8_t ecuAddr, const uint8_t* req, size_t reqLen,
                uint8_t* resp, size_t respMax, uint8_t* responder) {
  if (!g_lock || !g_q || !req || !resp || reqLen == 0) return 0;
  if (xSemaphoreTake(g_lock, pdMS_TO_TICKS(KCAN_GW_TIMEOUT_MS)) != pdTRUE) return 0;

  xQueueReset(g_q);
  g_functional = (ecuAddr == 0xDF);
  g_rxId = 0x600u | ecuAddr;
  g_active = true;

  const uint32_t deadline = millis() + KCAN_GW_TIMEOUT_MS;
  size_t n = 0;
  uint8_t from = ecuAddr;
  const bool sent = sendRequest(ecuAddr, req, reqLen, deadline);
  if (sent) {
    size_t got = 0;
    if (takeResponse(resp, respMax, got, from, deadline)) n = got;
  }
  g_active = false;
  g_functional = false;

  xSemaphoreGive(g_lock);
  if (n == 0) {
    Serial.printf("[KCAN] ecu 0x%02X no response\n", ecuAddr);
    return 0;
  }
  if (responder) *responder = from;
  Serial.printf("[KCAN] ecu 0x%02X response %u bytes from 0x%02X\n",
                ecuAddr, (unsigned)n, from);
  return n;
}

}  // namespace kcan_gw
