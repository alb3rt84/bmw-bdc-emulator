/**
 * @file isotp.cpp
 * @brief ISO-TP SF/FF/CF (+ FC CTS) for ESP32 CAN diagnostic path.
 */

#include "isotp.h"

#include <Arduino.h>
#include <string.h>

namespace isotp {

namespace {

uint8_t pciOffset(const Config& cfg) {
  return cfg.bmwAddressed ? 1 : 0;
}

uint8_t dataCapacity(const Config& cfg) {
  // Classical CAN DLC 8
  return (uint8_t)(8 - pciOffset(cfg));
}

void fillAddr(const Config& cfg, uint8_t* data, bool tx) {
  if (!cfg.bmwAddressed) return;
  data[0] = tx ? cfg.testerAddr : cfg.ecuAddr;
}

bool sendFrame(const Config& cfg, const uint8_t data[8], uint8_t dlc) {
  CanFrame f = {};
  f.id       = cfg.txId;
  f.dlc      = dlc;
  f.extended = false;
  memcpy(f.data, data, dlc);
  return canBusSend(cfg.channel, f);
}

}  // namespace

void initLink(Link& link, const Config& cfg) {
  link.cfg          = cfg;
  link.rxLen        = 0;
  link.rxExpected   = 0;
  link.rxSeq        = 0;
  link.rxInProgress = false;
  link.rxLastMs     = 0;
}

bool onCanFrame(Link& link, const CanFrame& frame) {
  if (frame.id != link.cfg.rxId || frame.extended) return false;
  if (frame.dlc < 1) return false;

  const uint8_t off = pciOffset(link.cfg);
  if (link.cfg.bmwAddressed) {
    if (frame.dlc < 2) return false;
    if (frame.data[0] != link.cfg.ecuAddr) return false;
  }

  if (frame.dlc <= off) return false;
  const uint8_t pci = frame.data[off];
  const uint8_t ft  = (uint8_t)(pci >> 4);

  // Timeout incomplete multi-frame
  if (link.rxInProgress && (millis() - link.rxLastMs) > 1000) {
    link.rxInProgress = false;
    link.rxLen        = 0;
  }

  if (ft == 0x0) {
    // Single Frame: PCI = 0x0N (N = length)
    const uint8_t n = (uint8_t)(pci & 0x0F);
    if (n == 0 || n > (frame.dlc - off - 1)) return false;
    memcpy(link.rxBuf, &frame.data[off + 1], n);
    link.rxLen        = n;
    link.rxInProgress = false;
    return true;
  }

  if (ft == 0x1) {
    // First Frame: length in low nibble + next byte
    if (frame.dlc < off + 2) return false;
    const size_t total =
        ((size_t)(pci & 0x0F) << 8) | (size_t)frame.data[off + 1];
    if (total == 0 || total > kMaxPayload) return false;

    const uint8_t copy = (uint8_t)(frame.dlc - off - 2);
    memcpy(link.rxBuf, &frame.data[off + 2], copy);
    link.rxLen        = copy;
    link.rxExpected   = total;
    link.rxSeq        = 1;
    link.rxInProgress = true;
    link.rxLastMs     = millis();

    // Flow Control CTS
    uint8_t fc[8] = {};
    fillAddr(link.cfg, fc, true);
    fc[off]     = 0x30;  // CTS
    fc[off + 1] = 0x00;  // block size 0 = send all
    fc[off + 2] = 0x00;  // STmin 0
    sendFrame(link.cfg, fc, 8);
    return false;
  }

  if (ft == 0x2 && link.rxInProgress) {
    const uint8_t seq = (uint8_t)(pci & 0x0F);
    if (seq != (link.rxSeq & 0x0F)) {
      link.rxInProgress = false;
      return false;
    }
    link.rxSeq++;
    const uint8_t copy = (uint8_t)(frame.dlc - off - 1);
    if (link.rxLen + copy > kMaxPayload) {
      link.rxInProgress = false;
      return false;
    }
    memcpy(link.rxBuf + link.rxLen, &frame.data[off + 1], copy);
    link.rxLen += copy;
    link.rxLastMs = millis();
    if (link.rxLen >= link.rxExpected) {
      link.rxLen        = link.rxExpected;
      link.rxInProgress = false;
      return true;
    }
    return false;
  }

  // Flow control from peer ignored on RX path (we are server)
  return false;
}

bool send(Link& link, const uint8_t* data, size_t len) {
  if (!data || len == 0 || len > kMaxPayload) return false;

  const uint8_t off = pciOffset(link.cfg);
  const uint8_t cap = dataCapacity(link.cfg);

  if (len <= (size_t)(cap - 1)) {
    uint8_t frame[8] = {};
    fillAddr(link.cfg, frame, true);
    frame[off] = (uint8_t)(0x00 | (len & 0x0F));
    memcpy(&frame[off + 1], data, len);
    return sendFrame(link.cfg, frame, 8);
  }

  // First Frame
  uint8_t ff[8] = {};
  fillAddr(link.cfg, ff, true);
  ff[off]     = (uint8_t)(0x10 | ((len >> 8) & 0x0F));
  ff[off + 1] = (uint8_t)(len & 0xFF);
  const uint8_t ffData = (uint8_t)(cap - 2);
  memcpy(&ff[off + 2], data, ffData);
  if (!sendFrame(link.cfg, ff, 8)) return false;

  // Wait briefly for FC (optional — many tools tolerate immediate CF)
  vTaskDelay(pdMS_TO_TICKS(2));

  size_t sent = ffData;
  uint8_t seq = 1;
  while (sent < len) {
    uint8_t cf[8] = {};
    fillAddr(link.cfg, cf, true);
    cf[off] = (uint8_t)(0x20 | (seq & 0x0F));
    seq     = (uint8_t)((seq + 1) & 0x0F);
    const size_t chunk = (len - sent) > (size_t)(cap - 1) ? (size_t)(cap - 1)
                                                          : (len - sent);
    memcpy(&cf[off + 1], data + sent, chunk);
    if (!sendFrame(link.cfg, cf, 8)) return false;
    sent += chunk;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return true;
}

}  // namespace isotp
