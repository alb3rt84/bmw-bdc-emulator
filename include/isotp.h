/**
 * @file isotp.h
 * @brief Minimal ISO-TP (ISO 15765-2) unicast helper for UDS over CAN.
 *
 * Supports Single-Frame and Multi-Frame (FF/CF) RX up to kMaxPayload,
 * and SF + FF/CF TX. Flow-Control is answered with CTS (0x30).
 */

#pragma once

#include "can_bus.h"

#include <stddef.h>
#include <stdint.h>

namespace isotp {

constexpr size_t kMaxPayload = 256;

struct Config {
  uint32_t   rxId;           // CAN ID we listen on (request)
  uint32_t   txId;           // CAN ID we reply on (response)
  CanChannel channel;
  bool       bmwAddressed;   // true: BMW style — data[0]=ecuAddr on RX, 0xF1 on TX
  uint8_t    ecuAddr;        // e.g. 0x10 for BDC when bmwAddressed
  uint8_t    testerAddr;     // usually 0xF1
};

struct Link {
  Config  cfg;
  uint8_t rxBuf[kMaxPayload];
  size_t  rxLen;
  size_t  rxExpected;
  uint8_t rxSeq;
  bool    rxInProgress;
  uint32_t rxLastMs;
};

void initLink(Link& link, const Config& cfg);

/**
 * Feed one CAN frame. Returns true when a complete UDS payload is ready in
 * link.rxBuf / link.rxLen (caller must copy before next onCanFrame).
 */
bool onCanFrame(Link& link, const CanFrame& frame);

/** Send a complete UDS payload (handles SF / FF+CF). */
bool send(Link& link, const uint8_t* data, size_t len);

}  // namespace isotp
