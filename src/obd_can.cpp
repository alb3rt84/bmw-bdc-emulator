/**
 * @file obd_can.cpp
 * @brief CAN OBD path into shared uds_bdc (BMW 0x6F1 addressed ISO-TP).
 */

#include "obd_can.h"
#include "config.h"
#include "isotp.h"
#include "uds_bdc.h"

#include <Arduino.h>
#include <string.h>

namespace obd_can {

namespace {

isotp::Link g_link;
bool g_ready = false;

}  // namespace

bool init() {
  isotp::Config cfg = {};
  cfg.rxId         = OBD_CAN_REQUEST_ID;
  cfg.txId         = OBD_CAN_RESPONSE_ID;
  cfg.bmwAddressed = true;
  cfg.ecuAddr      = uds_bdc::kCanEcuAddr;
  cfg.testerAddr   = 0xF1;

  switch (OBD_CAN_CHANNEL_SEL) {
    case 1:
      cfg.channel = CanChannel::Can2_Mcp;
      break;
    case 2:
      cfg.channel = CanChannel::Both;
      break;
    default:
      cfg.channel = CanChannel::Can1_Twai;
      break;
  }

  isotp::initLink(g_link, cfg);
  g_ready = true;

  Serial.printf("[OBD-CAN] ISO-TP BMW addr mode  req=0x%03X resp=0x%03X ecu=0x%02X\n",
                cfg.rxId, cfg.txId, cfg.ecuAddr);
  return true;
}

void onCanFrame(uint32_t id, const uint8_t* data, uint8_t dlc, bool fromCan1) {
  if (!g_ready || !data) return;

  const int sel = OBD_CAN_CHANNEL_SEL;
  if (fromCan1 && sel == 1) return;
  if (!fromCan1 && sel == 0) return;

  CanFrame f = {};
  f.id       = id;
  f.dlc      = dlc;
  f.extended = false;
  memcpy(f.data, data, dlc > 8 ? 8 : dlc);

  if (!isotp::onCanFrame(g_link, f)) return;

  uint8_t req[isotp::kMaxPayload];
  const size_t reqLen = g_link.rxLen;
  memcpy(req, g_link.rxBuf, reqLen);

  uint8_t resp[isotp::kMaxPayload];
  const size_t respLen =
      uds_bdc::handleRequest(req, reqLen, resp, sizeof(resp));
  if (respLen == 0) return;

  if (!isotp::send(g_link, resp, respLen)) {
    Serial.println(F("[OBD-CAN] ISO-TP TX failed"));
  }
}

}  // namespace obd_can
