/**
 * @file uds_bdc.cpp
 * @brief Shared BDC UDS server (session, TesterPresent, DID, DTC stub).
 *
 * Same handler is used for:
 *   - CAN OBD (ISO-TP / BMW 0x6F1)
 *   - DoIP Ethernet (ISO 13400 diagnostic messages)
 */

#include "uds_bdc.h"
#include "bmw_frames.h"
#include "config.h"

#include <Arduino.h>
#include <string.h>

namespace uds_bdc {

namespace {

uint8_t g_session = 0x01;  // defaultSession

// Same VIN HSFZ and DoIP announce. I and O are not legal in a VIN.
const char kVin[17] = {
    BENCH_VIN[0],  BENCH_VIN[1],  BENCH_VIN[2],  BENCH_VIN[3],  BENCH_VIN[4],
    BENCH_VIN[5],  BENCH_VIN[6],  BENCH_VIN[7],  BENCH_VIN[8],  BENCH_VIN[9],
    BENCH_VIN[10], BENCH_VIN[11], BENCH_VIN[12], BENCH_VIN[13], BENCH_VIN[14],
    BENCH_VIN[15], BENCH_VIN[16]};

size_t neg(uint8_t* out, size_t outMax, uint8_t sid, uint8_t nrc) {
  if (outMax < 3) return 0;
  out[0] = 0x7F;
  out[1] = sid;
  out[2] = nrc;
  return 3;
}

size_t posSid(uint8_t* out, size_t outMax, uint8_t sid) {
  if (outMax < 1) return 0;
  out[0] = (uint8_t)(sid + 0x40);
  return 1;
}

size_t handleSession(const uint8_t* req, size_t len, uint8_t* out, size_t outMax) {
  if (len < 2) return neg(out, outMax, 0x10, 0x13);  // incorrectMessageLength
  const uint8_t type = req[1];
  if (type != 0x01 && type != 0x02 && type != 0x03) {
    return neg(out, outMax, 0x10, 0x12);  // subFunctionNotSupported
  }
  g_session = type;
  if (outMax < 6) return neg(out, outMax, 0x10, 0x10);
  out[0] = 0x50;
  out[1] = type;
  // P2 / P2* placeholders (ms encoding per ISO 14229)
  out[2] = 0x00;
  out[3] = 0x32;  // P2 = 50 ms
  out[4] = 0x01;
  out[5] = 0xF4;  // P2* = 5000 ms
  Serial.printf("[UDS] Session -> 0x%02X\n", type);
  return 6;
}

size_t handleTesterPresent(const uint8_t* req, size_t len, uint8_t* out,
                           size_t outMax) {
  if (len < 2) return neg(out, outMax, 0x3E, 0x13);
  const uint8_t sf = req[1];
  if ((sf & 0x7F) != 0x00) return neg(out, outMax, 0x3E, 0x12);
  if (sf & 0x80) return 0;  // suppressPosRspMsgIndicationBit
  if (outMax < 2) return 0;
  out[0] = 0x7E;
  out[1] = 0x00;
  return 2;
}

size_t handleReadDid(const uint8_t* req, size_t len, uint8_t* out, size_t outMax) {
  if (len < 3) return neg(out, outMax, 0x22, 0x13);
  const uint16_t did = (uint16_t)((req[1] << 8) | req[2]);

  // F190 — VIN
  if (did == 0xF190) {
    if (outMax < 3 + 17) return neg(out, outMax, 0x22, 0x10);
    out[0] = 0x62;
    out[1] = 0xF1;
    out[2] = 0x90;
    memcpy(out + 3, kVin, 17);
    return 3 + 17;
  }

  // F186 — ActiveDiagnosticSession
  if (did == 0xF186) {
    if (outMax < 4) return neg(out, outMax, 0x22, 0x10);
    out[0] = 0x62;
    out[1] = 0xF1;
    out[2] = 0x86;
    out[3] = g_session;
    return 4;
  }

  // Custom bench DID 0x0100 — Terminal 15 / live signals snapshot
  //  [ign:u8][rpm:u16 BE][spd_x10:u16 BE][fuel:u8][clt:i8]
  if (did == 0x0100) {
    const bmw::LiveSignals s = bmw::getSignals();
    if (outMax < 3 + 7) return neg(out, outMax, 0x22, 0x10);
    out[0] = 0x62;
    out[1] = 0x01;
    out[2] = 0x00;
    out[3] = s.ignitionOn ? 0x01 : 0x00;
    out[4] = (uint8_t)(s.rpm >> 8);
    out[5] = (uint8_t)(s.rpm & 0xFF);
    const uint16_t spd = (uint16_t)(s.speedKmh * 10.f);
    out[6] = (uint8_t)(spd >> 8);
    out[7] = (uint8_t)(spd & 0xFF);
    out[8] = (uint8_t)s.fuelPct;
    out[9] = (uint8_t)s.coolantC;
    return 10;
  }

  // F18C — ECU Serial Number (placeholder ASCII)
  if (did == 0xF18C) {
    const char* sn = "BDC-EMU-0001";
    const size_t n = strlen(sn);
    if (outMax < 3 + n) return neg(out, outMax, 0x22, 0x10);
    out[0] = 0x62;
    out[1] = 0xF1;
    out[2] = 0x8C;
    memcpy(out + 3, sn, n);
    return 3 + n;
  }

  return neg(out, outMax, 0x22, 0x31);  // requestOutOfRange
}

size_t handleClearDtc(const uint8_t* req, size_t len, uint8_t* out, size_t outMax) {
  (void)req;
  if (len < 4) return neg(out, outMax, 0x14, 0x13);
  // Accept group 0xFFFFFF
  return posSid(out, outMax, 0x14);
}

size_t handleReadDtc(const uint8_t* req, size_t len, uint8_t* out, size_t outMax) {
  if (len < 2) return neg(out, outMax, 0x19, 0x13);
  const uint8_t sf = req[1];
  // 0x01 reportNumberOfDTCByStatusMask — return 0 DTCs
  if (sf == 0x01) {
    if (len < 3 || outMax < 6) return neg(out, outMax, 0x19, 0x13);
    out[0] = 0x59;
    out[1] = 0x01;
    out[2] = req[2];  // echo status mask
    out[3] = 0x00;    // ISO15031-6 DTCFormat
    out[4] = 0x00;    // count hi
    out[5] = 0x00;    // count lo
    return 6;
  }
  // 0x02 reportDTCByStatusMask — empty list
  if (sf == 0x02) {
    if (len < 3 || outMax < 3) return neg(out, outMax, 0x19, 0x13);
    out[0] = 0x59;
    out[1] = 0x02;
    out[2] = req[2];
    return 3;
  }
  return neg(out, outMax, 0x19, 0x12);
}

}  // namespace

bool init() {
  g_session = 0x01;
  Serial.println(F("[UDS] BDC server ready (shared CAN OBD + DoIP)"));
  return true;
}

uint8_t currentSession() {
  return g_session;
}

size_t handleRequest(const uint8_t* req, size_t reqLen, uint8_t* out, size_t outMax) {
  if (!req || !out || reqLen == 0 || outMax < 3) return 0;

  const uint8_t sid = req[0];
  switch (sid) {
    case 0x10:
      return handleSession(req, reqLen, out, outMax);
    case 0x3E:
      return handleTesterPresent(req, reqLen, out, outMax);
    case 0x22:
      return handleReadDid(req, reqLen, out, outMax);
    case 0x14:
      return handleClearDtc(req, reqLen, out, outMax);
    case 0x19:
      return handleReadDtc(req, reqLen, out, outMax);
    default:
      return neg(out, outMax, sid, 0x11);  // serviceNotSupported
  }
}

}  // namespace uds_bdc
