/**
 * @file uds_bdc.cpp
 * @brief Shared BDC UDS server — VIN/FA/I-Stufe from bdc_config.
 */

#include "uds_bdc.h"
#include "bdc_config.h"
#include "bmw_frames.h"

#include <Arduino.h>
#include <string.h>

namespace uds_bdc {

namespace {

uint8_t g_session = 0x01;

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

size_t replyAsciiDid(uint8_t* out, size_t outMax, uint16_t did, const char* text) {
  const size_t n = strlen(text);
  if (outMax < 3 + n) return 0;
  out[0] = 0x62;
  out[1] = (uint8_t)(did >> 8);
  out[2] = (uint8_t)(did & 0xFF);
  memcpy(out + 3, text, n);
  return 3 + n;
}

size_t handleSession(const uint8_t* req, size_t len, uint8_t* out, size_t outMax) {
  if (len < 2) return neg(out, outMax, 0x10, 0x13);
  const uint8_t type = req[1];
  if (type != 0x01 && type != 0x02 && type != 0x03) {
    return neg(out, outMax, 0x10, 0x12);
  }
  g_session = type;
  if (outMax < 6) return neg(out, outMax, 0x10, 0x10);
  out[0] = 0x50;
  out[1] = type;
  out[2] = 0x00;
  out[3] = 0x32;
  out[4] = 0x01;
  out[5] = 0xF4;
  Serial.printf("[UDS] Session -> 0x%02X\n", type);
  return 6;
}

size_t handleTesterPresent(const uint8_t* req, size_t len, uint8_t* out,
                           size_t outMax) {
  if (len < 2) return neg(out, outMax, 0x3E, 0x13);
  const uint8_t sf = req[1];
  if ((sf & 0x7F) != 0x00) return neg(out, outMax, 0x3E, 0x12);
  if (sf & 0x80) return 0;
  if (outMax < 2) return 0;
  out[0] = 0x7E;
  out[1] = 0x00;
  return 2;
}

size_t handleReadDid(const uint8_t* req, size_t len, uint8_t* out, size_t outMax) {
  if (len < 3) return neg(out, outMax, 0x22, 0x13);
  const uint16_t did = (uint16_t)((req[1] << 8) | req[2]);
  const bdc_config::Identity& id = bdc_config::get();

  if (did == 0xF190) {
    const size_t n = replyAsciiDid(out, outMax, did, id.vin);
    return n ? n : neg(out, outMax, 0x22, 0x10);
  }
  if (did == 0xF186) {
    if (outMax < 4) return neg(out, outMax, 0x22, 0x10);
    out[0] = 0x62;
    out[1] = 0xF1;
    out[2] = 0x86;
    out[3] = g_session;
    return 4;
  }
  if (did == 0xF18C) {
    const size_t n = replyAsciiDid(out, outMax, did, id.serial);
    return n ? n : neg(out, outMax, 0x22, 0x10);
  }

  // Bench identity DIDs (editable via Companion / 0x2E)
  // 0x0101 FA (ASCII), 0x0102 I-Stufe, 0x0103 Model/Baureihe
  if (did == 0x0101) {
    const size_t n = replyAsciiDid(out, outMax, did, id.fa);
    return n ? n : neg(out, outMax, 0x22, 0x10);
  }
  if (did == 0x0102) {
    const size_t n = replyAsciiDid(out, outMax, did, id.iStufe);
    return n ? n : neg(out, outMax, 0x22, 0x10);
  }
  if (did == 0x0103) {
    const size_t n = replyAsciiDid(out, outMax, did, id.model);
    return n ? n : neg(out, outMax, 0x22, 0x10);
  }

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

  return neg(out, outMax, 0x22, 0x31);
}

size_t handleWriteDid(const uint8_t* req, size_t len, uint8_t* out, size_t outMax) {
  // Require extended session for identity writes
  if (g_session != 0x03) return neg(out, outMax, 0x2E, 0x7E);  // not in correct session
  if (len < 4) return neg(out, outMax, 0x2E, 0x13);
  const uint16_t did = (uint16_t)((req[1] << 8) | req[2]);
  const char* data = (const char*)(req + 3);
  const size_t dataLen = len - 3;

  char tmp[bdc_config::kFaMax + 1];
  if (dataLen > bdc_config::kFaMax) return neg(out, outMax, 0x2E, 0x13);
  memcpy(tmp, data, dataLen);
  tmp[dataLen] = '\0';

  bool ok = false;
  if (did == 0xF190) {
    ok = bdc_config::setVin(tmp);
  } else if (did == 0xF18C) {
    ok = bdc_config::setSerial(tmp);
  } else if (did == 0x0101) {
    ok = bdc_config::setFa(tmp);
  } else if (did == 0x0102) {
    ok = bdc_config::setIStufe(tmp);
  } else if (did == 0x0103) {
    ok = bdc_config::setModel(tmp);
  } else {
    return neg(out, outMax, 0x2E, 0x31);
  }

  if (!ok) return neg(out, outMax, 0x2E, 0x22);  // conditionsNotCorrect
  bdc_config::save();
  if (outMax < 3) return 0;
  out[0] = 0x6E;
  out[1] = req[1];
  out[2] = req[2];
  return 3;
}

size_t handleClearDtc(const uint8_t* req, size_t len, uint8_t* out, size_t outMax) {
  (void)req;
  if (len < 4) return neg(out, outMax, 0x14, 0x13);
  return posSid(out, outMax, 0x14);
}

size_t handleReadDtc(const uint8_t* req, size_t len, uint8_t* out, size_t outMax) {
  if (len < 2) return neg(out, outMax, 0x19, 0x13);
  const uint8_t sf = req[1];
  if (sf == 0x01) {
    if (len < 3 || outMax < 6) return neg(out, outMax, 0x19, 0x13);
    out[0] = 0x59;
    out[1] = 0x01;
    out[2] = req[2];
    out[3] = 0x00;
    out[4] = 0x00;
    out[5] = 0x00;
    return 6;
  }
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
  Serial.println(F("[UDS] BDC server ready (VIN/FA from bdc_config)"));
  return true;
}

uint8_t currentSession() {
  return g_session;
}

size_t handleRequest(const uint8_t* req, size_t reqLen, uint8_t* out, size_t outMax) {
  if (!req || !out || reqLen == 0 || outMax < 3) return 0;

  switch (req[0]) {
    case 0x10:
      return handleSession(req, reqLen, out, outMax);
    case 0x3E:
      return handleTesterPresent(req, reqLen, out, outMax);
    case 0x22:
      return handleReadDid(req, reqLen, out, outMax);
    case 0x2E:
      return handleWriteDid(req, reqLen, out, outMax);
    case 0x14:
      return handleClearDtc(req, reqLen, out, outMax);
    case 0x19:
      return handleReadDtc(req, reqLen, out, outMax);
    default:
      return neg(out, outMax, req[0], 0x11);
  }
}

}  // namespace uds_bdc
