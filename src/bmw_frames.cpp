/**
 * @file bmw_frames.cpp
 * @brief Hardcoded SP2018 cyclic frames + live encoding for companion GUI.
 *
 * Frame IDs 0x510 / 0x12F / 0x34A / 0x2F8 are the requested G-Chassis BDC set.
 * RPM / speed / fuel / coolant IDs use common BMW encodings — mark EDIT if your
 * DBC differs (G-series powertrain IDs vary by option code).
 */

#include "bmw_frames.h"
#include "pc_link.h"

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

namespace bmw {

namespace {

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
LiveSignals g_sig = {true, true, 0, 0.f, 50.f, 90};
uint32_t g_km = 0;

// 0x12F byte 0 and byte 2. Both clamps on is the ready frame 0x45:
// bit0 ST_KL_R (KL30), bits3-2 ST_KL_15, bits7-6 ST_KEY_VLD.
uint8_t terminalByte(bool kl30, bool kl15) {
  uint8_t b = 0;
  if (kl30) b |= 0x01;
  if (kl15) b |= 0x04;
  if (kl30 || kl15) b |= 0x40;
  return b;
}

/**
 * Encode helpers — EDIT byte layout to match your DBC / captures.
 *
 * RPM 0x0A5: classic BMW style — (byte2<<8|byte3)/4 = RPM  (big-endian word)
 * Speed 0x1A1: km/h * 10 as uint16 LE in bytes 0..1 (placeholder)
 * Coolant 0x1D0: temp_C + 48 in byte 0 (placeholder)
 * Fuel 0x349: percent 0..100 in byte 0 (placeholder)
 */
void encodeRpm(uint8_t out[8], uint16_t rpm) {
  memset(out, 0, 8);
  if (rpm > 8000) rpm = 8000;
  const uint16_t raw = (uint16_t)(rpm * 4);
  out[2] = (uint8_t)(raw >> 8);
  out[3] = (uint8_t)(raw & 0xFF);
}

void encodeSpeed(uint8_t out[8], float kmh) {
  memset(out, 0, 8);
  if (kmh < 0.f) kmh = 0.f;
  if (kmh > 300.f) kmh = 300.f;
  const uint16_t raw = (uint16_t)lroundf(kmh * 10.f);
  out[0] = (uint8_t)(raw & 0xFF);
  out[1] = (uint8_t)(raw >> 8);
}

void encodeCoolant(uint8_t out[8], int16_t c) {
  memset(out, 0, 8);
  if (c < -40) c = -40;
  if (c > 140) c = 140;
  out[0] = (uint8_t)(c + 48);
}

void encodeFuel(uint8_t out[8], float pct) {
  memset(out, 0, 8);
  if (pct < 0.f) pct = 0.f;
  if (pct > 100.f) pct = 100.f;
  out[0] = (uint8_t)lroundf(pct);
}

}  // namespace

// ---------------------------------------------------------------------------
// One cyclic frame on the MCP2515: Klemmen 0x12F, KL30 and KL15.
// ---------------------------------------------------------------------------
static CyclicFrame g_table[] = {
    {"Terminal_0x12F", 0x12F, 8,
     {0x45, 0xFF, 0x45, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
     100, 0, CanChannel::Can2_Mcp},
    // Kombi odometer. Bytes 0–2 are kilometres, little-endian.
    {"Odometer_0x330", 0x330, 8,
     {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
     1000, 0, CanChannel::Can2_Mcp},
};

CyclicFrame* getCyclicTable(size_t& count) {
  count = sizeof(g_table) / sizeof(g_table[0]);
  return g_table;
}

bool setPayload(uint32_t id, const uint8_t* data, uint8_t len) {
  if (!data || len == 0 || len > 8) return false;
  size_t n = 0;
  CyclicFrame* t = getCyclicTable(n);
  for (size_t i = 0; i < n; i++) {
    if (t[i].id == id) {
      portENTER_CRITICAL(&g_mux);
      memcpy(t[i].payload, data, len);
      t[i].dlc = len;
      portEXIT_CRITICAL(&g_mux);
      return true;
    }
  }
  return false;
}

LiveSignals getSignals() {
  portENTER_CRITICAL(&g_mux);
  LiveSignals s = g_sig;
  portEXIT_CRITICAL(&g_mux);
  return s;
}

void publishTerminals() {
  const LiveSignals s = getSignals();
  const uint8_t b = terminalByte(s.kl30On, s.ignitionOn);
  const uint8_t p[8] = {b, 0xFF, b, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  setPayload(0x12F, p, 8);
  char msg[96];
  snprintf(msg, sizeof(msg),
           "[KL] CAN 12F %02X FF %02X FF FF FF FF FF  ENET %02X  KL30=%d KL15=%d",
           b, b, b, s.kl30On ? 1 : 0, s.ignitionOn ? 1 : 0);
  pc_link::noteLine(msg);
}

void setIgnition(bool on) {
  portENTER_CRITICAL(&g_mux);
  g_sig.ignitionOn = on;
  portEXIT_CRITICAL(&g_mux);
  publishTerminals();
}

void setKl30(bool on) {
  portENTER_CRITICAL(&g_mux);
  g_sig.kl30On = on;
  portEXIT_CRITICAL(&g_mux);
  publishTerminals();
}

void setClamps(bool kl30, bool kl15) {
  portENTER_CRITICAL(&g_mux);
  g_sig.kl30On = kl30;
  g_sig.ignitionOn = kl15;
  portEXIT_CRITICAL(&g_mux);
  publishTerminals();
}

uint8_t terminalStatusByte() {
  const LiveSignals s = getSignals();
  return terminalByte(s.kl30On, s.ignitionOn);
}

void setRpm(uint16_t rpm) {
  uint8_t p[8];
  encodeRpm(p, rpm);
  portENTER_CRITICAL(&g_mux);
  g_sig.rpm = rpm > 8000 ? 8000 : rpm;
  portEXIT_CRITICAL(&g_mux);
  setPayload(0x0A5, p, 8);
}

void setSpeedKmh(float kmh) {
  uint8_t p[8];
  encodeSpeed(p, kmh);
  portENTER_CRITICAL(&g_mux);
  g_sig.speedKmh = kmh;
  portEXIT_CRITICAL(&g_mux);
  setPayload(0x1A1, p, 8);
}

void setFuelPct(float pct) {
  uint8_t p[8];
  encodeFuel(p, pct);
  portENTER_CRITICAL(&g_mux);
  g_sig.fuelPct = pct;
  portEXIT_CRITICAL(&g_mux);
  setPayload(0x349, p, 8);
}

void setCoolantC(int16_t celsius) {
  uint8_t p[8];
  encodeCoolant(p, celsius);
  portENTER_CRITICAL(&g_mux);
  g_sig.coolantC = celsius;
  portEXIT_CRITICAL(&g_mux);
  setPayload(0x1D0, p, 8);
}

void publishKm() {
  uint32_t km = 0;
  portENTER_CRITICAL(&g_mux);
  km = g_km;
  portEXIT_CRITICAL(&g_mux);
  uint8_t p[8] = {};
  p[0] = (uint8_t)(km & 0xFF);
  p[1] = (uint8_t)((km >> 8) & 0xFF);
  p[2] = (uint8_t)((km >> 16) & 0xFF);
  setPayload(0x330, p, 8);
}

void loadOdometer() {
  Preferences prefs;
  prefs.begin("bdckm", true);
  const uint32_t saved = prefs.getUInt("km", 0);
  prefs.end();
  portENTER_CRITICAL(&g_mux);
  g_km = saved > 0xFFFFFFu ? 0xFFFFFFu : saved;
  portEXIT_CRITICAL(&g_mux);
  publishKm();
}

bool setOdometer(uint32_t km) {
  if (km > 0xFFFFFFu) return false;
  portENTER_CRITICAL(&g_mux);
  g_km = km;
  portEXIT_CRITICAL(&g_mux);
  publishKm();
  Preferences prefs;
  prefs.begin("bdckm", false);
  prefs.putUInt("km", km);
  prefs.end();
  char msg[48];
  snprintf(msg, sizeof(msg), "[KM] %u na CAN 330", (unsigned)km);
  pc_link::noteLine(msg);
  return true;
}

uint32_t odometerKm() {
  portENTER_CRITICAL(&g_mux);
  const uint32_t km = g_km;
  portEXIT_CRITICAL(&g_mux);
  return km;
}

void setSignals(const LiveSignals& s) {
  setClamps(s.kl30On, s.ignitionOn);
  setRpm(s.rpm);
  setSpeedKmh(s.speedKmh);
  setFuelPct(s.fuelPct);
  setCoolantC(s.coolantC);
}

void cyclicTxTask(void* /*arg*/) {
  size_t count = 0;
  CyclicFrame* table = getCyclicTable(count);

  // Ensure initial encodings match default LiveSignals
  setSignals(getSignals());

  Serial.printf("[BMW] Cyclic TX task started (%u frames)\n", (unsigned)count);

  for (;;) {
    const uint32_t now = millis();

    for (size_t i = 0; i < count; i++) {
      CyclicFrame& e = table[i];
      if ((now - e.lastSentMs) < e.periodMs) continue;

      CanFrame frame = {};
      frame.id       = e.id;
      frame.extended = false;

      portENTER_CRITICAL(&g_mux);
      frame.dlc = e.dlc;
      memcpy(frame.data, e.payload, e.dlc);
      portEXIT_CRITICAL(&g_mux);

      (void)canBusSend(e.channel, frame);
      e.lastSentMs = now;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

}  // namespace bmw
