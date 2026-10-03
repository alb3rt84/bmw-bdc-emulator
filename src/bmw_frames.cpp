/**
 * @file bmw_frames.cpp
 * @brief Hardcoded SP2018 cyclic frames + live encoding for companion GUI.
 *
 * Frame IDs 0x510 / 0x12F / 0x34A / 0x2F8 are the requested G-Chassis BDC set.
 * RPM / speed / fuel / coolant IDs use common BMW encodings — mark EDIT if your
 * DBC differs (G-series powertrain IDs vary by option code).
 */

#include "bmw_frames.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

namespace bmw {

namespace {

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
LiveSignals g_sig = {true, 0, 0.f, 50.f, 90};

// Terminal 15 payloads for Zustand Klemmen 0x12F
const uint8_t kIgnOn[8]  = {0x45, 0xFF, 0x45, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const uint8_t kIgnOff[8] = {0x00, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

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
// Cyclic table — BDC wake/ignition + live instrument signals
// ---------------------------------------------------------------------------
static CyclicFrame g_table[] = {
    {"NM_BDC_0x510", 0x510, 8,
     {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
     100, 0, CanChannel::Can1_Twai},

    {"Terminal_0x12F", 0x12F, 8,
     {0x45, 0xFF, 0x45, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
     100, 0, CanChannel::Can1_Twai},

    {"Fahrzustand_0x34A", 0x34A, 8,
     {0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
     20, 0, CanChannel::Can1_Twai},

    {"ZeitDatum_0x2F8", 0x2F8, 8,
     {0x24, 0x0C, 0x0F, 0x0E, 0x00, 0x00, 0x00, 0xFF},
     1000, 0, CanChannel::Can1_Twai},

    // --- Live companion signals (EDIT IDs / layouts to match your DBC) ---
    {"RPM_0x0A5", 0x0A5, 8,
     {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
     20, 0, CanChannel::Can1_Twai},

    {"Speed_0x1A1", 0x1A1, 8,
     {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
     20, 0, CanChannel::Can1_Twai},

    {"Coolant_0x1D0", 0x1D0, 8,
     {0x8A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // 90+48=138=0x8A
     100, 0, CanChannel::Can1_Twai},

    {"Fuel_0x349", 0x349, 8,
     {0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // 50%
     200, 0, CanChannel::Can1_Twai},
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

void setIgnition(bool on) {
  portENTER_CRITICAL(&g_mux);
  g_sig.ignitionOn = on;
  portEXIT_CRITICAL(&g_mux);
  setPayload(0x12F, on ? kIgnOn : kIgnOff, 8);
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

void setSignals(const LiveSignals& s) {
  setIgnition(s.ignitionOn);
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
