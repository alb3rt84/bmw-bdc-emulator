/**
 * @file bench_vin.cpp
 * @brief Runtime VIN shared by HSFZ identification, DoIP and UDS F190.
 */

#include "bench_vin.h"

#include "config.h"
#include "pc_link.h"

#include <Arduino.h>
#include <Preferences.h>
#include <string.h>

namespace bench_vin {

namespace {

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
char g_vin[18] = BENCH_VIN;

bool legalChar(char c) {
  if (c >= '0' && c <= '9') return true;
  if (c >= 'a' && c <= 'z') return false;
  if (c < 'A' || c > 'Z') return false;
  return c != 'I' && c != 'O' && c != 'Q';
}

}  // namespace

void load() {
  char stored[18] = {};
  Preferences prefs;
  prefs.begin("bdcvin", true);
  // 2 = VIN stored after the G20 identity was installed. Older bench VINs stay unused.
  const uint8_t gen = prefs.getUChar("gen", 0);
  const String saved = prefs.getString("vin", "");
  prefs.end();
  if (gen == 2 && saved.length() == 17) {
    memcpy(stored, saved.c_str(), 17);
    stored[17] = '\0';
    bool ok = true;
    for (int i = 0; i < 17; i++) {
      if (!legalChar(stored[i])) ok = false;
    }
    if (ok) {
      portENTER_CRITICAL(&g_mux);
      memcpy(g_vin, stored, 18);
      portEXIT_CRITICAL(&g_mux);
    }
  }
  char msg[48];
  char vin[18];
  copy(vin);
  snprintf(msg, sizeof(msg), "[VIN] announcing %s", vin);
  pc_link::noteLine(msg);
}

void copy(char out[18]) {
  portENTER_CRITICAL(&g_mux);
  memcpy(out, g_vin, 18);
  portEXIT_CRITICAL(&g_mux);
}

bool set(const char* vin) {
  if (!vin) return false;
  char upper[18] = {};
  for (int i = 0; i < 17; i++) {
    char c = vin[i];
    if (c == '\0') return false;
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
    if (!legalChar(c)) return false;
    upper[i] = c;
  }
  if (vin[17] != '\0') return false;

  portENTER_CRITICAL(&g_mux);
  memcpy(g_vin, upper, 18);
  portEXIT_CRITICAL(&g_mux);

  Preferences prefs;
  prefs.begin("bdcvin", false);
  prefs.putUChar("gen", 2);
  prefs.putString("vin", upper);
  prefs.end();

  char msg[48];
  snprintf(msg, sizeof(msg), "[VIN] announcing %s", upper);
  pc_link::noteLine(msg);
  return true;
}

}  // namespace bench_vin
