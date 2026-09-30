/**
 * @file bdc_config.cpp
 * @brief BDC identity store with ESP32 Preferences (NVS).
 */

#include "bdc_config.h"

#include <Arduino.h>
#include <Preferences.h>
#include <string.h>

namespace bdc_config {

namespace {

Preferences g_prefs;
Identity g_id;
bool g_ready = false;

void fillDefaults(Identity& id) {
  memset(&id, 0, sizeof(id));
  strncpy(id.vin, "WBADEMOGCHASSIS01", kVinLen);
  id.vin[kVinLen] = '\0';
  strncpy(id.fa, "FA_DEMO_G30_BDC", kFaMax);
  strncpy(id.iStufe, "I001-21-03-500", kIStufeMax);
  strncpy(id.serial, "BDC-EMU-0001", kSerialMax);
  strncpy(id.model, "G30", kModelMax);
  id.logicalAddress = 0x0010;
}

bool validVin(const char* v) {
  if (!v || strlen(v) != kVinLen) return false;
  for (size_t i = 0; i < kVinLen; i++) {
    const char c = v[i];
    const bool ok = (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9');
    if (!ok) return false;
  }
  return true;
}

}  // namespace

bool init() {
  fillDefaults(g_id);
  g_prefs.begin("bdc_cfg", false);
  load();
  g_ready = true;
  Serial.printf("[CFG] VIN=%s FA=%s I=%s LA=0x%04X\n", g_id.vin, g_id.fa,
                g_id.iStufe, g_id.logicalAddress);
  return true;
}

const Identity& get() {
  return g_id;
}

Identity copy() {
  return g_id;
}

void resetDefaults() {
  fillDefaults(g_id);
}

bool load() {
  if (!g_prefs.isKey("vin")) {
    return false;
  }
  String vin = g_prefs.getString("vin", g_id.vin);
  String fa = g_prefs.getString("fa", g_id.fa);
  String ist = g_prefs.getString("istufe", g_id.iStufe);
  String ser = g_prefs.getString("serial", g_id.serial);
  String model = g_prefs.getString("model", g_id.model);
  const uint16_t la = (uint16_t)g_prefs.getUShort("la", g_id.logicalAddress);

  if (vin.length() == (unsigned)kVinLen) {
    strncpy(g_id.vin, vin.c_str(), kVinLen);
    g_id.vin[kVinLen] = '\0';
  }
  strncpy(g_id.fa, fa.c_str(), kFaMax);
  g_id.fa[kFaMax] = '\0';
  strncpy(g_id.iStufe, ist.c_str(), kIStufeMax);
  g_id.iStufe[kIStufeMax] = '\0';
  strncpy(g_id.serial, ser.c_str(), kSerialMax);
  g_id.serial[kSerialMax] = '\0';
  strncpy(g_id.model, model.c_str(), kModelMax);
  g_id.model[kModelMax] = '\0';
  g_id.logicalAddress = la ? la : 0x0010;
  return true;
}

bool save() {
  g_prefs.putString("vin", g_id.vin);
  g_prefs.putString("fa", g_id.fa);
  g_prefs.putString("istufe", g_id.iStufe);
  g_prefs.putString("serial", g_id.serial);
  g_prefs.putString("model", g_id.model);
  g_prefs.putUShort("la", g_id.logicalAddress);
  Serial.println(F("[CFG] Saved to NVS"));
  return true;
}

bool setVin(const char* vin17) {
  if (!validVin(vin17)) return false;
  memcpy(g_id.vin, vin17, kVinLen);
  g_id.vin[kVinLen] = '\0';
  return true;
}

bool setFa(const char* fa) {
  if (!fa) return false;
  strncpy(g_id.fa, fa, kFaMax);
  g_id.fa[kFaMax] = '\0';
  return true;
}

bool setIStufe(const char* s) {
  if (!s) return false;
  strncpy(g_id.iStufe, s, kIStufeMax);
  g_id.iStufe[kIStufeMax] = '\0';
  return true;
}

bool setSerial(const char* s) {
  if (!s) return false;
  strncpy(g_id.serial, s, kSerialMax);
  g_id.serial[kSerialMax] = '\0';
  return true;
}

bool setModel(const char* s) {
  if (!s) return false;
  strncpy(g_id.model, s, kModelMax);
  g_id.model[kModelMax] = '\0';
  return true;
}

bool setLogicalAddress(uint16_t la) {
  if (la == 0) return false;
  g_id.logicalAddress = la;
  return true;
}

bool applyPatch(const Identity& patch, bool hasVin, bool hasFa, bool hasIStufe,
                bool hasSerial, bool hasModel, bool hasLa) {
  bool ok = true;
  if (hasVin) ok = setVin(patch.vin) && ok;
  if (hasFa) ok = setFa(patch.fa) && ok;
  if (hasIStufe) ok = setIStufe(patch.iStufe) && ok;
  if (hasSerial) ok = setSerial(patch.serial) && ok;
  if (hasModel) ok = setModel(patch.model) && ok;
  if (hasLa) ok = setLogicalAddress(patch.logicalAddress) && ok;
  return ok;
}

}  // namespace bdc_config
