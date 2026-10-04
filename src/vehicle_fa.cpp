/**
 * @file vehicle_fa.cpp
 * @brief Persisted FA blob and I-Stufe for the ZGW emulator.
 */

#include "vehicle_fa.h"

#include "pc_link.h"

#include <Arduino.h>
#include <Preferences.h>
#include <stdio.h>
#include <string.h>

namespace vehicle_fa {

namespace {

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
uint8_t g_fa[kMaxFa];
size_t g_len = 0;
char g_istufe[16] = {};
char g_werk[16] = {};
char g_ho[16] = {};

// G20 320d, VIN WBA5V510X0FJ28775. Same bytes as pc_companion/data/FA.xml.
size_t buildDefault(uint8_t* out) {
  static const uint8_t kG20Fa[] = {
#include "g20_fa.inc"
  };
  memcpy(out, kG20Fa, sizeof(kG20Fa));
  return sizeof(kG20Fa);
}

// End of the counted version-3 FA, or 0 when the walk does not fit.
size_t faEnd(const uint8_t* data, size_t len) {
  if (!data || len < 24 || data[0] != 0x03) return 0;
  size_t i = 21;
  const size_t eCount = data[i++];
  if (i + eCount * 4 > len) return 0;
  i += eCount * 4;
  if (i >= len) return 0;
  const size_t saCount = data[i++];
  if (i + saCount * 3 > len) return 0;
  i += saCount * 3;
  if (i >= len) return 0;
  const size_t hoCount = data[i++];
  if (i + hoCount * 4 > len) return 0;
  return i + hoCount * 4;
}

bool layoutOk(const uint8_t* data, size_t len) {
  const size_t end = faEnd(data, len);
  if (end == 0 || end > kMaxFa || len > kPsdzFaBytes) return false;
  for (size_t i = end; i < len; i++) {
    if (data[i] != 0) return false;
  }
  return true;
}

void fillSummary(const uint8_t* data, char out[20]) {
  char series[5], typeKey[5], time[5];
  memcpy(series, data + 1, 4);
  memcpy(typeKey, data + 5, 4);
  memcpy(time, data + 9, 4);
  series[4] = typeKey[4] = time[4] = '\0';
  snprintf(out, 20, "%s %s %s", series, typeKey, time);
}

bool stufeText(const char* in, char out[16]) {
  if (!in) return true;
  char tmp[16];
  size_t n = 0;
  for (; in[n] != '\0'; n++) {
    if (n >= sizeof(tmp) - 1) return false;
    char c = in[n];
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
    tmp[n] = c;
  }
  tmp[n] = '\0';
  if (n == 0) {
    out[0] = '\0';
    return true;
  }
  if (n != 14) return false;
  for (size_t i = 0; i < n; i++) {
    const char c = tmp[i];
    const bool dash = (i == 4 || i == 7 || i == 10);
    if (dash) {
      if (c != '-') return false;
      continue;
    }
    const bool digit = (i >= 5);
    if (digit) {
      if (c < '0' || c > '9') return false;
    } else if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) {
      return false;
    }
  }
  memcpy(out, tmp, 15);
  return true;
}

void logFa() {
  char sum[20];
  summary(sum);
  char msg[40];
  snprintf(msg, sizeof(msg), "[FA] %s", sum);
  pc_link::noteLine(msg);
}

void logVcm() {
  char cur[16], werk[16], ho[16];
  portENTER_CRITICAL(&g_mux);
  memcpy(cur, g_istufe, sizeof(cur));
  memcpy(werk, g_werk, sizeof(werk));
  memcpy(ho, g_ho, sizeof(ho));
  portEXIT_CRITICAL(&g_mux);
  char msg[80];
  snprintf(msg, sizeof(msg), "[VCM] I-Stufe %s werk %s ho %s",
           cur[0] ? cur : "-", werk[0] ? werk : "-", ho[0] ? ho : "-");
  pc_link::noteLine(msg);
}

bool peel(const uint8_t* data, size_t len, const uint8_t** payload, size_t* payloadLen) {
  if (!data || !payload || !payloadLen || len == 0 || len > kPsdzFaBytes) return false;
  if (data[0] == 0x03) {
    const size_t end = faEnd(data, len);
    if (end == 0 || end > kMaxFa) return false;
    for (size_t i = end; i < len; i++) {
      if (data[i] != 0) return false;
    }
    *payload = data;
    *payloadLen = end;
    return true;
  }
  if (len < 4) return false;
  const size_t declared = ((size_t)data[0] << 8) | data[1];
  if (declared < 24 || declared > kMaxFa || 3 + declared > len) return false;
  if (faEnd(data + 3, declared) != declared) return false;
  *payload = data + 3;
  *payloadLen = declared;
  return true;
}

}  // namespace

void load() {
  uint8_t stored[kMaxFa];
  size_t storedLen = 0;
  Preferences prefs;
  prefs.begin("bdcfa", true);
  // 2 = identity written by this G20 build. An older F15 blob has no gen.
  const uint8_t gen = prefs.getUChar("gen", 0);
  const size_t n = prefs.getBytesLength("blob");
  if (gen == 2 && n > 0 && n <= kMaxFa) {
    prefs.getBytes("blob", stored, n);
    storedLen = n;
  }
  prefs.end();

  uint8_t fresh[kMaxFa];
  const size_t freshLen = buildDefault(fresh);
  const uint8_t* use = fresh;
  size_t useLen = freshLen;
  if (layoutOk(stored, storedLen)) {
    use = stored;
    useLen = faEnd(stored, storedLen);
  }

  portENTER_CRITICAL(&g_mux);
  memcpy(g_fa, use, useLen);
  g_len = useLen;
  portEXIT_CRITICAL(&g_mux);
  logFa();

  char cur[16] = {};
  char werk[16] = {};
  char ho[16] = {};
  Preferences vcm;
  vcm.begin("bdcvcm", true);
  const String a = vcm.getString("istufe", "");
  const String b = vcm.getString("werk", "");
  const String c = vcm.getString("ho", "");
  vcm.end();
  if (a.length() < sizeof(cur)) memcpy(cur, a.c_str(), a.length());
  if (b.length() < sizeof(werk)) memcpy(werk, b.c_str(), b.length());
  if (c.length() < sizeof(ho)) memcpy(ho, c.c_str(), c.length());
  char curOk[16] = {};
  char werkOk[16] = {};
  char hoOk[16] = {};
  if (!stufeText(cur, curOk)) curOk[0] = '\0';
  if (!stufeText(werk, werkOk)) werkOk[0] = '\0';
  if (!stufeText(ho, hoOk)) hoOk[0] = '\0';
  portENTER_CRITICAL(&g_mux);
  memcpy(g_istufe, curOk, sizeof(g_istufe));
  memcpy(g_werk, werkOk, sizeof(g_werk));
  memcpy(g_ho, hoOk, sizeof(g_ho));
  portEXIT_CRITICAL(&g_mux);
  if (curOk[0] || werkOk[0] || hoOk[0]) logVcm();
}

size_t copy(uint8_t* out, size_t outMax) {
  if (!out) return 0;
  portENTER_CRITICAL(&g_mux);
  const size_t n = g_len;
  if (n == 0 || n > outMax) {
    portEXIT_CRITICAL(&g_mux);
    return 0;
  }
  memcpy(out, g_fa, n);
  portEXIT_CRITICAL(&g_mux);
  return n;
}

size_t copyWrapped(uint8_t* out, size_t outMax) {
  if (out == nullptr || outMax < kPsdzFaBytes) return 0;
  memset(out, 0, kPsdzFaBytes);
  if (copy(out, kPsdzFaBytes) == 0) return 0;
  return kPsdzFaBytes;
}

bool store(const uint8_t* data, size_t len) {
  const uint8_t* payload = nullptr;
  size_t end = 0;
  if (!peel(data, len, &payload, &end)) return false;
  portENTER_CRITICAL(&g_mux);
  memcpy(g_fa, payload, end);
  g_len = end;
  portEXIT_CRITICAL(&g_mux);
  Preferences prefs;
  prefs.begin("bdcfa", false);
  prefs.putUChar("gen", 2);
  prefs.putBytes("blob", payload, end);
  prefs.end();
  logFa();
  pc_link::noteFa(payload, end);
  return true;
}

void summary(char out[20]) {
  uint8_t head[13];
  portENTER_CRITICAL(&g_mux);
  const size_t n = g_len;
  if (n >= 13) memcpy(head, g_fa, 13);
  portEXIT_CRITICAL(&g_mux);
  if (n < 13) {
    snprintf(out, 20, "FA");
    return;
  }
  fillSummary(head, out);
}

bool setIStufe(const char* current, const char* werk, const char* ho) {
  char cur[16], werkOut[16], hoOut[16];
  portENTER_CRITICAL(&g_mux);
  memcpy(cur, g_istufe, sizeof(cur));
  memcpy(werkOut, g_werk, sizeof(werkOut));
  memcpy(hoOut, g_ho, sizeof(hoOut));
  portEXIT_CRITICAL(&g_mux);
  if (!stufeText(current, cur) || !stufeText(werk, werkOut) || !stufeText(ho, hoOut)) {
    return false;
  }
  portENTER_CRITICAL(&g_mux);
  memcpy(g_istufe, cur, sizeof(g_istufe));
  memcpy(g_werk, werkOut, sizeof(g_werk));
  memcpy(g_ho, hoOut, sizeof(g_ho));
  portEXIT_CRITICAL(&g_mux);
  Preferences prefs;
  prefs.begin("bdcvcm", false);
  prefs.putString("istufe", cur);
  prefs.putString("werk", werkOut);
  prefs.putString("ho", hoOut);
  prefs.end();
  logVcm();
  return true;
}

}  // namespace vehicle_fa
