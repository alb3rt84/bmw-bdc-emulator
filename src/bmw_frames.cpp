/**
 * @file bmw_frames.cpp
 * @brief Hardcoded BMW G-Chassis (SP2018) cyclic BDC frames + TX task.
 */

#include "bmw_frames.h"

#include <string.h>

namespace bmw {

// ---------------------------------------------------------------------------
// EDIT ZONE — real G-Chassis IDs / payloads (bench defaults)
// Inject vehicle-specific HEX here or via setPayload() at runtime.
// ---------------------------------------------------------------------------
static CyclicFrame g_table[] = {
    // 1) Network Management (OSEK NM) — bus wake-up, every 100 ms
    //    Frame ID 0x510 = BDC node NM. Keeps KOMBI / HU awake.
    {
        "NM_BDC_0x510",
        0x510,
        8,
        {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        100,
        0,
        CanChannel::Both,
    },

    // 2) Zustand Klemmen / Terminal Control — every 100 ms
    //    Frame ID 0x12F from BDC. Payload emulates KL15 ON + KL30B ON.
    {
        "Terminal_0x12F",
        0x12F,
        8,
        {0x45, 0xFF, 0x45, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
        100,
        0,
        CanChannel::Both,
    },

    // 3) Fahrzustand / Vehicle movement status — every 20 ms
    //    Frame ID 0x34A. Stationary, engine-off, but "alive" so MGU leaves logo.
    {
        "Fahrzustand_0x34A",
        0x34A,
        8,
        {0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        20,
        0,
        CanChannel::Both,
    },

    // 4) Zeit_Datum / Time-Date sync — every 1000 ms
    //    Frame ID 0x2F8. Dummy clock to silence cluster/HU sync DTCs.
    {
        "ZeitDatum_0x2F8",
        0x2F8,
        8,
        {0x24, 0x0C, 0x0F, 0x0E, 0x00, 0x00, 0x00, 0xFF},
        1000,
        0,
        CanChannel::Both,
    },
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
      memcpy(t[i].payload, data, len);
      t[i].dlc = len;
      return true;
    }
  }
  return false;
}

void cyclicTxTask(void* /*arg*/) {
  size_t count = 0;
  CyclicFrame* table = getCyclicTable(count);

  Serial.printf("[BMW] Cyclic TX task started (%u frames)\n", (unsigned)count);

  for (;;) {
    const uint32_t now = millis();

    for (size_t i = 0; i < count; i++) {
      CyclicFrame& e = table[i];
      if ((now - e.lastSentMs) < e.periodMs) continue;

      CanFrame frame = {};
      frame.id       = e.id;
      frame.dlc      = e.dlc;
      frame.extended = false;
      memcpy(frame.data, e.payload, e.dlc);

      (void)canBusSend(e.channel, frame);
      e.lastSentMs = now;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

}  // namespace bmw
