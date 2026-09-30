/**
 * @file lin_master.h
 * @brief LIN Master scheduler template (19200 baud, break + header).
 *
 * Uses a hardware UART + LIN transceiver (TJA1020). Provides a periodic
 * header schedule for polling / commanding LIN slaves (e.g. climate panels).
 */

#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace lin {

struct ScheduleEntry {
  uint8_t  pid;        // protected identifier (6-bit ID + parity), or raw ID 0..63
  uint8_t  dlc;        // expected response length (0 = header-only / master TX)
  uint8_t  data[8];    // master-publish payload when dlc > 0 and masterOwnsData
  bool     masterOwnsData;
  uint32_t periodMs;
  uint32_t lastSentMs;
};

bool init();

/** FreeRTOS task: walks the schedule and emits LIN headers / frames. */
void masterTask(void* arg);

/**
 * Replace / extend the default schedule at runtime.
 * Copies up to maxEntries into the internal table.
 */
void setSchedule(const ScheduleEntry* entries, size_t count);

}  // namespace lin
