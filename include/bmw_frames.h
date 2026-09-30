/**
 * @file bmw_frames.h
 * @brief BMW G-Chassis (SP2018) BDC network-management & terminal frames.
 *
 * EDIT HERE: replace placeholder payloads with captures from your vehicle / DBC
 * when you refine the bench behaviour. Frame IDs below are the real G-series
 * identifiers requested for Terminal 15 / wake-up emulation.
 */

#pragma once

#include "can_bus.h"

#include <stdint.h>

namespace bmw {

/** Cyclic schedule entry for the high-priority TX task. */
struct CyclicFrame {
  const char* name;       // human-readable label for logs
  uint32_t    id;         // 11-bit CAN ID
  uint8_t     dlc;
  uint8_t     payload[8]; // mutable so you can inject live HEX at runtime
  uint32_t    periodMs;
  uint32_t    lastSentMs; // updated by the cyclic task (ms since boot)
  CanChannel  channel;    // which bus(es) to mirror the frame onto
};

/**
 * Mutable table of essential BDC frames.
 * The cyclic FreeRTOS task walks this array and transmits when due.
 *
 * --- HOW TO INJECT CUSTOM HEX ---
 * 1. Find the entry by name / ID in bmw_frames.cpp
 * 2. Change payload[i] bytes, or call bmw::setPayload(id, data, len) at runtime
 * 3. Optionally change periodMs
 */
CyclicFrame* getCyclicTable(size_t& count);

/** Runtime payload override for a known frame ID (returns false if not found). */
bool setPayload(uint32_t id, const uint8_t* data, uint8_t len);

/**
 * High-priority FreeRTOS task entry: transmits due cyclic frames without delay().
 * Pass nullptr as argument. Created from main.cpp.
 */
void cyclicTxTask(void* arg);

}  // namespace bmw
