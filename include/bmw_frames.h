/**
 * @file bmw_frames.h
 * @brief BMW G-Chassis cyclic frames + live signal injection API.
 */

#pragma once

#include "can_bus.h"

#include <stdint.h>

namespace bmw {

struct CyclicFrame {
  const char* name;
  uint32_t    id;
  uint8_t     dlc;
  uint8_t     payload[8];
  uint32_t    periodMs;
  uint32_t    lastSentMs;
  CanChannel  channel;
};

struct LiveSignals {
  bool     ignitionOn;   // Terminal 15
  uint16_t rpm;          // 0..8000
  float    speedKmh;     // 0..300
  float    fuelPct;      // 0..100
  int16_t  coolantC;     // -40..140
};

CyclicFrame* getCyclicTable(size_t& count);

bool setPayload(uint32_t id, const uint8_t* data, uint8_t len);

/** Snapshot of live dashboard values (thread-safe-ish via atomic-ish copy). */
LiveSignals getSignals();

/**
 * Apply companion-app values into cyclic payloads.
 * Called from the PC-link task; cyclic TX task always reads latest payloads.
 */
void setIgnition(bool on);
void setRpm(uint16_t rpm);
void setSpeedKmh(float kmh);
void setFuelPct(float pct);
void setCoolantC(int16_t celsius);
void setSignals(const LiveSignals& s);

void cyclicTxTask(void* arg);

}  // namespace bmw
