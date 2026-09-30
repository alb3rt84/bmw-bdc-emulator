/**
 * @file can_bus.h
 * @brief Dual-CAN abstraction: ESP32 TWAI (CAN1) + MCP2515 SPI (CAN2).
 *
 * Non-blocking send/receive. Call canBusInit() once from setup(), then use
 * FreeRTOS tasks for cyclic TX and RX polling.
 */

#pragma once

#include <Arduino.h>
#include <stdint.h>

struct CanFrame {
  uint32_t id;
  uint8_t  dlc;
  uint8_t  data[8];
  bool     extended;  // false = standard 11-bit (BMW network traffic)
};

enum class CanChannel : uint8_t {
  Can1_Twai = 0,
  Can2_Mcp  = 1,
  Both      = 2,
};

/**
 * Initialise TWAI @ 500 kbit/s and MCP2515 @ 500 kbit/s.
 * @return true if both controllers started successfully.
 */
bool canBusInit();

/** Non-blocking transmit on the selected channel(s). */
bool canBusSend(CanChannel ch, const CanFrame& frame);

/**
 * Non-blocking receive. Returns true if a frame was available.
 * Prefer calling from a dedicated RX task with a short timeout.
 */
bool canBusReceive(CanChannel ch, CanFrame& out, uint32_t timeoutMs = 0);

/** Print frame to Serial (debug only — avoid in hot paths). */
void canBusLogFrame(const char* prefix, CanChannel ch, const CanFrame& f);
