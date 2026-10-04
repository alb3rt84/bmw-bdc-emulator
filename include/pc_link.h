/**
 * @file pc_link.h
 * @brief Companion PC protocol — JSON lines over USB-Serial and UDP :13401.
 */

#pragma once

#include <stdint.h>

namespace pc_link {

bool init();
void task(void* arg);

/** One line for the PC log window (Ethernet, clamps). */
void noteLine(const char* line);

/** CAN frame received on the MCP2515. */
void noteCan(uint32_t id, const uint8_t* data, uint8_t dlc);

/** FA blob just stored, so the PC window can show a Rheingold write. */
void noteFa(const uint8_t* data, size_t len);

}  // namespace pc_link
