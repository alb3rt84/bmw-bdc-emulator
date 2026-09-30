/**
 * @file obd_can.h
 * @brief Factory-style BDC diagnostics over CAN (ISO-TP + BMW 0x6F1 addressing).
 */

#pragma once

#include <stdint.h>

namespace obd_can {

bool init();

/**
 * Called from the CAN RX task for every received frame.
 * When a complete UDS request is assembled, runs uds_bdc and replies on CAN.
 */
void onCanFrame(uint32_t id, const uint8_t* data, uint8_t dlc, bool fromCan1);

}  // namespace obd_can
