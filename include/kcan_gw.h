/**
 * @file kcan_gw.h
 * @brief ENET/DoIP to one K-CAN. The module sits on the MCP2515 adapter.
 *
 * Physical DoIP target 0x00XX is sent as BMW ISO-TP on CAN ID 0x6F1 with
 * address byte XX. The module answers on 0x600|XX. BDC 0x0010 stays local.
 */

#pragma once

#include "can_bus.h"

#include <stddef.h>
#include <stdint.h>

namespace kcan_gw {

void init();

/**
 * Feed a frame from the module K-CAN (TWAI). Returns true when an active
 * transaction consumed it.
 */
bool onCanFrame(const CanFrame& frame);

/**
 * Send one UDS request as the tester and wait for the module response.
 * @param ecuAddr BMW address (low byte of the DoIP logical address). 0xDF = functional.
 * @param responder set to the ECU that answered (same as ecuAddr, or the
 *        responder's address after a functional request).
 * @return response length, 0 on timeout or CAN error.
 */
size_t transact(uint8_t ecuAddr, const uint8_t* req, size_t reqLen,
                uint8_t* resp, size_t respMax, uint8_t* responder);

}  // namespace kcan_gw
