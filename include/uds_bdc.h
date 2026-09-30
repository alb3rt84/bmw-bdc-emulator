/**
 * @file uds_bdc.h
 * @brief Shared UDS server for emulated BMW BDC — used by CAN OBD and DoIP.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace uds_bdc {

/** Default logical address used in DoIP (overridden by bdc_config NVS). */
constexpr uint16_t kLogicalAddress = 0x0010;

/** BMW ECU address byte used on CAN 0x6F1 addressed framing. */
constexpr uint8_t kCanEcuAddr = 0x10;

bool init();

/**
 * Process one UDS request, write positive/negative response into out[].
 * @return response length (0 = suppress response, e.g. TesterPresent SPRMIB)
 */
size_t handleRequest(const uint8_t* req, size_t reqLen, uint8_t* out, size_t outMax);

/** Current diagnostic session (0x01 default, 0x03 extended, …). */
uint8_t currentSession();

}  // namespace uds_bdc
