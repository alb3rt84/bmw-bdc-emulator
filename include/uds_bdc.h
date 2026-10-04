/**
 * @file uds_bdc.h
 * @brief Shared UDS server for emulated BMW BDC — used by CAN OBD and DoIP.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace uds_bdc {

/** Logical address used in DoIP (BDC / ZGM). */
constexpr uint16_t kLogicalAddress = 0x0010;

/** BMW ECU address byte used on CAN 0x6F1 addressed framing. */
constexpr uint8_t kCanEcuAddr = 0x10;

bool init();

/**
 * Process one UDS request, write positive/negative response into out[].
 * @return response length (0 = suppress response, e.g. TesterPresent SPRMIB)
 */
size_t handleRequest(const uint8_t* req, size_t reqLen, uint8_t* out, size_t outMax);

/**
 * Vehicle VIN (ReadDataByIdentifier F190) from the emulator, for any ECU.
 * ISTA reads this from the gateway and from other addresses before it
 * accepts the announced VIN. @return 0 when req is not that service.
 */
size_t answerVin(const uint8_t* req, size_t reqLen, uint8_t* out, size_t outMax);

/** Current diagnostic session (0x01 default, 0x03 extended, …). */
uint8_t currentSession();

}  // namespace uds_bdc
