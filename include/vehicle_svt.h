/**
 * @file vehicle_svt.h
 * @brief SVT-IST of the baked-in G20. Current controllers, not the VCM store.
 *
 * VCM in the BDC is the vehicle order (FA on 22 3F 06). SVT is the list of
 * ECUs fitted now. 22 F1 01 on one of those addresses returns that ECU SVK.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace vehicle_svt {

/** ECU name from the current SVT, or nullptr when this address is not in it. */
const char* name(uint8_t addr);

/**
 * ReadCurrentSVK (22 F1 01) for an address in the current SVT.
 * @return 0 when req is not that DID or addr is not in the SVT.
 */
size_t answerSvk(uint8_t addr, const uint8_t* req, size_t reqLen,
                 uint8_t* out, size_t outMax);

/** Factory G20 list, replaced when a saved SVT is present. */
void load();

/** Start a replacement. addEcu then commitReplace stores it. */
void beginReplace();

/**
 * One ECU. wire is count * 8 bytes: class, id big-endian, main, sub, patch.
 * @return false when the ECU does not fit.
 */
bool addEcu(uint8_t addr, const char* name, uint8_t ver, uint8_t dep,
            const uint8_t* wire, uint8_t count);

bool commitReplace();

}  // namespace vehicle_svt
