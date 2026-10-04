/**
 * @file bench_vin.h
 * @brief VIN the ZGW emulator announces on ENET, DoIP and UDS F190.
 */

#pragma once

namespace bench_vin {

/** Load the last VIN from NVS, or the compile-time bench VIN. */
void load();

/** 17 characters plus a terminating NUL. */
void copy(char out[18]);

/**
 * Accept a 17-character VIN (letters I, O and Q are illegal).
 * Stores it so the next identification uses it.
 * @return false when the text is not a VIN.
 */
bool set(const char* vin);

}  // namespace bench_vin
