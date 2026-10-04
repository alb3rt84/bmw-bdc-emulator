/**
 * @file vehicle_fa.h
 * @brief Fahrzeugauftrag served on UDS 22 3F 06, plus I-Stufe from the PC.
 *
 * The blob is FA version 3. I-Stufe is stored and logged. It is not answered
 * on a diagnostic identifier until that identifier is confirmed.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace vehicle_fa {

constexpr size_t kMaxFa = 480;

/** NVS blob, or the built-in F15 bench order when nothing valid is stored. */
void load();

/** Copy the current FA. @return length, or 0 when outMax is too small. */
size_t copy(uint8_t* out, size_t outMax);

/**
 * Replace the FA. The buffer must be version 3 and the counted E / SA / HO
 * fields must consume it exactly.
 */
bool store(const uint8_t* data, size_t len);

/** "F015 KR23 0418" style label, 19 characters plus NUL is enough. */
void summary(char out[20]);

/**
 * I-Stufe strings are 14 characters, "F025-18-03-520", or empty to clear.
 * A null pointer leaves that field unchanged.
 */
bool setIStufe(const char* current, const char* werk, const char* ho);

}  // namespace vehicle_fa
