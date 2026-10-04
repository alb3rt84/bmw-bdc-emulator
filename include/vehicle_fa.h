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
// Largest 22 3F 06 body we accept on a write, including a wrapper.
constexpr size_t kPsdzFaBytes = 842;

/** NVS blob, or the built-in F15 bench order when nothing valid is stored. */
void load();

/** Copy the current FA. @return length, or 0 when outMax is too small. */
size_t copy(uint8_t* out, size_t outMax);

/**
 * 22 3F 06 body. E-Sys reads a big-endian length from bytes 0–1 and copies
 * that many bytes starting at byte 3, then reads the following byte.
 * Byte 2 is the FA version. @return wrapped length, or 0 when outMax is too small.
 */
size_t copyWrapped(uint8_t* out, size_t outMax);

/**
 * Replace the FA. Accepts the bare version-3 order, or the wrapped form
 * answered on 22 3F 06. The stored bytes are the bare order.
 */
bool store(const uint8_t* data, size_t len);

/** "G020 5V51 1119" style label, 19 characters plus NUL is enough. */
void summary(char out[20]);

/**
 * I-Stufe strings are 14 characters, "F025-18-03-520", or empty to clear.
 * A null pointer leaves that field unchanged.
 */
bool setIStufe(const char* current, const char* werk, const char* ho);

}  // namespace vehicle_fa
