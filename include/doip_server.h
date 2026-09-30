/**
 * @file doip_server.h
 * @brief Minimal DoIP (ISO 13400) listener on TCP/UDP port 13400.
 *
 * Bench goal: accept traffic from E-Sys / ISTA and optionally forward raw
 * diagnostic payloads toward the Headunit over CAN (stub hook provided).
 */

#pragma once

#include <Arduino.h>

namespace doip {

/** Bring up LAN8720A Ethernet and bind UDP + TCP on port 13400. */
bool init();

/** FreeRTOS task: handle discovery (UDP) and diagnostic sessions (TCP). */
void serverTask(void* arg);

/**
 * Hook called when a DoIP diagnostic message payload is received.
 * Override / extend in doip_server.cpp to bridge toward HU_MGU via CAN UDS.
 */
void onDiagnosticPayload(const uint8_t* data, size_t len);

}  // namespace doip
