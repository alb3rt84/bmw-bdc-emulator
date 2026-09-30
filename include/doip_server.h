/**
 * @file doip_server.h
 * @brief DoIP (ISO 13400) listener — discovery + diagnostic sessions to uds_bdc.
 */

#pragma once

#include <Arduino.h>

namespace doip {

bool init();
void serverTask(void* arg);

}  // namespace doip
