/**
 * @file pc_link.h
 * @brief Companion PC protocol — JSON lines over USB-Serial and UDP :13401.
 */

#pragma once

namespace pc_link {

bool init();
void task(void* arg);

}  // namespace pc_link
