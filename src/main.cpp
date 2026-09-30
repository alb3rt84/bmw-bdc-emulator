/**
 * @file main.cpp
 * @brief BMW G-Chassis BDC / ZGM bench emulator — FreeRTOS entrypoint.
 *
 * Architecture
 * ------------
 *  Core 1 (APP):  high-priority cyclic CAN TX (wake + KL15 + vehicle state)
 *                 + CAN RX monitor + LIN master scheduler
 *  Core 0 (PRO):  Ethernet / DoIP TCP+UDP :13400
 *
 * No delay() is used in bus tasks — only vTaskDelay() for cooperative yields.
 */

#include <Arduino.h>

#include "bmw_frames.h"
#include "can_bus.h"
#include "config.h"
#include "doip_server.h"
#include "lin_master.h"

namespace {

void canRxTask(void* /*arg*/) {
  Serial.println(F("[CAN] RX monitor task started"));
  CanFrame f;
  for (;;) {
    // Poll both buses with short timeouts (non-blocking overall)
    if (canBusReceive(CanChannel::Can1_Twai, f, 5)) {
      // Uncomment for verbose sniffing:
      // canBusLogFrame("[CAN1 RX]", CanChannel::Can1_Twai, f);
      (void)f;
    }
    if (canBusReceive(CanChannel::Can2_Mcp, f, 0)) {
      // canBusLogFrame("[CAN2 RX]", CanChannel::Can2_Mcp, f);
      (void)f;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);  // only during boot for USB-CDC settle — not used in tasks
  Serial.println();
  Serial.println(F("=== BMW G-Chassis BDC/ZGM Bench Emulator ==="));
  Serial.println(F("CAN1=TWAI  CAN2=MCP2515  LIN=UART2  DoIP=:13400"));

  if (!canBusInit()) {
    Serial.println(F("[FATAL] No CAN controller available"));
  }

  lin::init();
  doip::init();

  // --- FreeRTOS tasks -------------------------------------------------------
  // Time-critical ignition / NM / Fahrzustand on Core 1
  xTaskCreatePinnedToCore(
      bmw::cyclicTxTask, "bmw_cyclic", TASK_STACK_CAN_CYCLIC, nullptr,
      TASK_PRIO_CAN_CYCLIC, nullptr, TASK_CORE_CAN);

  xTaskCreatePinnedToCore(
      canRxTask, "can_rx", TASK_STACK_CAN_RX, nullptr,
      TASK_PRIO_CAN_RX, nullptr, TASK_CORE_CAN);

  xTaskCreatePinnedToCore(
      lin::masterTask, "lin_master", TASK_STACK_LIN, nullptr,
      TASK_PRIO_LIN, nullptr, TASK_CORE_CAN);

  // DoIP / Ethernet on Core 0 so socket traffic cannot starve CAN
  xTaskCreatePinnedToCore(
      doip::serverTask, "doip", TASK_STACK_DOIP, nullptr,
      TASK_PRIO_DOIP, nullptr, TASK_CORE_NET);

  Serial.println(F("[BOOT] Tasks created — emulator running"));
}

void loop() {
  // All work is in FreeRTOS tasks. Idle here with a long yield so loopTask
  // does not burn CPU.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
