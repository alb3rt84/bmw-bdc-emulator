/**
 * @file main.cpp
 * @brief BMW G-Chassis BDC / ZGM bench emulator — FreeRTOS entrypoint.
 */

#include <Arduino.h>

#include "bmw_frames.h"
#include "can_bus.h"
#include "config.h"
#include "doip_server.h"
#include "lin_master.h"
#include "pc_link.h"

namespace {

void canRxTask(void* /*arg*/) {
  Serial.println(F("[CAN] RX monitor task started"));
  CanFrame f;
  for (;;) {
    if (canBusReceive(CanChannel::Can1_Twai, f, 5)) {
      (void)f;
    }
    if (canBusReceive(CanChannel::Can2_Mcp, f, 0)) {
      (void)f;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println(F("=== BMW G-Chassis BDC/ZGM Bench Emulator ==="));
  Serial.println(F("CAN1=TWAI  CAN2=MCP2515  LIN=UART2  DoIP=:13400  PC=:13401"));

  if (!canBusInit()) {
    Serial.println(F("[FATAL] No CAN controller available"));
  }

  lin::init();
  doip::init();
  pc_link::init();

  xTaskCreatePinnedToCore(
      bmw::cyclicTxTask, "bmw_cyclic", TASK_STACK_CAN_CYCLIC, nullptr,
      TASK_PRIO_CAN_CYCLIC, nullptr, TASK_CORE_CAN);

  xTaskCreatePinnedToCore(
      canRxTask, "can_rx", TASK_STACK_CAN_RX, nullptr,
      TASK_PRIO_CAN_RX, nullptr, TASK_CORE_CAN);

  xTaskCreatePinnedToCore(
      lin::masterTask, "lin_master", TASK_STACK_LIN, nullptr,
      TASK_PRIO_LIN, nullptr, TASK_CORE_CAN);

  xTaskCreatePinnedToCore(
      pc_link::task, "pc_link", TASK_STACK_PC_LINK, nullptr,
      TASK_PRIO_PC_LINK, nullptr, TASK_CORE_NET);

  xTaskCreatePinnedToCore(
      doip::serverTask, "doip", TASK_STACK_DOIP, nullptr,
      TASK_PRIO_DOIP, nullptr, TASK_CORE_NET);

  Serial.println(F("[BOOT] Tasks created — emulator running"));
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
