/**
 * @file main.cpp
 * @brief BMW G-Chassis BDC / ZGM bench emulator — FreeRTOS entrypoint.
 *
 * Dual-path factory diagnostics:
 *   CAN OBD  — BMW 0x6F1 ISO-TP → uds_bdc
 *   DoIP     — Ethernet :13400  → uds_bdc (same handler)
 */

#include <Arduino.h>

#include "bmw_frames.h"
#include "bdc_config.h"
#include "can_bus.h"
#include "config.h"
#include "doip_server.h"
#include "lin_master.h"
#include "obd_can.h"
#include "pc_link.h"
#include "uds_bdc.h"

namespace {

void canRxTask(void* /*arg*/) {
  Serial.println(F("[CAN] RX + OBD-ISO-TP task started"));
  CanFrame f;
  for (;;) {
    if (canBusReceive(CanChannel::Can1_Twai, f, 5)) {
      obd_can::onCanFrame(f.id, f.data, f.dlc, /*fromCan1=*/true);
    }
    if (canBusReceive(CanChannel::Can2_Mcp, f, 0)) {
      obd_can::onCanFrame(f.id, f.data, f.dlc, /*fromCan1=*/false);
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
  Serial.println(F("Diag: CAN OBD (0x6F1) + DoIP :13400  |  PC JSON :13401"));

  if (!canBusInit()) {
    Serial.println(F("[FATAL] No CAN controller available"));
  }

  bdc_config::init();
  uds_bdc::init();
  obd_can::init();
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

  Serial.println(F("[BOOT] Tasks created — dual-path BDC diagnostics online"));
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
