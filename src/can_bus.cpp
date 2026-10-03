/**
 * @file can_bus.cpp
 * @brief TWAI + MCP2515 dual-CAN drivers (non-blocking).
 */

#include "can_bus.h"
#include "config.h"

#include <SPI.h>
#include <string.h>
#include <driver/gpio.h>
#include <driver/twai.h>
#include <mcp2515.h>

namespace {

MCP2515* g_mcp = nullptr;
bool g_twaiOk = false;
bool g_mcpOk  = false;

SPIClass g_hspi(HSPI);

CAN_SPEED mcpSpeedFromKbps(int kbps) {
  switch (kbps) {
    case 100: return CAN_100KBPS;
    case 125: return CAN_125KBPS;
    case 250: return CAN_250KBPS;
    case 500: return CAN_500KBPS;
    case 1000: return CAN_1000KBPS;
    default: return CAN_500KBPS;
  }
}

twai_timing_config_t twaiTimingFromKbps(int kbps) {
  switch (kbps) {
    case 100: return TWAI_TIMING_CONFIG_100KBITS();
    case 125: return TWAI_TIMING_CONFIG_125KBITS();
    case 250: return TWAI_TIMING_CONFIG_250KBITS();
    case 1000: return TWAI_TIMING_CONFIG_1MBITS();
    default: return TWAI_TIMING_CONFIG_500KBITS();
  }
}

bool initTwai() {
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)PIN_TWAI_TX, (gpio_num_t)PIN_TWAI_RX,
                                  TWAI_MODE_NORMAL);
  // Increase TX queue so cyclic bursts never block the high-prio task
  g_config.tx_queue_len = 32;
  g_config.rx_queue_len = 32;

  twai_timing_config_t t_config = twaiTimingFromKbps(TWAI_BITRATE_KBPS);
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println(F("[CAN1] TWAI driver install failed"));
    return false;
  }
  if (twai_start() != ESP_OK) {
    Serial.println(F("[CAN1] TWAI start failed"));
    return false;
  }
  Serial.printf("[CAN1] TWAI ready @ %d kbit/s (TX=%d RX=%d)\n",
                TWAI_BITRATE_KBPS, PIN_TWAI_TX, PIN_TWAI_RX);
  return true;
}

// Dupont wires between the ETH01 header and a MCP2515 board often fail at
// 10 MHz. 1 MHz then 4 MHz. After RESET, CANSTAT opmode bits are 100.
bool mcpAnswers(uint32_t hz, uint8_t& canstat) {
  SPISettings settings(hz, MSBFIRST, SPI_MODE0);
  g_hspi.beginTransaction(settings);
  digitalWrite(PIN_MCP_CS, LOW);
  g_hspi.transfer(0xC0);
  digitalWrite(PIN_MCP_CS, HIGH);
  g_hspi.endTransaction();
  delay(10);

  g_hspi.beginTransaction(settings);
  digitalWrite(PIN_MCP_CS, LOW);
  g_hspi.transfer(0x03);
  g_hspi.transfer(0x0E);
  canstat = g_hspi.transfer(0x00);
  digitalWrite(PIN_MCP_CS, HIGH);
  g_hspi.endTransaction();
  return (canstat & 0xE0) == 0x80;
}

bool initMcp() {
  pinMode(PIN_MCP_CS, OUTPUT);
  digitalWrite(PIN_MCP_CS, HIGH);
  g_hspi.begin(PIN_MCP_SCK, PIN_MCP_MISO, PIN_MCP_MOSI, PIN_MCP_CS);
  if (PIN_MCP_MISO >= 0 && PIN_MCP_MISO <= 33) {
    gpio_set_pull_mode((gpio_num_t)PIN_MCP_MISO, GPIO_FLOATING);
  }
  delay(50);

  const uint32_t clocks[] = {1000000UL, 4000000UL};
  uint8_t canstat = 0xFF;
  uint32_t hz = clocks[0];
  bool seen = false;
  for (uint32_t clock : clocks) {
    if (mcpAnswers(clock, canstat)) {
      hz = clock;
      seen = true;
      break;
    }
    Serial.printf("[CAN2] MCP2515 silent @ %lu Hz, CANSTAT=0x%02X\n",
                  (unsigned long)clock, canstat);
  }
  if (!seen) {
    Serial.println(F("[CAN2] MCP2515 reset failed — 5V GND, IO15=CS IO14=SCK IO4=MOSI IO35=MISO"));
    return false;
  }

  g_mcp = new MCP2515(PIN_MCP_CS, hz, &g_hspi);
  if (g_mcp->reset() != MCP2515::ERROR_OK) {
    Serial.printf("[CAN2] MCP2515 reset failed, CANSTAT=0x%02X\n", canstat);
    return false;
  }
  const CAN_SPEED speed = mcpSpeedFromKbps(MCP_BITRATE_KBPS);
  CAN_CLOCK crystal = MCP_8MHZ;
  if (g_mcp->setBitrate(speed, MCP_8MHZ) != MCP2515::ERROR_OK) {
    if (g_mcp->setBitrate(speed, MCP_16MHZ) != MCP2515::ERROR_OK) {
      Serial.println(F("[CAN2] MCP2515 setBitrate failed"));
      return false;
    }
    crystal = MCP_16MHZ;
  }
  if (g_mcp->setNormalMode() != MCP2515::ERROR_OK) {
    Serial.println(F("[CAN2] MCP2515 setNormalMode failed"));
    return false;
  }
  Serial.printf("[CAN2] MCP2515 ready @ %d kbit/s, %s crystal, SPI %lu Hz (CS=%d SCK=%d MOSI=%d MISO=%d)\n",
                MCP_BITRATE_KBPS, crystal == MCP_16MHZ ? "16 MHz" : "8 MHz",
                (unsigned long)hz, PIN_MCP_CS, PIN_MCP_SCK, PIN_MCP_MOSI, PIN_MCP_MISO);
  return true;
}

bool sendTwai(const CanFrame& frame) {
  if (!g_twaiOk) return false;
  twai_message_t msg = {};
  msg.identifier       = frame.id;
  msg.data_length_code = frame.dlc;
  msg.extd             = frame.extended ? 1 : 0;
  msg.rtr              = 0;
  memcpy(msg.data, frame.data, frame.dlc > 8 ? 8 : frame.dlc);

  // Non-blocking: timeout 0 → return immediately if TX queue full
  esp_err_t err = twai_transmit(&msg, 0);
  return err == ESP_OK;
}

bool sendMcp(const CanFrame& frame) {
  if (!g_mcpOk || !g_mcp) return false;
  struct can_frame m = {};
  m.can_id  = frame.extended ? (frame.id | CAN_EFF_FLAG) : frame.id;
  m.can_dlc = frame.dlc;
  memcpy(m.data, frame.data, frame.dlc > 8 ? 8 : frame.dlc);
  return g_mcp->sendMessage(&m) == MCP2515::ERROR_OK;
}

bool recvTwai(CanFrame& out, uint32_t timeoutMs) {
  if (!g_twaiOk) return false;
  twai_message_t msg;
  TickType_t ticks = timeoutMs == 0 ? 0 : pdMS_TO_TICKS(timeoutMs);
  if (twai_receive(&msg, ticks) != ESP_OK) return false;
  if (msg.rtr) return false;
  out.id       = msg.identifier;
  out.dlc      = msg.data_length_code;
  out.extended = msg.extd;
  memcpy(out.data, msg.data, out.dlc);
  return true;
}

bool recvMcp(CanFrame& out) {
  if (!g_mcpOk || !g_mcp) return false;
  struct can_frame m;
  if (g_mcp->readMessage(&m) != MCP2515::ERROR_OK) return false;
  out.extended = (m.can_id & CAN_EFF_FLAG) != 0;
  out.id       = m.can_id & (out.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
  out.dlc      = m.can_dlc;
  memcpy(out.data, m.data, out.dlc);
  return true;
}

}  // namespace

bool canBusInit() {
  g_twaiOk = initTwai();
  g_mcpOk  = initMcp();
  // Bench can still run with one bus; report overall status
  if (!g_twaiOk && !g_mcpOk) {
    Serial.println(F("[CAN] Both controllers failed"));
    return false;
  }
  return true;
}

bool canBusSend(CanChannel ch, const CanFrame& frame) {
  bool ok = true;
  if (ch == CanChannel::Can1_Twai || ch == CanChannel::Both) {
    ok = sendTwai(frame) && ok;
  }
  if (ch == CanChannel::Can2_Mcp || ch == CanChannel::Both) {
    ok = sendMcp(frame) && ok;
  }
  return ok;
}

bool canBusReceive(CanChannel ch, CanFrame& out, uint32_t timeoutMs) {
  if (ch == CanChannel::Can1_Twai) {
    return recvTwai(out, timeoutMs);
  }
  if (ch == CanChannel::Can2_Mcp) {
    // MCP2515 path is polled; timeout approximated with a short wait loop
    uint32_t start = millis();
    do {
      if (recvMcp(out)) return true;
      if (timeoutMs == 0) break;
      vTaskDelay(pdMS_TO_TICKS(1));
    } while ((millis() - start) < timeoutMs);
    return false;
  }
  // Both: prefer TWAI with timeout, then MCP poll
  if (recvTwai(out, timeoutMs)) return true;
  return recvMcp(out);
}

void canBusRecover() {
  if (!g_twaiOk) return;
  static uint32_t lastCheck = 0;
  const uint32_t now = millis();
  if ((uint32_t)(now - lastCheck) < 500) return;
  lastCheck = now;

  twai_status_info_t status = {};
  if (twai_get_status_info(&status) != ESP_OK) return;
  if (status.state == TWAI_STATE_BUS_OFF) {
    Serial.println(F("[CAN1] bus-off, recovering"));
    twai_initiate_recovery();
  } else if (status.state == TWAI_STATE_STOPPED) {
    twai_start();
    Serial.println(F("[CAN1] TWAI restarted"));
  }
}

void canBusLogFrame(const char* prefix, CanChannel ch, const CanFrame& f) {
  Serial.printf("%s ch=%u id=0x%03X dlc=%u [", prefix, (unsigned)ch, f.id, f.dlc);
  for (uint8_t i = 0; i < f.dlc; i++) {
    Serial.printf("%02X%s", f.data[i], i + 1 < f.dlc ? " " : "");
  }
  Serial.println("]");
}
