/**
 * @file config.h
 * @brief Pin map, baud rates, and task priorities for the BMW BDC/ZGM emulator.
 *
 * IMPORTANT — LAN8720A RMII uses fixed ESP32 GPIOs:
 *   0 (CLK in), 18 (MDIO), 19 (TXD0), 21 (TX_EN), 22 (TXD1),
 *   23 (MDC), 25 (RXD0), 26 (RXD1), 27 (CRS_DV)
 * Those pins must NOT be reused for TWAI / MCP2515 / LIN.
 *
 * Defaults below keep buses on free GPIOs. Change only if your carrier differs.
 */

#pragma once

#include <Arduino.h>
#include <IPAddress.h>

// ---------------------------------------------------------------------------
// CAN1 — ESP32 native TWAI (PT-CAN / K-CAN @ 500 kbit/s)
// ---------------------------------------------------------------------------
#ifndef PIN_TWAI_TX
#define PIN_TWAI_TX 5
#endif
#ifndef PIN_TWAI_RX
#define PIN_TWAI_RX 4
#endif
#define TWAI_BITRATE_KBPS 500

// ---------------------------------------------------------------------------
// CAN2 — MCP2515 on HSPI (second domain bus @ 500 kbit/s)
// Avoids conflict with Ethernet MDIO/MDC/TXD0 on VSPI pins 18/19/23.
// ---------------------------------------------------------------------------
#ifndef PIN_MCP_CS
#define PIN_MCP_CS 15
#endif
#ifndef PIN_MCP_INT
#define PIN_MCP_INT 33   // optional interrupt; set -1 to poll only
#endif
#ifndef PIN_MCP_SCK
#define PIN_MCP_SCK 14
#endif
#ifndef PIN_MCP_MISO
#define PIN_MCP_MISO 12
#endif
#ifndef PIN_MCP_MOSI
#define PIN_MCP_MOSI 13
#endif
#define MCP_BITRATE_KBPS 500

// ---------------------------------------------------------------------------
// LIN Master — UART2 + TJA1020 (or TJA1021) transceiver
// ---------------------------------------------------------------------------
#ifndef PIN_LIN_TX
#define PIN_LIN_TX 17
#endif
#ifndef PIN_LIN_RX
#define PIN_LIN_RX 16
#endif
#ifndef PIN_LIN_NSLP
#define PIN_LIN_NSLP 32  // /NSLP HIGH = normal mode
#endif
#define LIN_UART_NUM 2
#define LIN_BAUD 19200

// ---------------------------------------------------------------------------
// Ethernet — LAN8720A RMII
// ---------------------------------------------------------------------------
#ifndef ETH_PHY_TYPE
#define ETH_PHY_TYPE ETH_PHY_LAN8720
#endif
#ifndef ETH_PHY_ADDR
#define ETH_PHY_ADDR 1
#endif
#ifndef ETH_PHY_POWER
#define ETH_PHY_POWER -1  // set to a GPIO if your module has a PHY enable pin
#endif
#ifndef ETH_PHY_MDC
#define ETH_PHY_MDC 23
#endif
#ifndef ETH_PHY_MDIO
#define ETH_PHY_MDIO 18
#endif
#ifndef ETH_CLK_MODE
#define ETH_CLK_MODE ETH_CLOCK_GPIO0_IN
#endif

// DoIP well-known port (ISO 13400)
#define DOIP_UDP_DISCOVERY_PORT 13400
#define DOIP_TCP_DATA_PORT      13400

// Static IP for bench (match your laptop subnet / E-Sys interface)
#define ETH_LOCAL_IP   IPAddress(192, 168, 0, 10)
#define ETH_GATEWAY    IPAddress(192, 168, 0, 1)
#define ETH_SUBNET     IPAddress(255, 255, 255, 0)

// ---------------------------------------------------------------------------
// FreeRTOS task tuning
// ---------------------------------------------------------------------------
#define TASK_STACK_CAN_CYCLIC   4096
#define TASK_STACK_CAN_RX       4096
#define TASK_STACK_LIN          3072
#define TASK_STACK_DOIP         8192

#define TASK_PRIO_CAN_CYCLIC    5   // highest — time-critical wake / ignition
#define TASK_PRIO_CAN_RX        4
#define TASK_PRIO_LIN           3
#define TASK_PRIO_DOIP          2

#define TASK_CORE_CAN           1   // pin bus I/O to APP CPU
#define TASK_CORE_NET           0   // pin Ethernet / DoIP to PRO CPU
