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
// CAN1 — ESP32 native TWAI. This is the K-CAN of the module under test
// (one TJA1050). ENET/DoIP questions for any address other than BDC 0x0010
// are copied onto this pair. Set the speed to that K-CAN: 100, 125, 250 or 500.
// ---------------------------------------------------------------------------
#ifndef PIN_TWAI_TX
#define PIN_TWAI_TX 5
#endif
#ifndef PIN_TWAI_RX
#define PIN_TWAI_RX 4
#endif
#ifndef TWAI_BITRATE_KBPS
#define TWAI_BITRATE_KBPS 500
#endif

// ---------------------------------------------------------------------------
// CAN2 — MCP2515 module (the CAN adapter on the bench). ENET questions go out
// here. HSPI avoids the Ethernet pins 18/19/23. Speed must match the module
// K-CAN: 100, 125, 250 or 500. BATT48 / K-CAN8 is 500.
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
// GPIO16 is the WT32-ETH01 PHY oscillator enable. UART must not take it.
#define PIN_LIN_RX 35
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
// WT32-ETH01 (GERUI): GPIO16 turns the LAN8720 oscillator on. Use -1 if the
// board has no PHY enable pin.
#define ETH_PHY_POWER 16
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

// DoIP well-known port (ISO 13400). Classic BMW ENET (E-Sys / ISTA cable)
// is a different protocol: HSFZ on TCP 6801.
#define DOIP_UDP_DISCOVERY_PORT 13400
#define DOIP_TCP_DATA_PORT      13400
#define ENET_HSFZ_TCP_PORT      6801
#define ENET_HSFZ_UDP_PORT      6811

// ZGW Search and ISTA broadcast vehicle identification to
// 169.254.255.255:6811. The ESP32 has to sit on that same link-local
// network, otherwise the broadcast never arrives.
#define ETH_LOCAL_IP   IPAddress(169, 254, 1, 20)
#define ETH_GATEWAY    IPAddress(0, 0, 0, 0)
#define ETH_SUBNET     IPAddress(255, 255, 0, 0)
// Check-digit-valid bench VIN. Letters I, O and Q are illegal in a VIN;
// ZGW Search drops the reply when the 17 characters after BMWVIN contain one.
#define BENCH_VIN      "WBA00000200000000"

// ---------------------------------------------------------------------------
// FreeRTOS task tuning
// ---------------------------------------------------------------------------
#define TASK_STACK_CAN_CYCLIC   4096
#define TASK_STACK_CAN_RX       4096
#define TASK_STACK_LIN          3072
#define TASK_STACK_DOIP         12288
#define TASK_STACK_PC_LINK      4096

#define TASK_PRIO_CAN_CYCLIC    5   // highest — time-critical wake / ignition
#define TASK_PRIO_CAN_RX        4
#define TASK_PRIO_LIN           3
#define TASK_PRIO_PC_LINK       3
#define TASK_PRIO_DOIP          2

#define TASK_CORE_CAN           1   // pin bus I/O to APP CPU
#define TASK_CORE_NET           0   // pin Ethernet / DoIP to PRO CPU

// Companion PC app UDP port (JSON protocol; DoIP stays on 13400)
#define PC_COMPANION_UDP_PORT   13401

// ---------------------------------------------------------------------------
// Factory BDC diagnostics — CAN OBD (BMW addressed ISO-TP)
// Request on 0x6F1 with data[0]=ecuAddr; response on 0x600|ecuAddr, data[0]=0xF1
// Channel: 0 = TWAI(CAN1), 1 = MCP(CAN2), 2 = Both
// ---------------------------------------------------------------------------
#ifndef OBD_CAN_REQUEST_ID
#define OBD_CAN_REQUEST_ID   0x6F1
#endif
#ifndef OBD_CAN_RESPONSE_ID
#define OBD_CAN_RESPONSE_ID  0x610  // 0x600 + 0x10 (BDC)
#endif
#ifndef OBD_CAN_CHANNEL_SEL
#define OBD_CAN_CHANNEL_SEL  0
#endif

// DoIP → K-CAN gateway. ISTA on ENET targets a logical address; the low byte
// is the BMW ECU address in 0x6F1. Response comes back on 0x600|ecu.
#ifndef KCAN_GW_TIMEOUT_MS
#define KCAN_GW_TIMEOUT_MS 1500
#endif
