# BMW G-Chassis BDC / ZGM Bench Emulator (ESP32)

ESP32-based Body Domain Controller / Central Gateway emulator for **on-the-table** testing of BMW G-series modules (Instrument Cluster / KOMBI, Headunit HU_MGU / ENTRYNAV2, LIN climate panels, etc.).

## Features

| Subsystem | Implementation |
|-----------|----------------|
| **CAN1** | ESP32 TWAI @ 500 kbit/s — cyclic wake frames |
| **CAN2** | MCP2515 @ 500 kbit/s — ENET converter to the module |
| **KL30 / KL15** | MCP2515 cyclic `0x12F` every 100 ms. KL15 is also the ENET ignition byte on TCP **6811** |
| **Live signals** | RPM `0x0A5`, Speed `0x1A1`, Coolant `0x1D0`, Fuel `0x349` (editable) |
| **LIN Master** | UART2 @ 19200 + break/header scheduler (TJA1020) |
| **DoIP** | LAN8720A Ethernet, TCP/UDP port **13400** → shared UDS BDC |
| **CAN OBD** | BMW `0x6F1` / `0x610` ISO-TP → **same** UDS BDC handler |
| **PC Companion** | JSON over USB-Serial or UDP **:13401** — see `pc_companion/` |

No `delay()` in bus tasks — only `vTaskDelay()` yields.

## Project layout

```
include/
  config.h         # pins, baud, task priorities / core affinity
  can_bus.h
  bmw_frames.h     # G-Chassis frame table API
  lin_master.h
  doip_server.h
src/
  main.cpp         # FreeRTOS task spawn
  can_bus.cpp
  bmw_frames.cpp   # <-- edit HEX payloads here
  lin_master.cpp
  doip_server.cpp
platformio.ini
```

## Build & flash

```bash
pio run -t upload
pio device monitor -b 115200
```

## Dual-path BDC diagnostics (factory-style)

One UDS server (`uds_bdc`) answers on both media:

| Path | Addressing | Services (initial) |
|------|------------|--------------------|
| **CAN OBD** | BMW ISO-TP: req `0x6F1` + `ecu=0x10`, resp `0x610` | `0x10` session, `0x3E` tester present, `0x22` DID (`F190` VIN, `F186` session, `F18C` SN, `0100` live signals), `0x14`/`0x19` DTC stubs |
| **ENET** | TCP **6801** diagnostics, TCP **6811** ignition. Tester `0xF4` | Address `0x10` and every other address in the SVT answered locally. Functional `0xDF` is one HSFZ reply per fitted ECU, each with that ECU as the source. UDS `22 F1 90` (VIN) and `22 F1 01` (SVK) are among those replies. An address that is not in the SVT is copied to the module K-CAN as `0x6F1` |
| **DoIP** | LA `0x0010`, TCP/UDP `:13400` | Same UDS handler, plus the same K-CAN forward for other logical addresses |

The module under test sits on the MCP2515 plugged into the WT32-ETH01 header. Silkscreen: **IO15 = CS, IO14 = SCK, IO4 = MOSI, IO35 = MISO**, VCC on **5V**, common **GND**. INT stays open. ISTA addresses other than `0x10` are copied to that bus as `0x6F1` and the answer on `0x600|ecu` goes back over ENET. BATT48 on K-CAN8 is 500. ESP32 address **169.254.1.20**, mask **255.255.0.0**, VIN **WBA00000200000000**. With `HostIdentService = 255.255.255.255` EDIABAS sends six bytes `00 00 00 00 00 11` as a global broadcast on UDP **6811** and waits `TimeoutIdentService` (2 s in the bench ini). `VehicleProtocol = HSFZ,DoIP` keeps whichever answer arrives first, so both announcements use VIN **WBA00000200000000** and gateway address `0x0010`. The ESP32 still owns **169.254.1.20/16** and answers a tester on any other address on that cable. The reply must contain the text `DIAGADR`, `BMWMAC` and `BMWVIN`; the tool takes the IP from the sender of that reply. The laptop Ethernet adapter therefore has to show an address starting with `169.254` (automatic is enough; wait until it appears). The serial monitor should print `[ETH] Link up` and `[ENET] ZGW search listening UDP :6811`. Diagnostics then use TCP **6801**.

Example CAN Single-Frame TesterPresent:
```
ID 0x6F1  data: 10 02 3E 00 00 00 00 00
              ^^ecu ^^ISO-TP SF len=2  ^^UDS
```
Response:
```
ID 0x610  data: F1 02 7E 00 ...
```

## PC Companion GUI

Python/Tkinter window (`pc_companion/gui_app.py`) talks to the ZGW at `169.254.1.20:13401`. It has KL30 and KL15 switches and a log of Ethernet events plus CAN frames received by the MCP2515.

```bat
cd pc_companion
pip install -r requirements.txt
pyinstaller --noconfirm --onefile --windowed --name BmwBdcCompanion --add-data "data;data" gui_app.py
```

Details: [`pc_companion/README.md`](pc_companion/README.md).

## Cyclic frame on the MCP2515

| ID | Period | Purpose | Both switches on |
|----|--------|---------|------------------|
| `0x12F` | 100 ms | Klemmen KL30 + KL15 | `45 FF 45 FF FF FF FF FF` |
| `0x330` | 1 s | Odometer, km little-endian in bytes 0–2 | `km` |

The PC window (`pc_companion/gui_app.py`) switches KL30 and KL15. KL30 only is byte `41`, KL15 only is byte `44`, both off is byte `00` (bytes 0 and 2). ISTA reads ignition on ENET TCP **6811** with `00 00 00 00 00 10` and expects `00 00 00 01 00 10` plus `04` (KL15 on) or `00` (KL15 off). The same answer is given if that control word arrives on TCP 6801. The KL30 switch changes CAN `0x12F`. The VIN typed in the window is the one in the ENET/DoIP announcement and in UDS F190 for every ECU address. It has to be the full 17-character VIN. The baked-in vehicle is a G20 320d, VIN `WBA5V510X0FJ28775`. E-Sys `requestFaFromMaster` reads FA with `22 3F 06` on VCM `0x10`. Bytes 0–1 are the order length, version 3 is byte 2, and the time criterion `1119` follows the series and type key. Bytes after the order repeat `1119` through index 842. A stored order from an older build is ignored, so the bench starts from `data/FA.xml` again. Rheingold writes that order with `2E 3F 06` and the VIN with `2E F1 90`; both stay in NVS and show up in the PC window. That VCM record starts as the order in `data/FA.xml` (series `G020`, type `5V51`, time `1119`). There is no separate VCM file. `data/SVT.xml` is the fitted controllers (29 addresses, `BDC_GW3` at `0x10`), compiled in and replaceable from the editor. `22 F1 01` returns that ECU's SVK. ISTA builds the control-unit tree by sending that read (and the fault-memory read) to functional address `0xDF`. The emulator answers once for each SVT address, and the HSFZ source byte is that address (`0x10` for `BDC_GW3`, then the rest). Address `0xDF` sends nothing, so it does not appear as a control unit that failed to respond. FA `22 3F 06` is still only the VCM at `0x10`. DoIP functional address `0xE400` uses the same list. The kilometre field is stored and sent as CAN `0x330` bytes 0–2, little-endian, every 1 s. Connecting pulls the stored kilometres and FA. The clamp byte on ENET TCP 6811 is the same byte as CAN `0x12F`.

## FreeRTOS mapping

- **Core 1:** `bmw_cyclic` (prio 5), `can_rx` (4), `lin_master` (3)
- **Core 0:** `doip` (prio 2)

---

## Wiring guide

> Power KOMBI / HU / LIN slaves from a proper bench PSU (KL30). Share GND with the ESP32.
> Use a CAN transceiver (SN65HVD230 / TJA1050) on TWAI TX/RX — the ESP32 pins are **not** CAN-H/L.

### CAN1 — TWAI + transceiver

Cyclic wake frames only. The module under test uses the MCP2515 below.

| ESP32 | Transceiver | Notes |
|------:|-------------|-------|
| RXD (IO5) | TXD | `PIN_TWAI_TX` |
| 485_EN (IO33) | RXD | `PIN_TWAI_RX` |
| 3V3 / 5V | VCC | per transceiver rating |
| GND | GND | common ground |
| — | CANH / CANL | optional second bus |

### CAN2 — MCP2515, ENET converter

ISTA on the ENET cable, module on this board's CANH/CANL. SPI stays off GPIO 18, 19 and 23 (those belong to the LAN8720).

The connector is labeled IO15, IO14, IO4 and IO35. There is no IO13 on this board. Leave IO12 empty.

| Silkscreen | MCP2515 | Notes |
|------------|---------|-------|
| IO15 | CS | |
| IO14 | SCK | |
| IO4 | MOSI / SI | |
| IO35 | MISO / SO | input only on the ESP32 |
| 5V | VCC | TJA1050 modules need 5 V |
| GND | GND | common with the module |
| — | CANH / CANL | module K-CAN + **120 Ω** if this node is at the end |

INT of the MCP2515 stays unconnected.

SPI starts at 1 MHz. Crystal: code tries **8 MHz** then **16 MHz**. A live chip prints `[CAN2] MCP2515 ready`.

### LIN Master — TJA1020 / TJA1021

| ESP32 | LIN PHY | Notes |
|------:|---------|-------|
| IO2 | TXD (MCU→PHY) | UART2 TX |
| IO39 | RXD (PHY→MCU) | UART2 RX, input only |
| CFG (IO32) | /NSLP | driven HIGH = normal mode |
| — | LIN bus | single-wire to slaves |
| 12 V | VS / INH rails | per PHY datasheet |
| GND | GND | |

### Ethernet — LAN8720A (RMII)

| Function | ESP32 GPIO |
|----------|------------|
| REF_CLK | GPIO 0 (50 MHz **in** from PHY) |
| MDIO | GPIO 18 |
| MDC | GPIO 23 |
| TX_EN | GPIO 21 |
| TXD0 | GPIO 19 |
| TXD1 | GPIO 22 |
| RXD0 | GPIO 25 |
| RXD1 | GPIO 26 |
| CRS_DV | GPIO 27 |
| PHY power (opt.) | set `ETH_PHY_POWER` in `config.h` |

Static IP default: **169.254.1.20**, mask **255.255.0.0** (change in `include/config.h`). ZGW Search only sees a gateway that answers the UDP 6811 broadcast on this link-local network.  
ENET (E-Sys / ISTA cable): **TCP port 6801**, HSFZ. DoIP stays on **UDP + TCP port 13400**.

### Pin conflict summary

LAN8720A owns GPIOs `0, 18, 19, 21, 22, 23, 25, 26, 27`. GPIO16 only enables the 50 MHz oscillator and must stay high.  
MCP2515 uses silkscreen IO15, IO14, IO4 and IO35. Leave IO12 empty. TWAI uses RXD (IO5) and 485_EN (IO33). LIN TX is IO2, RX is IO39, NSLP is CFG (IO32). GPIO0 must not be pulled up: that pin is the 50 MHz clock input.

---

## Next steps (stubs to extend)

1. Capture real G-Chassis traces and refine `0x12F` / `0x34A` bitfields.
2. Implement ISO-TP + UDS bridge in `doip::onDiagnosticPayload()` toward HU_MGU.
3. Fill the LIN schedule in `lin_master.cpp` with climate-panel PIDs.
4. Add bus-off recovery / TWAI alerts if a module floods the bus.

## Disclaimer

For **benchtop** module bring-up and diagnostics only. You are responsible for safe power, correct termination, and compliance with local law when working with vehicle electronics.
