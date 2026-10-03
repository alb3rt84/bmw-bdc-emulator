# BMW G-Chassis BDC / ZGM Bench Emulator (ESP32)

ESP32-based Body Domain Controller / Central Gateway emulator for **on-the-table** testing of BMW G-series modules (Instrument Cluster / KOMBI, Headunit HU_MGU / ENTRYNAV2, LIN climate panels, etc.).

## Features

| Subsystem | Implementation |
|-----------|----------------|
| **CAN1** | ESP32 native TWAI @ 500 kbit/s |
| **CAN2** | MCP2515 (SPI / HSPI) @ 500 kbit/s |
| **Wake / KL15** | FreeRTOS cyclic TX: `0x510`, `0x12F`, `0x34A`, `0x2F8` |
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
| **ENET** | TCP **6801** (HSFZ). Tester `0xF4`, ECU address in the HSFZ target byte | Address `0x10` answered locally. Any other address is copied to the module K-CAN as `0x6F1` and the answer goes back to the tester |
| **DoIP** | LA `0x0010`, TCP/UDP `:13400` | Same UDS handler, plus the same K-CAN forward for other logical addresses |

The module under test sits on the MCP2515 CAN adapter. SPI wiring to the ESP32 Ethernet board: CS GPIO15, SCK GPIO14, MOSI GPIO13, MISO GPIO12, INT GPIO33, common GND. Set `MCP_BITRATE_KBPS` in `include/config.h` to that K-CAN (100, 125, 250 or 500). BATT48 on K-CAN8 is 500. ESP32 address **169.254.1.20**, mask **255.255.0.0**, VIN **WBA00000200000000**. With `HostIdentService = 255.255.255.255` EDIABAS sends six bytes `00 00 00 00 00 11` as a global broadcast on UDP **6811** and waits `TimeoutIdentService` (2 s in the bench ini). `VehicleProtocol = HSFZ,DoIP` keeps whichever answer arrives first, so both announcements use VIN **WBA00000200000000** and gateway address `0x0010`. The ESP32 still owns **169.254.1.20/16** and answers a tester on any other address on that cable. The reply must contain the text `DIAGADR`, `BMWMAC` and `BMWVIN`; the tool takes the IP from the sender of that reply. The laptop Ethernet adapter therefore has to show an address starting with `169.254` (automatic is enough; wait until it appears). The serial monitor should print `[ETH] Link up` and `[ENET] ZGW search listening UDP :6811`. Diagnostics then use TCP **6801**.

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

Python/Tkinter app (`pc_companion/gui_app.py`) controls ignition + RPM/speed/fuel/coolant.

- **Robotell USB-CAN** — PC talks to the CH340 adapter directly (binary protocol, CAN 500 kbit/s) and transmits the cyclic BDC frames. Build `pc_companion\dist\BmwBdcCompanion.exe` with `build_exe.bat`.
- **Serial / UDP :13401** — same signals through the ESP32 firmware.

```bat
cd pc_companion
pip install -r requirements.txt
pyinstaller --noconfirm --onefile --windowed --name BmwBdcCompanion gui_app.py
```

Details: [`pc_companion/README.md`](pc_companion/README.md).

## Cyclic BMW G-Chassis frames (hardcoded)

| ID | Period | Purpose | Default payload |
|----|--------|---------|-----------------|
| `0x510` | 100 ms | OSEK NM (BDC) — keep bus awake | `00 01 00 00 00 00 00 00` |
| `0x12F` | 100 ms | Zustand Klemmen — KL15 / KL30B ON | `45 FF 45 FF FF FF FF FF` |
| `0x34A` | 20 ms | Fahrzustand — alive, stationary | `10 00 00 00 00 00 00 00` |
| `0x2F8` | 1000 ms | Zeit_Datum — silence sync DTCs | `24 0C 0F 0E 00 00 00 FF` |

Inject custom HEX at runtime:

```cpp
uint8_t p[8] = {0x45, 0xFF, 0x45, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bmw::setPayload(0x12F, p, 8);
```

Or edit the table in `src/bmw_frames.cpp`.

## FreeRTOS mapping

- **Core 1:** `bmw_cyclic` (prio 5), `can_rx` (4), `lin_master` (3)
- **Core 0:** `doip` (prio 2)

---

## Wiring guide

> Power KOMBI / HU / LIN slaves from a proper bench PSU (KL30). Share GND with the ESP32.
> Use a CAN transceiver (SN65HVD230 / TJA1050) on TWAI TX/RX — the ESP32 pins are **not** CAN-H/L.

### CAN1 — TWAI + transceiver

| ESP32 | Transceiver | Notes |
|------:|-------------|-------|
| GPIO 5 | TXD | `PIN_TWAI_TX` |
| GPIO 4 | RXD | `PIN_TWAI_RX` |
| 3V3 / 5V | VCC | per transceiver rating |
| GND | GND | common ground |
| — | CANH / CANL | to module bus + **120 Ω** termination if end-node |

### CAN2 — MCP2515 module (HSPI)

| ESP32 | MCP2515 | Notes |
|------:|---------|-------|
| GPIO 15 | CS | |
| GPIO 14 | SCK | HSPI |
| GPIO 12 | MISO | strapping pin — keep LOW at reset |
| GPIO 13 | MOSI | |
| GPIO 33 | INT | optional |
| 3V3 | VCC | most modules are 3V3 logic |
| GND | GND | |
| — | CANH / CANL | second domain bus + termination |

Crystal: code tries **8 MHz** then **16 MHz**.

### LIN Master — TJA1020 / TJA1021

| ESP32 | LIN PHY | Notes |
|------:|---------|-------|
| GPIO 17 | TXD (MCU→PHY) | UART2 TX |
| GPIO 16 | RXD (PHY→MCU) | UART2 RX |
| GPIO 32 | /NSLP | driven HIGH = normal mode |
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
MCP2515 uses HSPI `12–15`, TWAI `4–5`, LIN TX `2`, RX `35`, NSLP `32`. GPIO0 must not be pulled up: that pin is the 50 MHz clock input.

---

## Next steps (stubs to extend)

1. Capture real G-Chassis traces and refine `0x12F` / `0x34A` bitfields.
2. Implement ISO-TP + UDS bridge in `doip::onDiagnosticPayload()` toward HU_MGU.
3. Fill the LIN schedule in `lin_master.cpp` with climate-panel PIDs.
4. Add bus-off recovery / TWAI alerts if a module floods the bus.

## Disclaimer

For **benchtop** module bring-up and diagnostics only. You are responsible for safe power, correct termination, and compliance with local law when working with vehicle electronics.
