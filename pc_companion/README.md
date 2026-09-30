# BMW BDC/ZGM — PC Companion Application

Tkinter GUI with two diagnostic paths matching the ESP32 firmware:

| Tab | Path | Port | Purpose |
|-----|------|------|---------|
| **Live Control** | JSON Serial / UDP | COM or **:13401** | KL15, RPM, speed, fuel, coolant → cyclic CAN |
| **DoIP UDS** | Factory DoIP | **:13400** | Same UDS BDC as CAN OBD (session, DID, DTC) |

## Features

- Live Control: ignition toggle, signal sliders, idle/drive presets
- DoIP: UDP discover (VIN/LA), TCP routing activation, one-click UDS buttons
- Raw UDS hex entry + decoded responses (VIN, live DID `0100`, …)
- Shared log tab

## Protocol — Live Control (JSON lines)

| Command | Example |
|---------|---------|
| Ping | `{"cmd":"ping"}` |
| Ignition | `{"cmd":"ign","on":1}` |
| Signals | `{"cmd":"sig","rpm":1500,"spd":60,"fuel":75,"clt":90}` |

## Protocol — DoIP

- UDP/TCP `13400`, BDC logical address `0x0010`
- Services: `10` session, `3E` tester present, `22` DID, `14`/`19` DTC

## Run from source

```bash
cd pc_companion
python -m venv .venv
```

### Windows — if PowerShell blocks `Activate.ps1`

**Option A — CMD:**
```bat
cd pc_companion
python -m venv .venv
.venv\Scripts\activate.bat
pip install -r requirements.txt
python gui_app.py
```

**Option B — no activate:**
```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe gui_app.py
```

## Build Windows .EXE

Easiest: double-click / run from **CMD**:

```bat
cd pc_companion
build_exe.bat
```

→ `pc_companion\dist\BmwBdcCompanion.exe`

## Bench checklist

1. Flash ESP32 (dual-path UDS firmware)
2. Live Control: USB or UDP `:13401` → move sliders
3. DoIP: Ethernet to `192.168.0.10` → Discover → Connect → Read VIN / Live
