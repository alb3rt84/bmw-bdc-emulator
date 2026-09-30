# BMW BDC/ZGM — PC Companion Application

Tkinter GUI that talks to the ESP32 bench emulator over **USB-Serial** or **UDP :13401**.

## Features

- Select COM port or ESP32 Ethernet IP
- **Terminal 15** ignition toggle → updates CAN `0x12F` payload live
- Sliders stream **RPM / speed / fuel / coolant** → ESP32 injects into cyclic CAN TX

## Protocol (JSON lines)

One JSON object per line (`\n` terminated), either on Serial 115200 or UDP port **13401**.

| Command | Example |
|---------|---------|
| Ping / status | `{"cmd":"ping"}` |
| Ignition | `{"cmd":"ign","on":1}` |
| Live signals | `{"cmd":"sig","rpm":1500,"spd":60,"fuel":75,"clt":90}` |

Reply:

```json
{"ok":1,"ign":1,"rpm":1500,"spd":60.0,"fuel":75.0,"clt":90}
```

## Run from source (Windows / macOS / Linux)

```bash
cd pc_companion
python -m venv .venv

# Windows:
.venv\Scripts\activate
# macOS/Linux:
source .venv/bin/activate

pip install -r requirements.txt
python gui_app.py
```

## Build a standalone Windows .EXE (PyInstaller)

On a **Windows** PC (PyInstaller embeds the current OS):

```bat
cd pc_companion
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
pyinstaller --noconfirm --onefile --windowed --name BmwBdcCompanion gui_app.py
```

Output:

```
pc_companion\dist\BmwBdcCompanion.exe
```

Optional console for debugging (shows Serial traffic):

```bat
pyinstaller --noconfirm --onefile --console --name BmwBdcCompanion gui_app.py
```

### One-liner (PowerShell)

```powershell
pip install pyserial pyinstaller
pyinstaller --noconfirm --onefile --windowed --name BmwBdcCompanion gui_app.py
```

## Bench checklist

1. Flash ESP32 firmware (`pio run -t upload`)
2. Connect USB-UART **or** Ethernet (`192.168.0.10` by default)
3. Launch `BmwBdcCompanion.exe` → Connect
4. Toggle ignition / move sliders → watch CAN with a sniffer

Signal CAN IDs / encodings live in `src/bmw_frames.cpp` (search `encodeRpm`).
