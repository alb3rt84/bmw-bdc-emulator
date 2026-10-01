# BMW BDC/ZGM — PC Companion Application

Tkinter GUI with two diagnostic paths matching the ESP32 firmware:

| Tab | Path | Port | Purpose |
|-----|------|------|---------|
| **Live Control** | **Robotell USB-CAN** | COM (CH340) | PC sam nadaje ramki BDC na magistralę CAN |
| **Live Control** | JSON Serial / UDP | COM or **:13401** | To samo, ale przez firmware ESP32 |
| **DoIP UDS** | Factory DoIP | **:13400** | Same UDS BDC as CAN OBD (session, DID, DTC) |

## Adapter Robotell — program EXE (bez Pythona przy uruchamianiu)

Adapter Robotell to układ **CH340 + STM32**. Mówi własnym protokołem binarnym, nie tekstem JSON i nie SLCAN. Dlatego zwykły port COM w starym trybie „Serial” nie nawiązywał połączenia.

Gotowy program to `BmwBdcCompanion.exe`. Pythona potrzebujesz tylko raz, żeby go zbudować:

```bat
cd pc_companion
build_exe.bat
```

Plik: `pc_companion\dist\BmwBdcCompanion.exe`

### Połączenie

1. Zainstaluj sterownik **CH340** (w Menadżerze urządzeń ma być port COM, często `USB-SERIAL CH340`, VID `1A86`).
2. Zamknij program producenta **EmbededDebug** — trzyma port i Windows zgłasza „access denied”.
3. Uruchom exe → zakładka **Transmit** → **Robotell**.
4. **Odśwież**, wybierz port z opisem CH340.
5. CAN **500000** (BMW). Zostaw **Auto baud** (sprawdza 115200, potem 2000000 i pozostałe).
6. **Connect**. Status ma pokazać `Robotell COMx USB … CAN 500000`. Dopiero wtedy USB naprawdę odpowiada.
7. Ramka jest jak w CANhackerze: **ID**, **DLC 0–8**, bajty **D0–D7**, **Period** w ms. Bajty powyżej DLC są szare. **Dodaj** zapisuje DLC w tabeli nadawania — to ta DLC idzie na magistralę. **Start** włącza cykliczne nadawanie, **Stop** je zatrzymuje. **Wyślij raz** wysyła jedną ramkę od razu.
8. **Zapisz** zapisuje listę do pliku tekstowego (`12F 3 100 STD DATA AA BB CC`). **Wczytaj** wczytuje taki plik z powrotem.
9. Odebrane ramki są w osobnym oknie **Receive** (przycisk Receive otwiera je ponownie). Suwaki KL15, RPM, km/h, paliwo i temperatura podmieniają dane w wierszach o tym samym ID i nie zmieniają ich DLC.

Jeśli Connect kończy się komunikatem, że port się otwiera, ale adapter nie odpowiada — to nie jest ten COM albo prędkość USB jest nietypowa. Zostaw Auto baud i wybierz port CH340.

CANH/CANL potrzebują drugiego węzła i terminacji 120 Ω. Samo USB może być połączone, a ramki i tak nie wyjdą na pustą magistralę (brak ACK).

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
