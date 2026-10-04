# ZGW Emulator — okno na PC

Program pokazuje emulator ZGW. Są w nim tylko przełączniki KL30 i KL15 oraz log.

ESP32 wysyła na MCP2515 jedną ramkę, co 100 ms:

| ID | Znaczenie | Oba przełączniki włączone |
|----|-----------|---------------------------|
| `0x12F` | Klemmen, KL30 i KL15 | `45 FF 45 FF FF FF FF FF` |

| KL30 | KL15 | Bajt 0 i 2 |
|------|------|------------|
| wyłączony | wyłączony | `00` |
| włączony | wyłączony | `41` |
| wyłączony | włączony | `44` |
| włączony | włączony | `45` |

Połączenie to UDP na `169.254.1.20:13401` (adres ETH01). W polu VIN wpisuje się pełne 17 znaków auta. Emulator nadaje je w identyfikacji ENET, DoIP i w DID F190 na każdym adresie, więc ISTA nie prosi o VIN drugi raz. Skrót z nagłówka ISTA to ostatnie 7 znaków. Przełączniki KL30 i KL15 ustawiają jeden bajt. Ten bajt jest w ramce CAN `0x12F` i w odpowiedzi ENET na TCP 6811 (`45` gdy oba włączone, `00` gdy oba wyłączone). E-Sys przy połączeniu czyta FA (`22 3F 06`); emulator oddaje zamówienie F15, kwiecień 2018, typ KR23. W logu widać zdarzenia Ethernetu i ramki CAN odebrane przez MCP2515.

```bat
cd pc_companion
python gui_app.py
```

EXE: `build_exe.bat` → `dist\BmwBdcCompanion.exe`.

Polecenie do płytki:

```json
{"cmd":"kl","kl30":1,"kl15":1}
```
