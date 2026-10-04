# ZGW Emulator — okno na PC

Program pokazuje emulator ZGW. Są w nim przełączniki KL30 i KL15, przebieg, edycja FA i SVT oraz log.

ESP32 wysyła na MCP2515 dwie ramki:

| ID | Znaczenie | Zawartość |
|----|-----------|-----------|
| `0x12F` | Klemmen, KL30 i KL15, co 100 ms | `45 FF 45 FF FF FF FF FF` gdy oba włączone |
| `0x330` | Przebieg, co 1 s | kilometry little-endian w bajtach 0–2 |

| KL30 | KL15 | Bajt 0 i 2 |
|------|------|------------|
| wyłączony | wyłączony | `00` |
| włączony | wyłączony | `41` |
| wyłączony | włączony | `44` |
| włączony | włączony | `45` |

Połączenie to UDP na `169.254.1.20:13401` (adres ETH01). W polu VIN wpisuje się pełne 17 znaków auta. Emulator nadaje je w identyfikacji ENET, DoIP i w DID F190 na każdym adresie, więc ISTA nie prosi o VIN drugi raz. Skrót z nagłówka ISTA to ostatnie 7 znaków. Przełączniki KL30 i KL15 ustawiają jeden bajt. Ten bajt jest w ramce CAN `0x12F` i w odpowiedzi ENET na TCP 6811 (`45` gdy oba włączone, `00` gdy oba wyłączone).

Emulator startuje jako G20 320d, VIN `WBA5V510X0FJ28775`. VCM w BDC to zamówienie FA (`22 3F 06` i zapis `2E 3F 06`; dwa pierwsze bajty to długość, wersja `03` jest na bajcie 2, a odpowiedź ma 843 bajty, bo E-Sys czyta indeks 842): seria `G020`, typ `5V51`, data `1119`, lakier `0C31`, tapicerka `KGNL`. Plik `data/FA.xml` jest tym zamówieniem. Osobnego pliku VCM nie ma. SVT (`data/SVT.xml`) to aktualne sterowniki, 29 adresów od `BDC_GW3` na `0x10`. Ta lista jest wkompilowana i można ją podmienić z okna. Na `22 F1 01` emulator oddaje SVK danego sterownika.

Pole kilometrów i przycisk „Ustaw” zapisują przebieg. Płytka trzyma go i nadaje na CAN `0x330`. „Edytuj FA i SVT” wczytuje XML, pozwala zmienić zamówienie i listę sterowników, zapisuje XML i wysyła całość przyciskiem „Zapisz w emulatorze”. Połączenie oraz „Pobierz z emulatora” biorą z płytki przebieg i FA. Nie wysyłają pliku przy samym połączeniu, więc zapis FA albo VIN z Rheingolda (`2E 3F 06`, `2E F1 90`) zostaje i wraca do okna. Zapis SVT.xml trzyma sterowniki i SGBMID, bez pozostałych atrybutów E-Sys. W logu widać zdarzenia Ethernetu i ramki CAN odebrane przez MCP2515.

```bat
cd pc_companion
python gui_app.py
```

EXE: `build_exe.bat` → `dist\BmwBdcCompanion.exe`.

Polecenie do płytki:

```json
{"cmd":"kl","kl30":1,"kl15":1}
{"cmd":"km","km":123456}
{"cmd":"fa","hex":"0347303230..."}
```
