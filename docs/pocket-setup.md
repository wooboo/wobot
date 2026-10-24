# Pocket i Arduino Alvik — stan integracji

Stan sprawdzenia: 2026-09-07. Nie wgrano firmware ani nie wysłano komend ruchu.

Dodano osobny [wariant MicroPython dla Alvika](../firmware/alvik-controller/README.md)
z tym samym transportem Wi-Fi. Nie wymaga znajomości IP z góry, jeśli
w tej samej podsieci dostępne są ogłoszenia ELRS. Program domyślnie działa
diagnostycznie, bez inicjalizacji napędu.

## Potwierdzone lokalnie

- Repozytorium opisuje TSCINBUNY ZYC0076: UNO/ATmega328P, 74HC595,
  cztery koła mecanum i YD-ESP32-S3 N16R8. To dotychczasowy wariant projektu;
  nowy sterownik Alvika znajduje się w osobnym katalogu.
- USB wielokrotnie zgłosiło `/dev/cu.usbmodem1234561`, producent Arduino,
  produkt Nano ESP32, VID:PID `2341:056B`. Deskryptor nie identyfikuje
  rewizji płyty robota ani wgranego programu.
- `wobot.local` i `elrs_tx.local` nie rozwiązały się w dostępnej sieci.
  W 10-sekundowym pasywnym nasłuchu UDP/11000 nie odebrano pakietów.
  Nie oznacza to, że robot nie działa w innej sieci lub pod innym adresem.
- Przeszło 21 testów Rust, test C++ wyjść silników oraz kompilacja szkicu
  dla UNO (3076 B flash, 209 B RAM). Nie są to testy firmware Alvika.

## Co obsługuje kod Wobot

`firmware/esp32-controller/src/radio.rs` aktywuje moduł ELRS przez
HTTP `POST /udpcontrol` (`joystick_begin`) i odbiera kanały przez UDP/11000
(10 kanałów co 10 ms). `tools/elrs-monitor` obsługuje diagnostykę tego
samego transportu na komputerze, prosząc o 8 kanałów.

ESP32 przelicza kanały i wysyła do UNO własne 14-bajtowe ramki z CRC8
przez UART2, 115200 baud, GPIO17 TX / GPIO18 RX. To nie CRSF, SBUS ani
protokół Alvika. `/api/drive/status` udostępnia stan, nie komendę jazdy.
Nie ustalono, czy ten kod jest uruchomiony na robocie użytkownika.

Nie należy wgrywać szkicu UNO ani obrazu dla YD-ESP32-S3 do Alvika.
W osobnym wariancie Alvika sterownik czterech kół i UART Wobot zastąpiono
adapterem oficjalnej [arduino-alvik-mpy](https://github.com/arduino/arduino-alvik-mpy).
Dotychczasowy wariant ZYC0076 pozostał bez zmian.

## Realne warianty sterowania — propozycje, nie wykonane połączenia

1. **Pocket jako joystick USB → komputer → istniejący interfejs Wi-Fi robota.**
   Może zachować działający program robota. Wymaga poznania adresu i formatu
   komend oraz adaptera joystick → ten protokół. RadioMaster dokumentuje
   obsługę symulatorów przez USB w [instrukcji Pocket na stronie produktu](https://www.radiomasterrc.com/products/pocket-radio-controller-m2).
2. **Pocket ELRS Wi-Fi Joystick → UDP → program na Nano ESP32 → biblioteka
   Alvika.** To wariant najbardziej zbliżony do kodu Wobot, ale wymaga
   zgodnego firmware modułu Pocket, konfiguracji sieci i nowego adaptera
   napędu. Funkcję UDP/Wi-Fi Joystick dodano w
   [ExpressLRS 3.4.0](https://github.com/ExpressLRS/ExpressLRS/releases/tag/3.4.0).
   Nie potwierdzono jej dostępności ani aktywacji na tym egzemplarzu Pocket.
3. **Pocket ELRS przez RF → osobny odbiornik ELRS → CRSF UART → Alvik.**
   Wymaga dodatkowego odbiornika i sprawdzenia wolnych pinów oraz napięć.
   Sam ESP32 nie jest odbiornikiem RF ELRS. Sposób wyjścia odbiornika opisuje
   [ExpressLRS](https://www.expresslrs.org/quick-start/receivers/wiring-up/).

Bluetooth LE w Alviku i tryb
[BLE Joystick ExpressLRS](https://www.expresslrs.org/quick-start/transmitters/lua-howto/)
nie oznaczają gotowego parowania: potrzebna byłaby implementacja klienta
joysticka BLE i sprawdzenie raportów HID. Nie potwierdzono takiej integracji.

Każdy adapter musi obsłużyć ARM/DISARM, neutralne osie przy uzbrajaniu,
świeżość danych i zatrzymanie po utracie łączności. Sam most na komputerze
nie zapewnia zatrzymania po zerwaniu Wi-Fi — potrzebny jest timeout w robocie.
W aktualnym Wobot dane radia wygasają po 1 s, a watchdog UNO po 150 ms braku
ramek UART; nie wolno przypisywać tych parametrów fabrycznemu Alvikowi.

## Dane potrzebne przy uruchomieniu

Nazwa sieci i hasło wpisane lokalnie do konfiguracji Alvika oraz włączony
tryb Wi-Fi modułu Pocket w tej sieci. Puste pole IP pozwala wykryć ogłoszenia
ELRS; potrzebny jest ręczny adres, jeśli sieć nie przepuszcza broadcastu.
Przed kopiowaniem programu trzeba sprawdzić, czy Nano ma środowisko
MicroPython i bibliotekę Alvika; nie potwierdzono tego przez USB.
Dane sprzętowe fabrycznego Alvika zebrano w [alvik-hardware.md](alvik-hardware.md).
