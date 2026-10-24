# Arduino Alvik — ustalenia sprzętowe

## Potwierdzone w źródłach producenta

- Fabryczny Alvik ma dwa mikrokontrolery. Głównym kontrolerem jest **Arduino Nano ESP32**; układ pomocniczy **STM32F411RC (Arm Cortex-M4, 32-bit)** steruje sensorami i aktuatorami. Nano ESP32 komunikuje się ze STM32 przez dedykowane API. Źródło: [Arduino Alvik User Manual, sekcje 1.3 i 2.1](https://docs.arduino.cc/resources/datasheets/AKX00066-datasheet.pdf) (s. 4, 7–8); [strona Arduino Alvik](https://docs.arduino.cc/hardware/alvik).
- Nano ESP32 użyty przez Alvik zawiera moduł radiowy **u-blox NORA-W106-10B**, w którym znajduje się SoC **Espressif ESP32-S3** i wbudowana antena. Źródło: [Arduino Nano ESP32 — dokumentacja](https://docs.arduino.cc/hardware/nano-esp32); [Nano ESP32 datasheet](https://docs.arduino.cc/resources/datasheets/ABX00083-datasheet.pdf) (sekcja 6).
- Moduł NORA-W106-10B obsługuje **Wi‑Fi 4, IEEE 802.11 b/g/n**. Dokumentacja Espressif określa ESP32-S3 jako układ z radiem Wi‑Fi w **paśmie 2,4 GHz**. Zatem dla fabrycznego kontrolera Alvik potwierdzone jest Wi‑Fi 2,4 GHz (bez potwierdzenia obsługi 5 GHz). Źródła: [Nano ESP32 datasheet](https://docs.arduino.cc/resources/datasheets/ABX00083-datasheet.pdf) (sekcja 6.2); [Espressif ESP32-S3 Product Overview](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32s3/product-overview.html).
- NORA-W106-10B obsługuje **Bluetooth Low Energy 5.0** (do 2 Mb/s, tryb central/peripheral i wiele połączeń). Oficjalna strona Nano ESP32 potwierdza także obsługę Bluetooth i zastosowania BLE. Źródła: [Nano ESP32 datasheet](https://docs.arduino.cc/resources/datasheets/ABX00083-datasheet.pdf) (sekcja 6.3); [Arduino Nano ESP32 — dokumentacja](https://docs.arduino.cc/hardware/nano-esp32).

## Wniosek dotyczący badanego USB

- Zgłoszenie USB **VID 2341 / PID 056B** jest zgodne z identyfikacją Arduino Nano ESP32 w obserwowanym urządzeniu, ale samo VID/PID nie dowodzi konkretnej rewizji płytki ani nie rozstrzyga, czy identyfikator pochodzi z modułu zamontowanego w danym Alviku. Traktuję więc Nano ESP32/NORA-W106-10B, STM32F411RC oraz parametry radiowe powyżej jako specyfikację fabrycznego Alvika; identyfikację konkretnego egzemplarza pozostawiam jako **niepotwierdzoną** bez dodatkowego odczytu oznaczeń, schematu lub firmware.

