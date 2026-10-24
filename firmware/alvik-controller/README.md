# Arduino Alvik — Pocket przez Wi-Fi

Osobny program MicroPython dla fabrycznego Nano ESP32 w Alviku. Nie zmienia
firmware Rust, NVS, panelu WWW ani szkiców UNO istniejącego ZYC0076.

```text
Pocket ELRS: Wi-Fi Joystick
  → istniejąca sieć Wi-Fi, UDP/11000
  → Nano ESP32: alvik_main.py
  → oficjalna biblioteka arduino_alvik → STM32 Alvika → dwa koła
```

To program przygotowany i testowany lokalnie, jeszcze nie zweryfikowany na
robocie. Nie wymaga osobnego odbiornika ELRS ani dodatkowego UART na kablach.
Wykorzystuje to samo HTTP `/udpcontrol`, 100 Hz i format kanałów UDP co
`../esp32-controller/src/radio.rs`. Wersja firmware Pocket musi obsługiwać
Wi-Fi Joystick — funkcja jest opisana w
[ExpressLRS 3.4.0](https://github.com/ExpressLRS/ExpressLRS/releases/tag/3.4.0).
Zwykłe Wi-Fi robota i zwykły tryb RF Pocket same nie zapewniają tego połączenia.

## Sprzęt i różnice

Alvik ma Nano ESP32 / NORA-W106 / ESP32-S3, Wi-Fi 2,4 GHz, Bluetooth LE
i pomocniczy STM32. Szczegółowe źródła są w
[ustaleniach sprzętowych](../../docs/alvik-hardware.md).

Sterowanie kołami odbywa się przez `ArduinoAlvik.set_wheels_speed(left, right)`
w RPM, zgodnie z [biblioteką Arduino](https://github.com/arduino/arduino-alvik-mpy).
Nie używamy pinów D5/D6, rejestru 74HC595 ani ramek UART Wobot dla UNO.
Alvik nie ma kół mecanum: wychylenie osi jazdy bokiem zatrzymuje go i wymaga
ponownego DISARM. CH7 nie steruje serwem w tym wariancie.

## Pierwsze uruchomienie bez ruchu

1. Użyj Nano ESP32 zamontowanego w Alviku, z MicroPython i biblioteką
   `arduino_alvik` zgodną z firmware kontrolera STM32. Sam deskryptor USB
   „Nano ESP32” nie potwierdza, jaki program jest wgrany. Nie wgrywaj obrazów
   UNO ani YD-ESP32-S3 z tego repozytorium. Instalację lub aktualizację
   środowiska opisuje [Arduino](https://support.arduino.cc/hc/en-us/articles/18309882557724-Update-firmware-and-libraries-for-Arduino-Alvik).
2. Skopiuj `config.example.json` jako lokalny `config.json` i wpisz `ssid`
   oraz `password` istniejącej sieci 2,4 GHz. Ten plik jest ignorowany przez Git.
   Zachowaj `dry_run: true` i `calibration_confirmed: false`.
3. W Pocket włącz Wi-Fi modułu ELRS i połącz moduł z tą samą siecią.
   Nie zmieniaj modelu/mikserów działającego robota. Przełącznik ARM pozostaw
   wyłączony. Nie uruchamiaj równocześnie monitora ELRS na komputerze ani
   starego robota, który automatycznie reaktywuje tę samą sesję joysticka.
4. Przez Arduino Lab for MicroPython skopiuj do katalogu głównego Alvika
   `control.py`, `joystick.py`, `alvik_main.py` i `config.json`. Zachowaj kopię
   istniejących plików, jeśli występują kolizje nazw. Nie zastępuj `main.py`,
   programu demonstracyjnego ani bibliotek. Zatrzymaj dotychczasowy program
   przy uniesionych kołach i uruchom z REPL:

   ```python
   import alvik_main
   alvik_main.run()
   ```

5. Program wypisze adres Wi-Fi Alvika. Przy pustym `transmitter_ip` wykryje
   pierwsze poprawne ogłoszenie ELRS na UDP/11000 i wypisze nazwę oraz IP
   nadajnika. W tej samej podsieci nie trzeba znać adresu z góry. W sieciach
   z blokowanym broadcastem lub między podsieciami wpisz IP modułu Pocket
   z routera do `transmitter_ip`. Nie wpisuj tu adresu robota.
6. Odczytaj kanały podczas ruchu drążków i przełączników. W trybie diagnostycznym
   biblioteka Alvika nie jest nawet inicjalizowana; `output rpm` zawsze jest
   zerowe. `requested rpm` pokazuje wynik mapowania po spełnieniu ARM.
   Pierwsze ogłoszenie jest wybierane tylko na czas uruchomienia; po identyfikacji
   warto wpisać IP właściwego Pocket do konfiguracji, zwłaszcza przy wielu radiach.

Uruchomienie Alvika przejmuje sesję Wi-Fi Joystick na ten adres odbiorczy.
Nie usuwa konfiguracji poprzedniego robota, ale nie jest równoczesnym
sterowaniem dwoma robotami. Po zakończeniu (`Ctrl-C`) można przywrócić
strumień z panelu poprzedniego robota. Nie wysyłamy `joystick_end` przy
zamykaniu programu, aby nie wyłączyć ewentualnej sesji innego klienta.

## Mapowanie i kalibracja

Domyślne kanały odpowiadają domyślnym mapowaniom w Wobot; zapisane na starym
ESP32 ustawienia NVS mogą się różnić i nie są automatycznie kopiowane.

| Kanał | Funkcja |
| --- | --- |
| CH1 | skręt, środek = prosto |
| CH2 | przód/tył w profilu prawego drążka |
| CH3 | gaz w profilu pedału, minimum = stop |
| CH4 | jazda bokiem — musi być wycentrowana na Alviku |
| CH5 | ARM, tylko górna skrajna pozycja uzbraja |
| CH6 | kierunek pedału: dół = wstecz, środek = stop, góra = przód |
| CH8 | góra = prawy drążek; dół/środek = pedał |

`channels` używa numerów od 1. `ranges` to 10 trójek `[min, środek, max]`
w surowej skali Wi-Fi 0–32767, nie mikrosekundy PWM ani surowe CRSF.
Wpisz zaobserwowane zakresy i środki; dla przełącznika dwupozycyjnego jako
środek wpisz środek zakresu. `inverted` zawiera nazwy odwróconych funkcji,
np. `["throttle"]`. Zapisz zmiany po zakończeniu programu i uruchom go ponownie.
Zakresy przykładowe nie są pomiarem egzemplarza użytkownika.

Po sprawdzeniu mapowania ustaw `calibration_confirmed: true`.
Do osobnej próby ruchu z kołami uniesionymi ustaw `dry_run: false`.
Początkowy limit wynosi 20 RPM. Nie przenosimy impulsu rozruchowego 210/255
ze szkicu UNO — Alvik przyjmuje prędkość w RPM.

## Zatrzymanie i ograniczenia

- Przy starcie, zmianie profilu, błędnej ramce i utracie danych potrzebny
  jest DISARM, a następnie ARM z neutralnym gazem, skrętem i CH4.
- Dane wygasają po 250 ms (ustawienie tylko Alvika). Zatrzymanie jest
  sprawdzane w pętli co około 20 ms; rzeczywisty czas wymaga pomiaru.
- HTTP aktywujące radio i ponowne łączenie Wi-Fi następują po komendzie
  zerowej prędkości. Kolejka UDP z okresu aktywacji jest odrzucana.
- Przepełniona kolejka, błąd sieci, `Ctrl-C` i zwykłe wyjątki zatrzymują
  napęd. Jedynie pakiety z wybranego IP mogą sterować robotem.
- Format ELRS Wi-Fi nie daje tutaj uwierzytelnienia ani numeru sekwencji:
  filtr IP nie chroni przed podszyciem się lub powtórzeniem pakietu.
- Lokalny timeout nie zastępuje niezależnego watchdoga przy zawieszeniu
  interpretera lub awarii UART/STM32. Zachowania fabrycznego kontrolera w takiej
  sytuacji nie przetestowano. Nie deklarujemy gotowości do jazdy bez nadzoru.

## Testy

Z katalogu repozytorium:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -p 'test_alvik*.py' -v
cargo test --workspace
```

Testy Python obejmują format pakietów, kalibrację, profile, ARM, timeout
(także przepełnienie licznika czasu), filtrowanie nadawcy, przepełnienie
kolejki, błędy HTTP oraz pętlę programu z atrapami sieci i Alvika.
Nie potwierdzają fizycznego kierunku kół ani zgodności wgranego MicroPython.
Pliki `.py` można sprawdzić kompilatorem `mpy-cross`; na robota kopiujemy
źródła, aby nie zakładać wersji formatu bytecode jego interpretera.

Wyniki lokalne 2026-09-07: 27 testów Python — OK; wszystkie trzy pliki
programu skompilowane przez `mpy-cross` 1.29.0 (mpy v6.3). W tej sesji
przeszły też 21 istniejących testów Rust, test C++ wyjść UNO i kompilacja
szkicu UNO. Nie kompilowano ponownie niezmienionego przez tę integrację
firmware ESP-IDF ani nie instalowano programu na Alviku.
