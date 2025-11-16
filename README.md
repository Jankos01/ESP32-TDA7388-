# Wzmacniacz Audio Bluetooth ESP32 

Projekt "bezgłowego" (headless) odbiornika audio Bluetooth o wysokiej jakości, zbudowanego na bazie modułu ESP32-WROOM-32UE (DevKitC), przetwornika DAC PCM5102 i wzmacniacza samochodowego TDA7388. Całość uzupełnia inteligentny system chłodzenia, sterowany czujnikiem temperatury DS18B20 oraz tranzystorem MOSFET.

Projekt koncentruje się na stabilności, jakości dźwięku oraz efektywnym zarządzaniu energią i temperaturą, co czyni go idealnym do zastosowań jako stacjonarny system audio.

## 🌟 Główne Funkcje

* **Wysoka Jakość Dźwięku:** Przesyłanie sygnału audio przez interfejs I2S do zewnętrznego przetwornika DAC PCM5102 dla minimalizacji zakłóceń i poprawy wierności dźwięku.
* **Anti-Pop:** Eliminuje nieprzyjemne "stuknięcia" i "pyknięcia" w głośnikach podczas włączania i przełączania stanów wzmacniacza, poprzez precyzyjne sterowanie pinami `MUTE` i `ST-BY`.
* **Inteligentne Sterowanie Wzmacniaczem TDA7388:**
    * Wzmacniacz domyślnie uruchamia się w stanie `MUTE` (wyciszony) dla bezpieczeństwa głośników.
    * Automatyczne od-wyciszanie (`AMP_ON`), gdy tylko wykryty zostanie strumień danych audio Bluetooth.
    * Automatyczne przejście w stan `MUTE` po konfigurowalnym czasie bez odtwarzania muzyki (domyślnie 30 sekund).
    * Automatyczne przejście w energooszczędny stan `STANDBY` (wzmacniacz wyłączony) po kolejnym konfigurowalnym czasie w trybie `MUTE` (domyślnie 30 sekund).
* **Dynamiczne Chłodzenie PWM:** Wentylator jest aktywowany i jego prędkość jest regulowana proporcjonalnie tylko wtedy, gdy wzmacniacz jest w stanie `AMP_ON` i temperatura czujnika DS18B20 przekroczy ustalony próg (np. 40°C).
* **Centralna Konfiguracja:** Wszystkie piny GPIO, progi temperaturowe, czasy opóźnień, nazwa Bluetooth i inne parametry są łatwo edytowalne w pliku `include/config.h`.
* **Optymalizacje Systemowe:** Podniesienie priorytetu zadania I2S i stabilizacja głównej pętli `loop()` mają na celu minimalizację potencjalnych zakłóceń audio i zapewnienie płynnej pracy.

## ⚙️ Komponenty Projektu

* **Płytka Deweloperska:** ESP32-DevKitC (lub inna zgodna z ESP32-WROOM-32UE)
* **Przetwornik Cyfrowo-Analogowy (DAC):** Moduł PCM5102A (I2S Stereo)
* **Wzmacniacz Audio:** Moduł oparty na TDA7388 (4 kanały, często używany w motoryzacji)
* **Czujnik Temperatury:** DS18B20 (cyfrowy, 1-Wire, wodoodporny)
* **Tranzystor MOSFET:** N-Channel Logic Level, np. IRLZ44N (do sterowania wentylatorem PWM)
* **Wentylator:** Wentylator 12V DC (zazwyczaj 3-pinowy, ale w projekcie używamy tylko sterowania PWM prędkością)
* **Rezystory:**
    * `4.7kΩ` (dla DS18B20 - Pull-up na linii danych)
    * `10kΩ` (dla bramki MOSFETa - Pull-down)
* **Zasilanie:** Stabilne źródło zasilania 12V DC dla wzmacniacza i wentylatora. ESP32 i DAC mogą być zasilane z 3.3V/5V z tego samego zasilacza (poprzez odpowiednie stabilizatory/konwertery na płytce ESP32).
* **Kondensatory (opcjonalnie, do redukcji szumów):** `100nF` (ceramiczny) i `100uF` (elektrolityczny) blisko DAC PCM5102A na linii zasilania 3.3V.
* **Tłumik Sprzętowy (opcjonalnie, do redukcji szumów):** Rezystory tworzące dzielnik napięcia (L-Pad) na wejściach audio wzmacniacza, między DAC a TDA7388.

## 🔌 Schemat Połączeń (ESP32-DevKitC Layout)

Poniższy schemat przedstawia połączenia komponentów z płytką ESP32-DevKitC, bazując na jej standardowym pinoucie.

![Schemat Połączeń Inteligentnego Wzmacniacza Bluetooth ESP32 v2](https://i.imgur.com/TwoiEaW.png)

### Szczegółowa Lista Połączeń:

| Moduł ESP32-DevKitC Pin (GPIO) | Komponent Docelowy | Uwagi |
| :----------------------------- | :----------------- | :---- |
| **`GPIO12`** | `DIN` (DAC PCM5102)  | I2S Data |
| **`GPIO13`** | `LRC` (DAC PCM5102)  | I2S Word Select (Left/Right Clock) |
| **`GPIO14`** | `BCK` (DAC PCM5102)  | I2S Bit Clock |
| **`GPIO27`** | `SCK` (DAC PCM5102)  | Pin SCK z DAC powinien być podłączony do tego GPIO, które jest ustawione na LOW (GND) w kodzie, aby zapewnić prawidłową pracę PCM5102. |
| **`GPIO32`** | `ST-BY` (TDA7388)    | Pin Standby wzmacniacza (wymaga filtra RC dla anti-pop, jeśli nie jest wbudowany we wzmacniaczu) |
| **`GPIO33`** | `MUTE` (TDA7388)     | Pin Mute wzmacniacza (wymaga filtra RC dla anti-pop, jeśli nie jest wbudowany we wzmacniaczu) |
| **`GPIO17`** | `DATA` (DS18B20)     | Linia danych czujnika temperatury 1-Wire. **Wymaga zewnętrznego rezystora pull-up 4.7kΩ do 3.3V.** |
| **`GPIO16`** | `GATE` (MOSFET IRLZ44N) | Sterowanie bramką tranzystora MOSFET. **Wymaga rezystora pull-down 10kΩ do GND.** |
| **`3V3`** | `VCC` (DAC PCM5102)  | Zasilanie DAC |
| **`3V3`** | `VCC` (DS18B20)      | Zasilanie czujnika temperatury |
| **`GND`** | `GND` (DAC PCM5102)  | Wspólna masa |
| **`GND`** | `GND` (DS18B20)      | Wspólna masa |
| **`GND`** | `SOURCE` (MOSFET IRLZ44N) | Wspólna masa dla tranzystora |
| **`GND`** | `GND` (TDA7388)      | Wspólna masa dla wzmacniacza. **Kluczowe jest, aby wszystkie GND były połączone!** |

---
## 💻 Przegląd Kodu

Projekt składa się z trzech głównych plików: `platformio.ini`, `include/config.h` oraz `src/main.cpp`.

### `platformio.ini`
Ten plik konfiguracyjny PlatformIO definiuje środowisko kompilacji, zależności i ustawienia monitora szeregowego.



### `include/config.h`

Plik `config.h` zawiera wszystkie stałe konfiguracyjne projektu. Umożliwia łatwe dostosowanie pinów, progów, czasów opóźnień i nazw bez konieczności modyfikowania głównego pliku kodu `main.cpp`.

```cpp
#ifndef CONFIG_H
#define CONFIG_H

// Piny I2S dla DAC PCM5102A
const int PIN_I2S_BCLK = 14; 
const int PIN_I2S_LRC  = 13; 
const int PIN_I2S_DATA = 12; 
const int PIN_DAC_SCK_TO_GND = 27; 

// Piny sterujące wzmacniaczem TDA7388
const int PIN_AMP_STBY = 32; 
const int PIN_AMP_MUTE = 33; 

// Piny dla czujnika temperatury i wentylatora
const int PIN_TEMP_SENSOR = 17; 
const int PIN_FAN_PWM = 16;     

// Nazwa urządzenia Bluetooth
const char* BLUETOOTH_DEVICE_NAME = "ESP32-Audio";

// Domyślna głośność i czasy opóźnień
const long standbyDelay = 30000;  // 30 sekund
const int DEFAULT_VOLUME = 100;
const int AMP_WAKE_DELAY = 50; 
const int AMP_MUTE_DELAY = 10; 

// Progi temperaturowe i prędkości wentylatora
const float TEMP_FAN_OFF = 40.0; 
const float TEMP_FAN_MAX = 60.0; 
const int FAN_SPEED_MIN = 100;   
const int FAN_SPEED_MAX = 255;   
const long TEMP_CHECK_INTERVAL = 5000; 

#endif // CONFIG_H
```

**Opis kluczowych sekcji:**
  * **Piny I2S (DAC PCM5102A):** Definiuje piny GPIO ESP32 używane do przesyłania danych audio cyfrowych (I2S) do zewnętrznego przetwornika DAC. `PIN_DAC_SCK_TO_GND` to pin kontrolny dla DAC-a, ustawiany na LOW w kodzie.
  * **Piny Wzmacniacza (TDA7388):** Określa piny GPIO ESP32, które sterują stanem pracy wzmacniacza: `ST-BY` (Standby - tryb uśpienia/aktywności) i `MUTE` (wyciszenie/odtwarzanie).
  * **Piny Chłodzenia:** Definiuje pin GPIO dla cyfrowego czujnika temperatury `DS18B20` (interfejs 1-Wire) oraz pin PWM dla tranzystora MOSFET, który kontroluje prędkość wentylatora.
  * **Ustawienia Bluetooth:** Konfiguruje nazwę urządzenia Bluetooth (`BLUETOOTH_DEVICE_NAME`), widoczną dla innych urządzeń, a także opcjonalne ustawienia hasła (PIN-u) do parowania.
  * **Ustawienia Audio i Wzmacniacza:** Zawiera parametry czasowe dla logiki automatycznego zarządzania stanem wzmacniacza (np. `standbyDelay` określa czas bez aktywności audio, po którym wzmacniacz przejdzie w tryb MUTE). `DEFAULT_VOLUME` to poziom głośności ustawiany po połączeniu Bluetooth. `AMP_WAKE_DELAY` i `AMP_MUTE_DELAY` to krótkie opóźnienia, kluczowe dla funkcji Anti-Pop.
  * **Ustawienia Wentylatora:** Definiuje parametry techniczne sygnału PWM dla wentylatora (kanał, częstotliwość, rozdzielczość bitowa) oraz progi temperaturowe (`TEMP_FAN_OFF`, `TEMP_FAN_MAX`), w których wentylator zaczyna działać i reguluje swoją prędkość. `TEMP_CHECK_INTERVAL` określa, jak często temperatura jest sprawdzana.

---

### `src/main.cpp`

To jest główny plik źródłowy zawierający całą logikę programu, zarządzanie Bluetooth A2DP, sterowanie wzmacniaczem i wentylatorem.

#### Inicjalizacja i globalne zmienne

```cpp
#include <Arduino.h>
#include "BluetoothA2DPSink.h"
#include "config.h" 
#include <DallasTemperature.h>  

BluetoothA2DPSink a2dp_sink; 
DallasTemperature tempSensor(&oneWire); 

enum AmpState { AMP_OFF, AMP_MUTE, AMP_ON };
AmpState currentAmpState = AMP_OFF; 

unsigned long standbyTimer = 0; 
unsigned long lastTempCheck = 0; 
```
**Opis:**
  * Sekcja `#include` wczytuje wszystkie niezbędne biblioteki.
  * `BluetoothA2DPSink a2dp_sink;`: Tworzy główny obiekt biblioteki Bluetooth A2DP, który zarządza połączeniem i strumieniem audio.
  * `DallasTemperature tempSensor(&oneWire);`: Obiekt do komunikacji z czujnikiem temperatury DS18B20.
  * `enum AmpState { AMP_OFF, AMP_MUTE, AMP_ON };`: Definiuje czytelne stany pracy wzmacniacza.
  * `currentAmpState`: Przechowuje aktualny stan wzmacniacza.
  * `standbyTimer`, `lastTempCheck`: Zmienne `unsigned long` do implementacji niewblokujących timerów, śledzących czas bez aktywności audio i czas ostatniego sprawdzenia temperatury.

#### Funkcja `setAmpState(AmpState newState)`

```cpp
void setAmpState(AmpState newState) {
  if (newState == currentAmpState) return; 

  switch (newState) {
    case AMP_ON: 
      digitalWrite(PIN_AMP_STBY, HIGH); 
      delay(AMP_WAKE_DELAY); 
      digitalWrite(PIN_AMP_MUTE, HIGH); 
      break;
    case AMP_MUTE: 
      digitalWrite(PIN_AMP_STBY, HIGH); 
      delay(AMP_WAKE_DELAY); 
      digitalWrite(PIN_AMP_MUTE, LOW); 
      break;
    case AMP_OFF: 
      digitalWrite(PIN_AMP_MUTE, LOW); 
      delay(AMP_MUTE_DELAY); 
      digitalWrite(PIN_AMP_STBY, LOW); 
      break;
  }
  currentAmpState = newState;
}
```
**Opis:**
  * Ta funkcja jest centralnym punktem do bezpiecznego sterowania wzmacniaczem TDA7388. Akceptuje nowy stan pracy (`AMP_ON`, `AMP_MUTE`, `AMP_OFF`).
  * Wykorzystuje instrukcję `switch` do dokładnego ustawiania pinów `ST-BY` (Standby) i `MUTE` w odpowiedniej sekwencji dla każdego stanu.
  * Wbudowane opóźnienia (`AMP_WAKE_DELAY`, `AMP_MUTE_DELAY` z `config.h`) służą do funkcji Anti-Pop, zapobiegając nieprzyjemnym "stuknięciom" w głośnikach podczas przełączania stanów.

#### Funkcja `handleFanControl()`

```cpp
void handleFanControl() {
  tempSensor.requestTemperatures(); 
  float tempC = tempSensor.getTempCByIndex(0);

  if (tempC == DEVICE_DISCONNECTED_C) {
    ledcWrite(FAN_PWM_CHANNEL, 0); 
    return;
  }

  if (currentAmpState == AMP_ON) { 
    if (tempC < TEMP_FAN_OFF) {
      ledcWrite(FAN_PWM_CHANNEL, 0); 
    } else if (tempC > TEMP_FAN_MAX) {
      ledcWrite(FAN_PWM_CHANNEL, FAN_SPEED_MAX); 
    } else {
      int fanSpeed = map(tempC, TEMP_FAN_OFF, TEMP_FAN_MAX, FAN_SPEED_MIN, FAN_SPEED_MAX);
      ledcWrite(FAN_PWM_CHANNEL, fanSpeed);
    }
  } else {
    ledcWrite(FAN_PWM_CHANNEL, 0); 
  }
}
```
**Opis:**
  * Ta funkcja odpowiada za odczyt temperatury z czujnika DS18B20 i sterowanie wentylatorem.
  * Sprawdza, czy wzmacniacz jest włączony (`AMP_ON`) – wentylator działa tylko wtedy, gdy wzmacniacz jest aktywny.
  * Realizuje logikę proporcjonalnego sterowania prędkością wentylatora za pomocą PWM: wentylator włącza się, gdy temperatura przekroczy `TEMP_FAN_OFF`, a jego prędkość rośnie liniowo aż do `TEMP_FAN_MAX`, gdzie pracuje na pełnej mocy. Poniżej `TEMP_FAN_OFF` wentylator jest wyłączony.
  * W przypadku błędu odczytu czujnika, wentylator jest zatrzymywany.

#### Funkcje zwrotne Bluetooth (Callbacks)

```cpp
void connection_state_changed(esp_a2d_connection_state_t state, void *ptr) {
  if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
    setAmpState(AMP_MUTE); 
    a2dp_sink.set_volume(DEFAULT_VOLUME); 
  } else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
    setAmpState(AMP_MUTE); 
  }
  standbyTimer = millis(); 
}

void stream_reader_callback(const uint8_t *data, uint32_t length) {
  if (currentAmpState != AMP_ON) {
    setAmpState(AMP_ON);
  }
  standbyTimer = millis(); 
}
// Podobna struktura dla avrc_metadata_callback
```
**Opis:**
  * Są to funkcje "callback", które są automatycznie wywoływane przez bibliotekę `BluetoothA2DPSink` w odpowiedzi na różne zdarzenia Bluetooth.
  * `connection_state_changed`: Reaguje na podłączenie lub rozłączenie urządzenia Bluetooth. Po połączeniu ustawia wzmacniacz w tryb `MUTE` (oczekując na dane audio) i ustawia domyślną głośność. Po rozłączeniu również przechodzi w `MUTE`.
  * `stream_reader_callback`: To najważniejsza funkcja do wykrywania, czy muzyka jest faktycznie odtwarzana. Jest wywoływana przy przesyłaniu danych audio. Upewnia się, że wzmacniacz jest włączony (`AMP_ON`) i resetuje timer standby.
  * `avrc_metadata_callback`: (Chociaż nie pokazano w całości, ma podobną strukturę) Jest wywoływana przy odbieraniu metadanych (np. tytułu utworu) i również służy do resetowania timera standby oraz upewniania się, że wzmacniacz jest aktywny.

#### Funkcja `setup()`

```cpp
void setup() {
  Serial.begin(115200); 

  // Inicjalizacja pinów wzmacniacza i DAC SCK
  pinMode(PIN_AMP_STBY, OUTPUT);
  digitalWrite(PIN_DAC_SCK_TO_GND, LOW);

  // Konfiguracja PWM wentylatora
  ledcSetup(FAN_PWM_CHANNEL, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
  ledcAttachPin(PIN_FAN_PWM, FAN_PWM_CHANNEL);

  // Inicjalizacja czujnika DS18B20
  tempSensor.begin();
  
  // Konfiguracja I2S dla DAC
  i2s_pin_config_t my_i2s_pins = { .bck_io_num = PIN_I2S_BCLK, .ws_io_num = PIN_I2S_LRC, .data_out_num = PIN_I2S_DATA, .data_in_num = I2S_PIN_NO_CHANGE };
  a2dp_sink.set_pin_config(my_i2s_pins);
  
  // Rejestracja funkcji zwrotnych Bluetooth
  a2dp_sink.set_on_connection_state_changed(connection_state_changed);
  // ... inne rejestracje ...
  
  // Optymalizacja I2S
  a2dp_sink.set_i2s_port_priority(configMAX_PRIORITIES - 2);

  // Konfiguracja i start Bluetooth A2DP
  a2dp_sink.start(BLUETOOTH_DEVICE_NAME); 
}
```
**Opis:**
  * Ta funkcja jest wywoływana tylko raz na początku programu.
  * Inicjalizuje port szeregowy, konfiguruje piny GPIO dla wzmacniacza i DAC (ustawienie `PIN_DAC_SCK_TO_GND` na LOW), konfiguruje PWM wentylatora i inicjalizuje czujnik DS18B20.
  * Ustawia konfigurację I2S dla przetwornika DAC.
  * Rejestruje wszystkie funkcje zwrotne Bluetooth.
  * `a2dp_sink.set_i2s_port_priority(...)`: Podnosi priorytet zadania I2S w systemie operacyjnym ESP32, co jest optymalizacją mającą na celu zapewnienie płynniejszego odtwarzania audio i zminimalizowanie zakłóceń.
  * `a2dp_sink.start(BLUETOOTH_DEVICE_NAME);`: Uruchamia usługę Bluetooth A2DP z nazwą zdefiniowaną w `config.h` (oraz z opcjonalnym PIN-em, jeśli skonfigurowano).

#### Funkcja `loop()`

```cpp
void loop() {
  unsigned long now = millis();

  // Logika timera auto-wyciszenia/standby
  if (standbyTimer != 0 && (now - standbyTimer > standbyDelay)) {
    if (currentAmpState == AMP_ON) {
      setAmpState(AMP_MUTE);
      standbyTimer = millis(); 
    }
    else if (currentAmpState == AMP_MUTE) {
      setAmpState(AMP_OFF);
      standbyTimer = 0; 
    }
  }

  // Logika sprawdzania temperatury
  if (now - lastTempCheck > TEMP_CHECK_INTERVAL) {
    lastTempCheck = now;
    handleFanControl(); 
  }
  
  delay(1); 
}
```
**Opis:**
  * To jest główna pętla programu, która działa w nieskończoność po zakończeniu `setup()`.
  * **Logika timera auto-wyciszenia/standby:** W regularnych odstępach czasu (`standbyDelay` z `config.h`) sprawdza, czy nie ma aktywności audio. Jeśli tak, wzmacniacz przełącza się kolejno ze stanu `AMP_ON` na `AMP_MUTE`, a następnie z `AMP_MUTE` na `AMP_OFF` (Standby), oszczędzając energię.
  * **Logika sprawdzania temperatury:** Co `TEMP_CHECK_INTERVAL` (z `config.h`) wywołuje funkcję `handleFanControl()`, aby odczytać temperaturę i odpowiednio sterować wentylatorem.
  * `delay(1);`: Krótkie opóźnienie w pętli, które jest dobrą praktyką w systemach RTOS (jak ESP32). Pozwala to procesorowi na wykonanie innych zadań systemowych w tle (np. obsługę Bluetooth) i zapobiega "zapychaniu" pętli głównej.

---

## 💻 Instalacja i Uruchomienie (PlatformIO)

Ten projekt jest przeznaczony do kompilacji i wgrania za pomocą PlatformIO IDE w Visual Studio Code.

### Wymagania:
* Visual Studio Code
* Rozszerzenie PlatformIO IDE dla VS Code
* Zainstalowany Git na systemie operacyjnym

### Kroki:

1.  **Sklonuj Repozytorium:**
    ```bash
    git clone [https://github.com/TwojaNazwaUzytkownika/Inteligentny_Wzmacniacz_Bluetooth.git](https://github.com/TwojaNazwaUzytkownika/Inteligentny_Wzmacniacz_Bluetooth.git)
    cd Inteligentny_Wzmacniacz_Bluetooth
    ```
    *(Zastąp `TwojaNazwaUzytkownika` i `Inteligentny_Wzmacniacz_Bluetooth` swoimi danymi, jeśli projekt jest już na GitHubie).*

2.  **Otwórz Projekt w VS Code:**
    Otwórz folder `Inteligentny_Wzmacniacz_Bluetooth` w Visual Studio Code (`File -> Open Folder...`).

3.  **Zainstaluj Zależności:**
    PlatformIO automatycznie pobierze wymagane biblioteki (BluetoothA2DPSink, OneWire, DallasTemperature) zgodnie z plikiem `platformio.ini`.

4.  **Skonfiguruj Projekt (`include/config.h`):**
    Otwórz plik `include/config.h` i dostosuj następujące parametry do swoich potrzeb:
    * `BLUETOOTH_DEVICE_NAME`: Nazwa, pod którą urządzenie będzie widoczne.
    * `DEFAULT_VOLUME`: Domyślna głośność po połączeniu (0-127).
    * Progi temperaturowe (`TEMP_FAN_OFF`, `TEMP_FAN_MAX`) oraz prędkości wentylatora.
    * Piny GPIO, jeśli używasz innej konfiguracji niż domyślna.

5.  **Kompilacja i Wgranie:**
    * Podłącz swoją płytkę ESP32 do komputera.
    * W PlatformIO IDE (dolny pasek VS Code lub boczny panel):
        * Kliknij ikonę **"Check" (✓)**, aby skompilować kod.
        * Kliknij ikonę **"Upload" (strzałka w prawo)**, aby wgrać skompilowany kod na ESP32.

6.  **Monitor Portu Szeregowego:**
    * Kliknij ikonę **"Serial Monitor" (wtyczka)**, aby otworzyć terminal szeregowy. Upewnij się, że prędkość (baud rate) jest ustawiona na `115200`.
    * W tym terminalu będziesz widzieć wszystkie logi diagnostyczne i komunikaty o stanie urządzenia.

---

## 📝 Struktura Projektu


