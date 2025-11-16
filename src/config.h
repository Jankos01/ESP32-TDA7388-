#ifndef CONFIG_H
#define CONFIG_H

// ===             KONFIGURACJA SPRZĘTOWA (WERSJA Z CHŁODZENIEM)         ===
// =========================================================================

// --- Piny I2S (Przetwornik DAC PCM5102A) ---
const int PIN_I2S_BCLK = 14; // Bit Clock (BCK)
const int PIN_I2S_LRC  = 13; // Left/Right Clock (WS, LRCK)
const int PIN_I2S_DATA = 12; // Data Out (DIN, DOUT, SD)

// --- Pin sterujący DAC ---
const int PIN_DAC_SCK_TO_GND = 27;

// --- Piny Wzmacniacza (TDA7388) ---
const int PIN_AMP_STBY = 25; // Pin Standby
const int PIN_AMP_MUTE = 26; // Pin Mute

// --- Piny Chłodzenia ---
const int PIN_TEMP_SENSOR = 19; // Pin 1-Wire dla czujnika DS18B20
const int PIN_FAN_PWM = 16;     // Pin PWM do sterowania MOSFETem (IRLZ44N)

// --- Ustawienia Audio ---
const long standbyDelay = 12000;  // 30 sekund do Standby

// === DODANA STAŁA: Domyślna głośność ===
// Ustawia głośność (w zakresie 0-127) przy każdym nowym połączeniu.
// 100 = ok. 78% (dobry start)
// 127 = 100% (maksimum)
const int DEFAULT_VOLUME = 60;

// --- Ustawienia Wzmacniacza (Opóźnienia) ---
const int AMP_WAKE_DELAY = 1500; 
const int AMP_MUTE_DELAY = 1500; 

// --- Ustawienia Wentylatora ---
const int FAN_PWM_CHANNEL = 0; 
const int FAN_PWM_FREQ = 25000; 
const int FAN_PWM_RESOLUTION = 8; // 0-255

const float TEMP_FAN_OFF = 40.0; // Poniżej tej temp. wentylator stoi
const float TEMP_FAN_MAX = 60.0; // Powyżej tej temp. wentylator działa na 100%
const int FAN_SPEED_MIN = 100;   // Minimalna moc PWM (aby wentylator ruszył)
const int FAN_SPEED_MAX = 255;   // Maksymalna moc PWM

const long TEMP_CHECK_INTERVAL = 5000; // Sprawdzaj co 5 sekund (5000 ms)

const char* BLUETOOTH_DEVICE_NAME = "ESP DAC";

#endif // CONFIG_H