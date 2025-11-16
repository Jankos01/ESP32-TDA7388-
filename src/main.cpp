#include <Arduino.h>
#include "BluetoothA2DPCommon.h"
#include "BluetoothA2DPSink.h"
#include "config.h" 
#include <OneWire.h>            
#include <DallasTemperature.h>  

// --- Konfiguracja Audio (A2DP) ---
BluetoothA2DPSink a2dp_sink;

// --- Konfiguracja Czujnika Temperatury ---
OneWire oneWire(PIN_TEMP_SENSOR);
DallasTemperature tempSensor(&oneWire);

// --- Stany Wzmacniacza TDA7388 ---
enum AmpState { AMP_OFF, AMP_MUTE, AMP_ON };
AmpState currentAmpState = AMP_OFF; 

// --- Zmienne dla Timerów ---
unsigned long standbyTimer = 0; 
unsigned long lastTempCheck = 0; 

// === FUNKCJA: Zarządzanie stanem wzmacniacza TDA7388 ===
void setAmpState(AmpState newState) {
  if (newState == currentAmpState) return; 

  switch (newState) {
    case AMP_ON: 
      Serial.println("AMP: Stan -> ON (Gra)");
      if (currentAmpState == AMP_OFF) { 
        digitalWrite(PIN_AMP_STBY, HIGH); 
        delay(AMP_WAKE_DELAY); 
      }
      digitalWrite(PIN_AMP_MUTE, HIGH); 
      break;
    case AMP_MUTE: 
      Serial.println("AMP: Stan -> MUTE (Cisza, czuwanie)");
      if (currentAmpState == AMP_OFF) { 
        digitalWrite(PIN_AMP_STBY, HIGH); 
        delay(AMP_WAKE_DELAY); 
      }
      digitalWrite(PIN_AMP_MUTE, LOW); 
      break;
    case AMP_OFF: 
      Serial.println("AMP: Stan -> OFF (Standby)");
      if (currentAmpState == AMP_ON) { 
        digitalWrite(PIN_AMP_MUTE, LOW); 
        delay(AMP_MUTE_DELAY); 
      }
      digitalWrite(PIN_AMP_STBY, LOW); 
      break;
  }
  currentAmpState = newState;
}

// === FUNKCJA: Sterowanie Wentylatorem ===
void handleFanControl() {
  
  if (currentAmpState != AMP_ON) {
    ledcWrite(FAN_PWM_CHANNEL, 0); 
    return; 
  }
  
  Serial.print("TEMP: Odczyt temperatury (Wzmacniacz włączony)...");
  tempSensor.requestTemperatures(); 
  float tempC = tempSensor.getTempCByIndex(0);

  if (tempC == DEVICE_DISCONNECTED_C) {
    Serial.println(" BŁĄD! Nie można odczytać czujnika DS18B20.");
    ledcWrite(FAN_PWM_CHANNEL, 0); 
    return;
  }

  Serial.print(" "); Serial.print(tempC); Serial.println(" C");

  if (tempC < TEMP_FAN_OFF) {
    ledcWrite(FAN_PWM_CHANNEL, 0);
  } else if (tempC > TEMP_FAN_MAX) {
    ledcWrite(FAN_PWM_CHANNEL, FAN_SPEED_MAX);
  } else {
    int fanSpeed = map(tempC, TEMP_FAN_OFF, TEMP_FAN_MAX, FAN_SPEED_MIN, FAN_SPEED_MAX);
    ledcWrite(FAN_PWM_CHANNEL, fanSpeed);
  }
}

// --- Funkcje zwrotne Bluetooth ---

void connection_state_changed(esp_a2d_connection_state_t state, void *ptr) {
  Serial.print("DEBUG: connection_state_changed WYWOŁANA! Stan: "); Serial.println(state);
  if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
    Serial.println("DEBUG: Połączono. Wzmacniacz w stanie MUTE, czeka na dane.");
    setAmpState(AMP_MUTE); 
    standbyTimer = millis(); 
    
    
    a2dp_sink.set_volume(DEFAULT_VOLUME); 
   

  } else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED || state == ESP_A2D_CONNECTION_STATE_DISCONNECTING) {
    Serial.println("DEBUG: Rozłączono. Przechodzę w MUTE i uruchamiam timer Standby.");
    setAmpState(AMP_MUTE); 
    standbyTimer = millis(); 
  }
}

void avrc_metadata_callback(uint8_t id, const uint8_t *text) {
  if (id == ESP_AVRC_MD_ATTR_TITLE) {
      Serial.print("DEBUG: Otrzymano metadane tytułu: "); Serial.println((const char*)text);
      standbyTimer = millis();  
      setAmpState(AMP_ON); 
    }
}

void stream_reader_callback(const uint8_t *data, uint32_t length) {
  if (currentAmpState != AMP_ON) {
    setAmpState(AMP_ON);
  }
  standbyTimer = millis(); 
}

// --- SETUP ---
void setup() {
  Serial.begin(115200); Serial.println("\n\n--- START PROGRAMU (Headless, TDA7388 + Chłodzenie v4) ---");

  // --- Ustaw piny wzmacniacza (Mute/Standby) ---
  pinMode(PIN_AMP_STBY, OUTPUT);
  pinMode(PIN_AMP_MUTE, OUTPUT);
  setAmpState(AMP_MUTE); // Start w MUTE
  standbyTimer = millis(); 
  Serial.println("DEBUG: Wzmacniacz w stanie MUTE na starcie. Uruchomiono timer Standby.");

  // --- Ustaw pin DAC SCK na LOW ---
  pinMode(PIN_DAC_SCK_TO_GND, OUTPUT);
  digitalWrite(PIN_DAC_SCK_TO_GND, LOW);
  Serial.println("DEBUG: Pin DAC SCK ustawiony na LOW (GND).");

  // --- Konfiguracja wentylatora (PWM) ---
  ledcSetup(FAN_PWM_CHANNEL, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
  ledcAttachPin(PIN_FAN_PWM, FAN_PWM_CHANNEL);
  ledcWrite(FAN_PWM_CHANNEL, 0); // Wentylator wyłączony na starcie
  Serial.println("DEBUG: PWM Wentylatora skonfigurowane.");

  // --- Inicjalizacja czujnika temperatury ---
  tempSensor.begin();
  Serial.println("DEBUG: Czujnik DS18B20 uruchomiony.");
  
  // --- Konfiguracja I2S ---
  Serial.println("DEBUG: Konfiguracja I2S...");
  i2s_pin_config_t my_i2s_pins = { .bck_io_num = PIN_I2S_BCLK, .ws_io_num = PIN_I2S_LRC, .data_out_num = PIN_I2S_DATA, .data_in_num = I2S_PIN_NO_CHANGE };
  a2dp_sink.set_pin_config(my_i2s_pins); Serial.println("DEBUG: I2S OK.");
  
  // --- Rejestracja callbacków ---
  Serial.println("DEBUG: Rejestracja callbacków...");
  a2dp_sink.set_on_connection_state_changed(connection_state_changed);
  a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);
  a2dp_sink.set_stream_reader(stream_reader_callback); 
  Serial.println("DEBUG: Callbacki OK.");
  
 // Wewnątrz setup()

// === ZMIANA: Używamy nazwy i hasła z config.h ===
Serial.print("DEBUG: Uruchamianie A2DP z nazwą: "); 
Serial.println(BLUETOOTH_DEVICE_NAME); 

a2dp_sink.start(BLUETOOTH_DEVICE_NAME); 

Serial.println("DEBUG: A2DP Uruchomione. Czekam na połączenie.");
}


// --- LOOP ---
void loop() {
  unsigned long now = millis();

  // --- Logika timera auto-wyciszenia/standby ---
  if (standbyTimer != 0 && (now - standbyTimer > standbyDelay)) {
    if (currentAmpState == AMP_ON) {
      Serial.println("DEBUG: 30 sekund bez danych audio. Przechodzę w MUTE.");
      setAmpState(AMP_MUTE);
      standbyTimer = millis(); 
    }
    else if (currentAmpState == AMP_MUTE) {
      Serial.println("DEBUG: 30 sekund w stanie MUTE. Przechodzę w STANDBY (AMP_OFF).");
      setAmpState(AMP_OFF);
      standbyTimer = 0; 
    }
  }

  // --- Logika sprawdzania temperatury ---
  if (now - lastTempCheck > TEMP_CHECK_INTERVAL) {
    lastTempCheck = now;
    handleFanControl(); 
  }
}