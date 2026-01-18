/*
 * PoolArduino_2025_1_patched.ino
 * Arduino UNO R4 WiFi Sketch für pH, ORP, RTD, analoge Sensoren,
 * MQTT-Publishing und 16x2 LCD (D1 Robot LCD Keypad Shield, parallel).
 * Erweiterte Statusanzeige mit Tastensteuerung (kein Menü).
 * Modus 1: pH/ORP/RTD, Modus 2: Analoge Werte, Modus 3: MQTT-Status, Modus 4: Version.
 * Optimierter I2C-Scan für bekannte Adressen (0x63, 0x66, 0x62).
 * Button-Thresholds: Right (~0), Up (~132), Down (~307), Left (~479), Select (~717).
 * Fixes: Separate Backoff-Zustände für WiFi/MQTT, sichere strncpy, RSSI-Guards,
 *        optional reduzierte Subscriptions, größere MQTT-Puffer, Button-Debounce.
 *        ORP als Integer mV, LCD-Backlight-Toggle (Left), A2-Label, WDT während MQTT-Flush,
 *        I2C-Timeout/Clock robuster, automatische I2C/EZO-Recovery (A/B/C).
 * Neu: DAC (GP8211S) 0–10V über MQTT auf Wire1 (Qwiic).
 * Neu: Versionsanzeige (v1.3) bei Start und in Modus 4 (Right-Taste).
 * Autor: Martin (+ Patches)
 * Datum: Juni 2025 / Patch: Sept 2025
 */
#include <Wire.h>
#include <Ezo_i2c.h>
#include <sequencer2.h>
#include <WiFiS3.h>
#include <WDT.h>
#include <ArduinoMqttClient.h>
#include <CircularBuffer.hpp>
#include <LiquidCrystal.h>
#include <DFRobot_GP8XXX.h>   // <--- DAC
#include "config.h"
#include "lcd.h"
#include "limits.h"

// D1 Robot LCD Keypad Shield (parallel LCD)
#define USE_D1_SHIELD

#define INITIAL_RECONNECT_DELAY 5000
#define MAX_BUFFER_SIZE 50
#define MAX_BACKOFF_DELAY 60000
#define WATCHDOG_TIMEOUT 8000
#define DEBUG_MODE 1
#define DHCP_TIMEOUT 30000
#define I2C_TIMEOUT_MS 200 // robusteres Timeout für I2C
#define SKETCH_VERSION "v1.3" // Versionsnummer des Sketches

// OPTIONAL: Sensor-Power per MOSFET/Enable (Stage C Recovery).
// -> Auskommentiert lassen, wenn nicht verdrahtet.
// #define SENSOR_PWR_PIN 9
// #define SENSOR_PWR_ACTIVE_HIGH 1

// Anzeigemodi
#define DISPLAY_MODE_SENSOR 1   // pH, ORP, RTD
#define DISPLAY_MODE_ANALOG 2   // A2, A1
#define DISPLAY_MODE_MQTT 3     // MQTT-Status
#define DISPLAY_MODE_VERSION 4  // Versionsanzeige

void step1();
void step2();
void idleStep();
void updateMQTT();
void updateDisplay();
void onMqttMessage(int messageSize);

bool wifiConnected = false;
bool mqttConnected = false;

// GETRENNT: Backoff-Zustände für WiFi/MQTT
unsigned long lastWiFiReconnectAttempt = 0;
uint8_t wifiReconnectAttemptCount = 0;
unsigned long lastMQTTReconnectAttempt = 0;
uint8_t mqttReconnectAttemptCount = 0;

char lastError[16] = "";
int displayMode = DISPLAY_MODE_SENSOR; // Standard: Sensorwerte
bool mqttPaused = false;

struct SensorReadings {
  float ph = 0.0;
  float rtd = 0.0;
  float orp = 0.0;
  int analog0 = 0;
  int analog1 = 0;
  unsigned long timestamp = 0;
  bool phValid = false;
  bool rtdValid = false;
  bool orpValid = false;
  bool analog0Valid = false;
  bool analog1Valid = false;
} lastReadings;

// Größer dimensionierter MQTT-Puffer
struct MQTTMessage {
  char topic[64];
  char payload[24];
};
CircularBuffer<MQTTMessage, MAX_BUFFER_SIZE> mqttBuffer;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const int analogPin0 = A2; // A2 für analogen Sensor 0
const int analogPin1 = A1; // A1 für analogen Sensor 1
const int buttonPin = A0;  // D1 Shield Buttons

Ezo_board ph = Ezo_board(99, "PH");      // 0x63
Ezo_board rtd = Ezo_board(102, "TEMP");  // 0x66
Ezo_board orp = Ezo_board(98, "ORP");    // 0x62

Sequencer2 Seq(&step1, 5000, &step2, 5000);
Sequencer2 SeqDisplay(&updateDisplay, 5000, &idleStep, 0);
Sequencer2 SeqMQTT(&idleStep, 25000, &updateMQTT, 5000);

void flushMQTTBuffer();
void logMessage(const char* message);
void logDebug(const char* message);
bool connectWiFi();
void reconnectWiFi();
void reconnectMQTT();
void subscribeMQTTTopics();
void scanI2C();

// Recovery-Helpers (EZO / Wire)
void i2cBusUnstick();        // Stage A
void i2cReinit();            // Stage A helper
void sensorNudgeSleepWake(); // Stage B
void sensorPowerCycle();     // Stage C (optional)

// Button-Debounce / Hysterese
int lastStableButton = 1023;
int stableCount = 0;
const int BUTTON_BAND = 20;           // ADC Toleranz
const int BUTTON_STABLE_READS = 2;    // 2 * 500ms = ~1s stabil
bool lcdBacklightOn = true;

// Fehlerüberwachung für EZO/I2C (Wire)
uint8_t rtdConsecErr = 0;
uint8_t orpConsecErr = 0;
const uint8_t ERR_THRESH_STAGE_A = 3; // ~15 s
const uint8_t ERR_THRESH_STAGE_B = 6; // ~30 s
const uint8_t ERR_THRESH_STAGE_C = 9; // ~45 s
unsigned long lastRecoveryActionMs = 0;
const unsigned long RECOVERY_COOLDOWN = 10000; // 10 s Schutzzeit

// ------------------- DAC (GP8211S) on Wire1 -------------------
DFRobot_GP8211S dac;   // Standard-Konstruktor
bool dacReady = false;
float dacSetVoltage = 0.0f;  // last applied (0..10)
unsigned long lastDacPublishMs = 0;
const unsigned long DAC_STATE_PUBLISH_INTERVAL = 60000; // 60s

void initDAC();
bool setDacVoltage(float v);
uint16_t voltageToDacRaw(float v);
float parseVoltagePayload(String s);
void publishDacState(bool force);
// -------------------------------------------------------------

void setup() {
  Wire.begin(); // Für Sensoren (pH, RTD, ORP) auf Wire
  Wire.setTimeout(I2C_TIMEOUT_MS);
  Wire.setClock(100000); // konservative I2C-Clock

  Serial.begin(115200);
  delay(1000);
  Serial.flush();

  if (R_SYSTEM->RSTSR0 & (1 << 1)) {
    logMessage("Watchdog-Reset erkannt");
    R_SYSTEM->RSTSR0 = 0;
  }

  logMessage("Starte...");
  // Zeige Versionsnummer bei Start
  if (lcdInitialized) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("PoolArduino");
    lcd.setCursor(0, 1);
    lcd.print(SKETCH_VERSION);
    delay(1000);
  }

  WDT.begin(WATCHDOG_TIMEOUT);
  logDebug("Watchdog initialisiert");

  // Explizite Initialisierung von D10 für LCD-Backlight
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  lcdBacklightOn = true;

  initLCD();
  if (lcdInitialized) {
    logDebug("D1 LCD initialisiert");
  } else {
    logMessage("D1 LCD-Init fehlgeschlagen");
  }

#ifdef SENSOR_PWR_PIN
  pinMode(SENSOR_PWR_PIN, OUTPUT);
  #if SENSOR_PWR_ACTIVE_HIGH
    digitalWrite(SENSOR_PWR_PIN, HIGH);
  #else
    digitalWrite(SENSOR_PWR_PIN, LOW);
  #endif
#endif

  scanI2C();   // Sensoren (Wire)
  initDAC();   // DAC (Wire1)

  Seq.reset();
  SeqDisplay.reset();
  SeqMQTT.reset();

  wifiConnected = connectWiFi();
  if (!wifiConnected) {
    strncpy(lastError, "WiFi fehlgeschlagen", sizeof(lastError) - 1); lastError[sizeof(lastError) - 1] = '\0';
    logMessage(lastError);
  } else {
    mqttClient.setId(mqtt_client_id);
    reconnectMQTT();
    logDebug("MQTT-Setup abgeschlossen");
    strncpy(lastError, "Setup abgeschlossen", sizeof(lastError) - 1); lastError[sizeof(lastError) - 1] = '\0';
    logMessage(lastError);
  }
}

void scanI2C() {
  logDebug("Scanne I2C-Bus (bekannte Adressen)...");
  byte error;
  int nDevices = 0;
  const byte addresses[] = {99, 102, 98}; // pH, RTD, ORP
  const char* names[] = {"pH", "RTD", "ORP"};

  for (int i = 0; i < 3; i++) {
    byte address = addresses[i];
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      char msg[32];
      snprintf(msg, sizeof(msg), "I2C-Gerät %s bei 0x%02X", names[i], address);
      logDebug(msg);
      nDevices++;
    } else {
      char msg[32];
      snprintf(msg, sizeof(msg), "I2C-Fehler %s bei 0x%02X: %d", names[i], address, error);
      logDebug(msg);
      if (error == 5) {
        logMessage("I2C-Timeout: Prüfe Pullups");
      }
    }
    WDT.refresh();
  }

  if (nDevices == 0) {
    logMessage("Keine I2C-Geräte gefunden");
    logMessage("Prüfe Pullup-Widerstände");
  } else {
    char msg[32];
    snprintf(msg, sizeof(msg), "I2C-Scan: %d Geräte gefunden", nDevices);
    logDebug(msg);
  }
}

bool connectWiFi() {
  const int maxRetries = 3;
  int retryCount = 0;

  while (retryCount < maxRetries) {
    logDebug("Verbinde mit WiFi...");
    logMessage("WiFi verbinden...");
    WiFi.begin(ssid, pass);
    delay(1000);
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < DHCP_TIMEOUT) {
      delay(500);
      Serial.print(".");
      WDT.refresh();
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress(0, 0, 0, 0)) {
      logDebug("WiFi verbunden");
      Serial.print("IP: "); Serial.println(WiFi.localIP());
      Serial.print("MAC: ");
      uint8_t mac[6];
      WiFi.macAddress(mac);
      Serial.print(mac[0], HEX); Serial.print(":");
      Serial.print(mac[1], HEX); Serial.print(":");
      Serial.print(mac[2], HEX); Serial.print(":");
      Serial.print(mac[3], HEX); Serial.print(":");
      Serial.print(mac[4], HEX); Serial.print(":");
      Serial.println(mac[5], HEX);
      Serial.print("RSSI: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
      logMessage("WiFi verbunden");
      wifiConnected = true;
      return true;
    }

    Serial.print("WiFi fehlgeschlagen, Status: ");
    Serial.println(WiFi.status());
    logDebug("WiFi-Verbindung fehlgeschlagen");
    logMessage("WiFi fehlgeschlagen");
    WiFi.disconnect();
    delay(1000);
    retryCount++;
  }

  logMessage("WiFi-Verbindung fehlgeschlagen");
  wifiConnected = false;
  return false;
}

void reconnectWiFi() {
  if (wifiConnected) return;

  unsigned long delayTime = min((unsigned long)INITIAL_RECONNECT_DELAY * (1UL << wifiReconnectAttemptCount), (unsigned long)MAX_BACKOFF_DELAY);
  if (millis() - lastWiFiReconnectAttempt < delayTime) return;

  lastWiFiReconnectAttempt = millis();
  wifiReconnectAttemptCount = min<uint8_t>(wifiReconnectAttemptCount + 1, 10);

  logDebug("Versuche WiFi-Wiederverbindung...");
  logMessage("WiFi verbindet wieder...");

  WiFi.disconnect();
  wifiConnected = connectWiFi();

  if (wifiConnected) {
    wifiReconnectAttemptCount = 0;
    long rssi = WiFi.RSSI();
    char rssiMsg[48];
    snprintf(rssiMsg, sizeof(rssiMsg), "WiFi wiederverbunden, RSSI: %ld dBm", rssi);
    logMessage(rssiMsg);
  }
}

void reconnectMQTT() {
  if (!wifiConnected || WiFi.localIP() == IPAddress(0, 0, 0, 0)) return;

  unsigned long delayTime = min((unsigned long)INITIAL_RECONNECT_DELAY * (1UL << mqttReconnectAttemptCount), (unsigned long)MAX_BACKOFF_DELAY);
  if (millis() - lastMQTTReconnectAttempt < delayTime) return;

  lastMQTTReconnectAttempt = millis();
  mqttReconnectAttemptCount = min<uint8_t>(mqttReconnectAttemptCount + 1, 10);

  logDebug("Verbinde mit MQTT...");
  logMessage("MQTT verbindet wieder...");

  mqttClient.setId(mqtt_client_id);
  mqttClient.setUsernamePassword(mqtt_user, mqtt_password);

  Serial.print("WiFi-Status: "); Serial.print(WiFi.status());
  Serial.print(", IP: "); Serial.println(WiFi.localIP());
  Serial.print("RSSI: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");

  if (mqttClient.connect(broker, port)) {
    logDebug("MQTT verbunden");
    mqttConnected = true;
    mqttClient.onMessage(onMqttMessage);   // << NEW: callback
    subscribeMQTTTopics();
    flushMQTTBuffer();
    mqttReconnectAttemptCount = 0;
    strncpy(lastError, "MQTT verbunden", sizeof(lastError) - 1); lastError[sizeof(lastError) - 1] = '\0';
    logMessage(lastError);

    // publish DAC state once after MQTT reconnect
    publishDacState(true);
  } else {
    logDebug("MQTT-Verbindung fehlgeschlagen");
    Serial.print("Fehlercode: ");
    Serial.println(mqttClient.connectError());
    mqttConnected = false;
    strncpy(lastError, "MQTT fehlgeschlagen", sizeof(lastError) - 1); lastError[sizeof(lastError) - 1] = '\0';
    logMessage(lastError);
  }
}

void subscribeMQTTTopics() {
  mqttClient.subscribe(topic_cal);      // existing control topic
  mqttClient.subscribe(topic_dac_set);  // NEW: DAC set voltage
  logDebug("Abonnierte MQTT-Themen (control + dac)");
}

void loop() {
  static unsigned long lastLoop = 0;
  if (millis() - lastLoop < 10) return;
  lastLoop = millis();

  WDT.refresh();

  // Button-Handling mit Debounce/Hysterese
  static unsigned long lastButtonCheck = 0;
  if (millis() - lastButtonCheck >= 500) {
    int raw = analogRead(buttonPin);
    if (DEBUG_MODE) {
      Serial.print("Button-Rohwert: ");
      Serial.println(raw);
    }

    if (abs(raw - lastStableButton) <= BUTTON_BAND) {
      stableCount++;
    } else {
      stableCount = 0;
      lastStableButton = raw;
    }

    if (stableCount >= BUTTON_STABLE_READS) {
      int buttonValue = lastStableButton;
      if (buttonValue < 25) {
        logMessage("Button: Right");
        displayMode = DISPLAY_MODE_VERSION;
        logDebug("Modus: Version");
        updateDisplay();
      } else if (buttonValue < 200) {
        logMessage("Button: Up");
        displayMode = DISPLAY_MODE_SENSOR;
        logDebug("Modus: Sensorwerte");
        updateDisplay();
      } else if (buttonValue < 400) {
        logMessage("Button: Down");
        displayMode = DISPLAY_MODE_ANALOG;
        logDebug("Modus: Analoge Werte");
        updateDisplay();
      } else if (buttonValue < 600) {
        logMessage("Button: Left");
        lcdBacklightOn = !lcdBacklightOn;
        digitalWrite(10, lcdBacklightOn ? HIGH : LOW);
        logDebug(lcdBacklightOn ? "LCD Backlight: ON" : "LCD Backlight: OFF");
      } else if (buttonValue < 900) {
        logMessage("Button: Select");
        displayMode = DISPLAY_MODE_MQTT;
        mqttPaused = !mqttPaused;
        logMessage(mqttPaused ? "MQTT pausiert" : "MQTT fortgesetzt");
        logDebug("Modus: MQTT-Status");
        updateDisplay();
      }
      stableCount = 0;
    }
    lastButtonCheck = millis();
  }

  if (WiFi.status() != WL_CONNECTED || WiFi.localIP() == IPAddress(0, 0, 0, 0)) {
    wifiConnected = false;
    mqttConnected = false;
    reconnectWiFi();
  } else {
    wifiConnected = true;
  }

  if (wifiConnected && !mqttClient.connected()) {
    mqttConnected = false;
    reconnectMQTT();
  } else if (wifiConnected && mqttClient.connected()) {
    mqttConnected = true;
    mqttClient.poll();
  }

  Seq.run();
  SeqDisplay.run();
  SeqMQTT.run();

  // Periodic DAC state publish (keeps HA in sync)
  publishDacState(false);

  static unsigned long lastStatusUpdate = 0;
  if (millis() - lastStatusUpdate > 30000) {
    char statusMsg[40];
    long rssi = wifiConnected ? WiFi.RSSI() : 0;
    snprintf(statusMsg, sizeof(statusMsg), "WiFi:%s MQTT:%s RSSI:%ld",
             wifiConnected ? "OK" : "ERR",
             mqttConnected ? "OK" : "ERR",
             rssi);
    logMessage(statusMsg);
    char lcdMsg[32];
    snprintf(lcdMsg, sizeof(lcdMsg), "LCD Initialisiert: %d", lcdInitialized);
    logDebug(lcdMsg);
    lastStatusUpdate = millis();
  }
}

void step1() {
  rtd.send_read_cmd();
  if (rtd.get_error() != Ezo_board::SUCCESS) {
    rtdConsecErr = (rtdConsecErr < 250) ? rtdConsecErr + 1 : 250;
    char errMsg[32];
    snprintf(errMsg, sizeof(errMsg), "RTD-Fehler, Code: %d", rtd.get_error());
    Serial.println(errMsg);
    logMessage("RTD-Fehler");
    strncpy(lastError, "RTD-Fehler", sizeof(lastError) - 1); lastError[sizeof(lastError) - 1] = '\0';
    lastReadings.rtdValid = false;
  } else {
    rtdConsecErr = 0;
    float temp = rtd.get_last_received_reading();
    if (temp > -1000.0) {
      ph.send_read_with_temp_comp(temp);
    } else {
      ph.send_read_with_temp_comp(25.0);
      logMessage("RTD ungültig");
      strncpy(lastError, "RTD ungültig", sizeof(lastError) - 1); lastError[sizeof(lastError) - 1] = '\0';
      lastReadings.rtdValid = false;
    }
  }

  if (ph.get_error() != Ezo_board::SUCCESS) {
    char errMsg[32];
    snprintf(errMsg, sizeof(errMsg), "PH-Fehler, Code: %d", ph.get_error());
    Serial.println(errMsg);
    logMessage("PH-Fehler");
    strncpy(lastError, "PH-Fehler", sizeof(lastError) - 1); lastError[sizeof(lastError) - 1] = '\0';
    lastReadings.phValid = false;
  }

  orp.send_read_cmd();
  if (orp.get_error() != Ezo_board::SUCCESS) {
    orpConsecErr = (orpConsecErr < 250) ? orpConsecErr + 1 : 250;
    char errMsg[32];
    snprintf(errMsg, sizeof(errMsg), "ORP-Fehler, Code: %d", orp.get_error());
    Serial.println(errMsg);
    logMessage("ORP-Fehler");
    strncpy(lastError, "ORP-Fehler", sizeof(lastError) - 1); lastError[sizeof(lastError) - 1] = '\0';
    lastReadings.orpValid = false;
  } else {
    orpConsecErr = 0;
  }
}

void step2() {
  ph.receive_read_cmd();
  if (ph.get_error() != Ezo_board::SUCCESS) {
    char errMsg[32];
    snprintf(errMsg, sizeof(errMsg), "PH-Fehler, Code: %d", ph.get_error());
    Serial.println(errMsg);
    logMessage("PH-Fehler");
    strncpy(lastError, "PH-Fehler", sizeof(lastError) - 1); lastError[sizeof(lastError) - 1] = '\0';
    lastReadings.phValid = false;
  } else {
    lastReadings.ph = ph.get_last_received_reading();
    lastReadings.phValid = (lastReadings.ph >= PH_MIN && lastReadings.ph <= PH_MAX);
    if (!lastReadings.phValid) {
      logMessage("PH ungültig");
      strncpy(lastError, "PH ungültig", sizeof(lastError) - 1); lastError[sizeof(lastError) - 1] = '\0';
    }
  }

  rtd.receive_read_cmd();
  if (rtd.get_error() != Ezo_board::SUCCESS) {
    rtdConsecErr = (rtdConsecErr < 250) ? rtdConsecErr + 1 : 250;
    char errMsg[32];
    snprintf(errMsg, sizeof(errMsg), "RTD-Fehler, Code: %d", rtd.get_error());
    Serial.println(errMsg);
    logMessage("RTD-Fehler");
    strncpy(lastError, "RTD-Fehler", sizeof(lastError) - 1); lastError[sizeof(lastError) - 1] = '\0';
    lastReadings.rtdValid = false;
  } else {
    rtdConsecErr = 0;
    lastReadings.rtd = rtd.get_last_received_reading();
    lastReadings.rtdValid = (lastReadings.rtd >= RTD_MIN && lastReadings.rtd <= RTD_MAX);
    if (!lastReadings.rtdValid) {
      logMessage("RTD ungültig");
      strncpy(lastError, "RTD ungültig", sizeof(lastError) - 1); lastError[sizeof(lastError) - 1] = '\0';
    }
  }

  orp.receive_read_cmd();
  if (orp.get_error() != Ezo_board::SUCCESS) {
    orpConsecErr = (orpConsecErr < 250) ? orpConsecErr + 1 : 250;
    char errMsg[32];
    snprintf(errMsg, sizeof(errMsg), "ORP-Fehler, Code: %d", orp.get_error());
    Serial.println(errMsg);
    logMessage("ORP-Fehler");
    strncpy(lastError, "ORP-Fehler", sizeof(lastError) - 1); lastError[sizeof(lastError) - 1] = '\0';
    lastReadings.orpValid = false;
  } else {
    orpConsecErr = 0;
    lastReadings.orp = orp.get_last_received_reading();
    lastReadings.orpValid = (lastReadings.orp >= ORP_MIN && lastReadings.orp <= ORP_MAX);
    if (!lastReadings.orpValid) {
      logMessage("ORP ungültig");
      strncpy(lastError, "ORP ungültig", sizeof(lastError) - 1); lastError[sizeof(lastError) - 1] = '\0';
    }
  }

  lastReadings.timestamp = millis();

  // Recovery-Manager (einmal pro step2-Runde) - nur für Wire/EZO
  uint8_t worst = (rtdConsecErr > orpConsecErr) ? rtdConsecErr : orpConsecErr;
  if (worst >= ERR_THRESH_STAGE_A && millis() - lastRecoveryActionMs > RECOVERY_COOLDOWN) {
    if (worst < ERR_THRESH_STAGE_B) {
      logDebug("RECOVERY A: I2C Bus Unstick + Reinit");
      i2cBusUnstick();
      i2cReinit();
      lastRecoveryActionMs = millis();
    } else if (worst < ERR_THRESH_STAGE_C) {
      logDebug("RECOVERY B: EZO Sleep/Wake");
      sensorNudgeSleepWake();
      lastRecoveryActionMs = millis();
    } else {
      logDebug("RECOVERY C: Sensor Power Cycle");
      sensorPowerCycle();
      lastRecoveryActionMs = millis();
      rtdConsecErr = 0;
      orpConsecErr = 0;
    }
  }
}

void updateDisplay() {
  char statusMsg[16];
  snprintf(statusMsg, sizeof(statusMsg), "W:%s M:%s",
           wifiConnected ? "OK" : "ERR",
           mqttConnected ? "OK" : "ERR");

  char debugMsg[64];
  snprintf(debugMsg, sizeof(debugMsg), "LCD-Status: %s", statusMsg);
  logDebug(debugMsg);

  if (!lcdInitialized) {
    logDebug("Überspringe LCD-Update: nicht initialisiert");
    return;
  }

  lcd.clear();
  char line1[17], line2[17];

  if (displayMode == DISPLAY_MODE_SENSOR) {
    snprintf(line1, sizeof(line1), "pH:%.2f T:%.1fC", lastReadings.ph, lastReadings.rtd);
    snprintf(line2, sizeof(line2), "ORP:%.0fmV %s", lastReadings.orp, statusMsg);
  } else if (displayMode == DISPLAY_MODE_ANALOG) {
    snprintf(line1, sizeof(line1), "A2:%d A1:%d", lastReadings.analog0, lastReadings.analog1);
    snprintf(line2, sizeof(line2), "%s", statusMsg);
  } else if (displayMode == DISPLAY_MODE_MQTT) {
    long rssi = wifiConnected ? WiFi.RSSI() : 0;
    snprintf(line1, sizeof(line1), "MQTT:%s", mqttPaused ? "Pausiert" : "Aktiv");
    snprintf(line2, sizeof(line2), "RSSI:%ld dBm %s", rssi, statusMsg);
  } else if (displayMode == DISPLAY_MODE_VERSION) {
    snprintf(line1, sizeof(line1), "Sketch:%s", SKETCH_VERSION);
    snprintf(line2, sizeof(line2), "%s", statusMsg);
  }

  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void updateMQTT() {
  if (mqttPaused) {
    logDebug("MQTT pausiert, überspringe Veröffentlichung");
    return;
  }

  char buffer[12];
  MQTTMessage msg;

  auto publishOrBuffer = [&](const char* topic, const char* payload) {
    if (mqttConnected) {
      mqttClient.beginMessage(topic);
      mqttClient.print(payload);
      mqttClient.endMessage();
      char debugMsg[80];
      snprintf(debugMsg, sizeof(debugMsg), "Veröffentliche auf %s: %s", topic, payload);
      logDebug(debugMsg);
    } else if (!mqttBuffer.isFull()) {
      strncpy(msg.topic, topic, sizeof(msg.topic) - 1); msg.topic[sizeof(msg.topic) - 1] = '\0';
      strncpy(msg.payload, payload, sizeof(msg.payload) - 1); msg.payload[sizeof(msg.payload) - 1] = '\0';
      mqttBuffer.push(msg);
    } else {
      logDebug("MQTT-Puffer voll");
    }
  };

  if (lastReadings.phValid) {
    dtostrf(lastReadings.ph, 6, 2, buffer);
    publishOrBuffer(topic_ph, buffer);
  } else {
    logDebug("Überspringe MQTT-Veröffentlichung für sensor2/ph: ungültige Daten");
  }

  if (lastReadings.orpValid) {
    snprintf(buffer, sizeof(buffer), "%d", (int)lastReadings.orp);
    publishOrBuffer(topic_orp, buffer);
  } else {
    logDebug("Überspringe MQTT-Veröffentlichung für sensor2/orp: ungültige Daten");
  }

  if (lastReadings.rtdValid) {
    dtostrf(lastReadings.rtd, 6, 2, buffer);
    publishOrBuffer(topic_rtd, buffer);
  } else {
    logDebug("Überspringe MQTT-Veröffentlichung für sensor2/rtd: ungültige Daten");
  }

  lastReadings.analog0 = analogRead(analogPin0);
  lastReadings.analog1 = analogRead(analogPin1);
  lastReadings.analog0Valid = (lastReadings.analog0 >= ANALOG_MIN && lastReadings.analog0 <= ANALOG_MAX);
  lastReadings.analog1Valid = (lastReadings.analog1 >= ANALOG_MIN && lastReadings.analog1 <= ANALOG_MAX);

  if (!lastReadings.analog0Valid || !lastReadings.analog1Valid) {
    logMessage("Analog ungültig");
    strncpy(lastError, "Analog ungültig", sizeof(lastError) - 1); lastError[sizeof(lastError) - 1] = '\0';
  }

  if (lastReadings.analog0Valid) {
    snprintf(buffer, sizeof(buffer), "%d", lastReadings.analog0);
    publishOrBuffer(topic_a0, buffer);
  } else {
    logDebug("Überspringe MQTT-Veröffentlichung für sensor2/a0: ungültige Daten");
  }

  if (lastReadings.analog1Valid) {
    snprintf(buffer, sizeof(buffer), "%d", lastReadings.analog1);
    publishOrBuffer(topic_a1, buffer);
  } else {
    logDebug("Überspringe MQTT-Veröffentlichung für sensor2/a1: ungültige Daten");
  }
}

void flushMQTTBuffer() {
  static unsigned long lastPublish = 0;
  uint8_t sent = 0;
  while (!mqttBuffer.isEmpty() && mqttConnected && millis() - lastPublish >= 50 && sent < 20) {
    MQTTMessage msg = mqttBuffer.shift();
    mqttClient.beginMessage(msg.topic);
    mqttClient.print(msg.payload);
    mqttClient.endMessage();
    lastPublish = millis();
    sent++;
    WDT.refresh();
  }
}

void logMessage(const char* message) {
  Serial.println(message);
  Serial.flush();
  if (lcdInitialized) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Status:");
    lcd.setCursor(0, 1);
    lcd.print(message);
    delay(600);
    updateDisplay();
  }
}

void logDebug(const char* message) {
  if (DEBUG_MODE) {
    Serial.println(message);
    Serial.flush();
  }
}

void idleStep() {}

// --- Recovery Helpers (Wire / EZO) ---
void i2cReinit() {
  Wire.end();
  delay(5);
  Wire.begin();
  Wire.setTimeout(I2C_TIMEOUT_MS);
  Wire.setClock(100000);
}

void i2cBusUnstick() {
  pinMode(SCL, OUTPUT);
  pinMode(SDA, INPUT_PULLUP);
  for (uint8_t i = 0; i < 9; i++) {
    digitalWrite(SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(SCL, LOW);
    delayMicroseconds(5);
  }
  digitalWrite(SCL, HIGH);
  delayMicroseconds(5);
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, HIGH);
  delayMicroseconds(5);
  pinMode(SDA, INPUT_PULLUP);
}

void sensorNudgeSleepWake() {
  ph.send_cmd("Sleep");
  orp.send_cmd("Sleep");
  rtd.send_cmd("Sleep");
  delay(50);
  i2cReinit();
  ph.send_cmd("Status,?");
  orp.send_cmd("Status,?");
  rtd.send_cmd("Status,?");
}

void sensorPowerCycle() {
#ifdef SENSOR_PWR_PIN
  #if SENSOR_PWR_ACTIVE_HIGH
    digitalWrite(SENSOR_PWR_PIN, LOW);
  #else
    digitalWrite(SENSOR_PWR_PIN, HIGH);
  #endif
  delay(300);
  #if SENSOR_PWR_ACTIVE_HIGH
    digitalWrite(SENSOR_PWR_PIN, HIGH);
  #else
    digitalWrite(SENSOR_PWR_PIN, LOW);
  #endif
  delay(500);
  i2cReinit();
#else
  logDebug("SensorPowerCycle übersprungen (SENSOR_PWR_PIN nicht definiert)");
#endif
}

// ------------------- DAC helpers (Wire1) -------------------
uint16_t voltageToDacRaw(float v) {
  if (v < 0.0f) v = 0.0f;
  if (v > 10.0f) v = 10.0f;
  float ratio = v / 10.0f;
  uint32_t raw = (uint32_t)(ratio * 32767.0f + 0.5f);
  if (raw > 32767) raw = 32767;
  return (uint16_t)raw;
}

float parseVoltagePayload(String s) {
  s.trim();
  s.replace(",", "."); // erlaubt 7,5
  int eq = s.indexOf('=');
  if (eq >= 0) s = s.substring(eq + 1);
  int col = s.indexOf(':');
  if (col >= 0) s = s.substring(col + 1);
  s.trim();
  return s.toFloat();
}

bool setDacVoltage(float v) {
  if (!dacReady) return false;
  if (v < 0.0f) v = 0.0f;
  if (v > 10.0f) v = 10.0f;

  uint16_t raw = voltageToDacRaw(v);
  dac.setDACOutRange(dac.eOutputRange10V);
  dac.setDACOutVoltage(raw);
  dacSetVoltage = v;
  return true;
}

void publishDacState(bool force) {
  if (!mqttClient.connected()) return;   // statt mqttConnected
  if (!force && (millis() - lastDacPublishMs < DAC_STATE_PUBLISH_INTERVAL)) return;

  lastDacPublishMs = millis();

  char buf[16];
  dtostrf(dacSetVoltage, 0, 2, buf);

  mqttClient.beginMessage(topic_dac_state);
  mqttClient.print(buf);
  mqttClient.endMessage();

  if (DEBUG_MODE) {
    char msg[64];
    snprintf(msg, sizeof(msg), "DAC state: %s V", buf);
    logDebug(msg);
  }
}

void initDAC() {
  // DAC on Wire1 (Qwiic)
  Wire1.begin();
  dac._pWire = &Wire1;

  const uint8_t tries = 5;
  for (uint8_t i = 0; i < tries; i++) {
    if (dac.begin() == 0) {
      dacReady = true;
      break;
    }
    logMessage("DAC I2C Fehler");
    delay(300);
    WDT.refresh();
  }

  if (!dacReady) {
    logMessage("DAC nicht bereit");
    return;
  }

  dac.setDACOutRange(dac.eOutputRange10V);
  setDacVoltage(0.0f); // safe default

  logMessage("DAC bereit");
  if (DEBUG_MODE) logDebug("GP8211S DAC ready (Wire1, 0-10V)");
}
// ----------------------------------------------------------
void onMqttMessage(int messageSize) {
  String topic = mqttClient.messageTopic();
  String message;

  while (mqttClient.available()) {
    message += (char)mqttClient.read();
  }

  // HARD DEBUG: always print what arrived
  Serial.print("[MQTT RX] topic='");
  Serial.print(topic);
  Serial.print("' payload='");
  Serial.print(message);
  Serial.println("'");

  // CAL
  if (topic.equals(topic_cal)) {
    if (message == "cal,mid,7") {
      ph.send_cmd("Cal,mid,6.86");
      logMessage(ph.get_error() == Ezo_board::SUCCESS ? "pH Mitte Kal OK" : "pH Mitte Kal Fehl");
    } else if (message == "cal,low,4") {
      ph.send_cmd("Cal,low,4.01");
      logMessage(ph.get_error() == Ezo_board::SUCCESS ? "pH Tief Kal OK" : "pH Tief Kal Fehl");
    } else if (message == "cal,high,10") {
      ph.send_cmd("Cal,high,9.18");
      logMessage(ph.get_error() == Ezo_board::SUCCESS ? "pH Hoch Kal OK" : "pH Hoch Kal Fehl");
    } else if (message == "cal,orp,256") {
      orp.send_cmd("Cal,256");
      logMessage(orp.get_error() == Ezo_board::SUCCESS ? "ORP Kal OK" : "ORP Kal Fehl");
    } else if (message == "cal,clear") {
      ph.send_cmd("Cal,clear");
      orp.send_cmd("Cal,clear");
      logMessage((ph.get_error() == Ezo_board::SUCCESS && orp.get_error() == Ezo_board::SUCCESS) ? "Kal Gelöscht OK" : "Kal Löschen Fehl");
    }
    return;
  }

  // DAC
  if (topic.equals(topic_dac_set)) {
    float v = parseVoltagePayload(message);
    if (v < 0.0f) v = 0.0f;
    if (v > 10.0f) v = 10.0f;

    if (!dacReady) {
      logMessage("DAC nicht bereit");
      return;
    }

    if (setDacVoltage(v)) {
      Serial.print("[DAC] Applied: ");
      Serial.println(dacSetVoltage, 2);
      logMessage("DAC gesetzt");
      publishDacState(true);
    } else {
      logMessage("DAC Fehler");
    }
    return;
  }

  Serial.println("[MQTT RX] topic not handled");
}