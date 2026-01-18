/*
 * PoolArduino_2025_1.ino
 * Arduino UNO R4 WiFi Sketch für pH, ORP, RTD, analoge Sensoren,
 * MQTT-Publishing und 16x2 LCD (D1 Robot LCD Keypad Shield, parallel).
 * Erweiterte Statusanzeige mit Tastensteuerung (kein Menü).
 * Modus 1: pH/ORP/RTD, Modus 2: Analoge Werte, Modus 3: MQTT-Status, Modus 4: Version.
 * Optimierter I2C-Scan für bekannte Adressen (0x63, 0x66, 0x62).
 * Button-Thresholds: Right (~0), Up (~132), Down (~307), Left (~479), Select (~717).
 * Korrektur: SeqDisplay mit zwei Schritten (updateDisplay, idleStep).
 * Änderung: MQTT-Client-ID aus config.h (mqtt_client_id).
 * Neu: Versionsanzeige (v1.2) bei Start und in Modus 4 (Right-Taste).
 * Autor: Martin
 * Datum: Juni 2025
 */
#include <Wire.h>
#include <Ezo_i2c.h>
#include <sequencer2.h>
#include <WiFiS3.h>
#include <WDT.h>
#include <ArduinoMqttClient.h>
#include <CircularBuffer.hpp>
#include <LiquidCrystal.h>
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
#define I2C_TIMEOUT_MS 50 // Timeout für I2C
#define SKETCH_VERSION "v1.2" // Versionsnummer des Sketches

// Anzeigemodi
#define DISPLAY_MODE_SENSOR 1  // pH, ORP, RTD
#define DISPLAY_MODE_ANALOG 2  // A0, A1
#define DISPLAY_MODE_MQTT 3    // MQTT-Status
#define DISPLAY_MODE_VERSION 4  // Versionsanzeige

void step1();
void step2();
void idleStep();
void updateMQTT();
void updateDisplay();

bool wifiConnected = false;
bool mqttConnected = false;
unsigned long lastReconnectAttempt = 0;
uint8_t reconnectAttemptCount = 0;
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

struct MQTTMessage {
    char topic[32];
    char payload[16];
};
CircularBuffer<MQTTMessage, MAX_BUFFER_SIZE> mqttBuffer;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const int analogPin0 = A2; // A2 für analogen Sensor 0
const int analogPin1 = A1; // A1 für analogen Sensor 1
const int buttonPin = A0; // D1 Shield Buttons

Ezo_board ph = Ezo_board(99, "PH");    // 0x63
Ezo_board rtd = Ezo_board(102, "TEMP"); // 0x66
Ezo_board orp = Ezo_board(98, "ORP");  // 0x62

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

void setup() {
    Wire.begin(); // Für Sensoren (pH, RTD, ORP)
    Wire.setTimeout(I2C_TIMEOUT_MS); // Timeout für I2C
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
        delay(1000); // Version 1 Sekunde anzeigen
    }
    WDT.begin(WATCHDOG_TIMEOUT);
    logDebug("Watchdog initialisiert");

    // Explizite Initialisierung von D10 für LCD-Backlight
    pinMode(10, OUTPUT);
    digitalWrite(10, HIGH); // Backlight einschalten
    initLCD();
    if (lcdInitialized) {
        logDebug("D1 LCD initialisiert");
    } else {
        logMessage("D1 LCD-Init fehlgeschlagen");
    }

    scanI2C(); // Für Sensoren
    Seq.reset();
    SeqDisplay.reset();
    SeqMQTT.reset();

    wifiConnected = connectWiFi();
    if (!wifiConnected) {
        strncpy(lastError, "WiFi fehlgeschlagen", sizeof(lastError));
        logMessage(lastError);
    } else {
        mqttClient.setId(mqtt_client_id); // Verwende Client-ID aus config.h
        reconnectMQTT();
        logDebug("MQTT-Setup abgeschlossen");
        strncpy(lastError, "Setup abgeschlossen", sizeof(lastError));
        logMessage(lastError);
    }
}

void scanI2C() {
    logDebug("Scanne I2C-Bus (bekannte Adressen)...");
    byte error;
    int nDevices = 0;
    const byte addresses[] = {99, 102, 98}; // pH: 0x63, RTD: 0x66, ORP: 0x62
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
        WDT.refresh(); // Verhindere Watchdog-Timeout
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
            Serial.print("IP: ");
            Serial.println(WiFi.localIP());
            Serial.print("MAC: ");
            uint8_t mac[6];
            WiFi.macAddress(mac);
            Serial.print(mac[0], HEX); Serial.print(":");
            Serial.print(mac[1], HEX); Serial.print(":");
            Serial.print(mac[2], HEX); Serial.print(":");
            Serial.print(mac[3], HEX); Serial.print(":");
            Serial.print(mac[4], HEX); Serial.print(":");
            Serial.println(mac[5], HEX);
            Serial.print("RSSI: ");
            Serial.print(WiFi.RSSI());
            Serial.println(" dBm");
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
    
    unsigned long delayTime = min((unsigned long)INITIAL_RECONNECT_DELAY * (1UL << reconnectAttemptCount), (unsigned long)MAX_BACKOFF_DELAY);
    if (millis() - lastReconnectAttempt < delayTime) return;
    
    lastReconnectAttempt = millis();
    reconnectAttemptCount = min(reconnectAttemptCount + 1, 10);
    
    logDebug("Versuche WiFi-Wiederverbindung...");
    logMessage("WiFi verbindet wieder...");
    
    WiFi.disconnect();
    wifiConnected = connectWiFi();
    
    if (wifiConnected) {
        reconnectAttemptCount = 0;
        long rssi = WiFi.RSSI();
        char rssiMsg[32];
        snprintf(rssiMsg, sizeof(rssiMsg), "WiFi wiederverbunden, RSSI: %ld dBm", rssi);
        logMessage(rssiMsg);
    }
}

void reconnectMQTT() {
    if (!wifiConnected || WiFi.localIP() == IPAddress(0, 0, 0, 0)) return;
    
    unsigned long delayTime = min((unsigned long)INITIAL_RECONNECT_DELAY * (1UL << reconnectAttemptCount), (unsigned long)MAX_BACKOFF_DELAY);
    if (millis() - lastReconnectAttempt < delayTime) return;
    
    lastReconnectAttempt = millis();
    reconnectAttemptCount = min(reconnectAttemptCount + 1, 10);
    
    logDebug("Verbinde mit MQTT...");
    logMessage("MQTT verbindet wieder...");
    
    Serial.print("WiFi-Status: ");
    Serial.print(WiFi.status());
    Serial.print(", IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    
    if (mqttClient.connect(broker, port)) {
        logDebug("MQTT verbunden");
        mqttConnected = true;
        subscribeMQTTTopics();
        flushMQTTBuffer();
        reconnectAttemptCount = 0;
        strncpy(lastError, "MQTT verbunden", sizeof(lastError));
        logMessage(lastError);
    } else {
        logDebug("MQTT-Verbindung fehlgeschlagen");
        Serial.print("Fehlercode: ");
        Serial.println(mqttClient.connectError());
        mqttConnected = false;
        strncpy(lastError, "MQTT fehlgeschlagen", sizeof(lastError));
        logMessage(lastError);
    }
}

void subscribeMQTTTopics() {
    mqttClient.subscribe(topic_ph);
    mqttClient.subscribe(topic_rtd);
    mqttClient.subscribe(topic_orp);
    mqttClient.subscribe(topic_a0);
    mqttClient.subscribe(topic_a1);
    mqttClient.subscribe(topic_cal);
    logDebug("Abonnierte MQTT-Themen");
}

void loop() {
    static unsigned long lastLoop = 0;
    if (millis() - lastLoop < 10) return;
    lastLoop = millis();

    WDT.refresh();

    // Button-Handling mit angepassten Schwellwerten
    static unsigned long lastButtonCheck = 0;
    if (millis() - lastButtonCheck >= 500) {
        int buttonValue = analogRead(buttonPin);
        Serial.print("Button-Wert: ");
        Serial.println(buttonValue);

        if (buttonValue < 25) {
            logMessage("Button: Right");
            displayMode = DISPLAY_MODE_VERSION; // Modus 4: Version
            logDebug("Modus: Version");
            updateDisplay();
        } else if (buttonValue < 200) {
            logMessage("Button: Up");
            displayMode = DISPLAY_MODE_SENSOR; // Modus 1: Sensorwerte
            logDebug("Modus: Sensorwerte");
            updateDisplay();
        } else if (buttonValue < 400) {
            logMessage("Button: Down");
            displayMode = DISPLAY_MODE_ANALOG; // Modus 2: Analoge Werte
            logDebug("Modus: Analoge Werte");
            updateDisplay();
        } else if (buttonValue < 600) {
            logMessage("Button: Left");
            // Platzhalter für zukünftige Funktionen
        } else if (buttonValue < 900) {
            logMessage("Button: Select");
            displayMode = DISPLAY_MODE_MQTT; // Modus 3: MQTT-Status
            mqttPaused = !mqttPaused; // Toggle MQTT Pause
            logMessage(mqttPaused ? "MQTT pausiert" : "MQTT fortgesetzt");
            logDebug("Modus: MQTT-Status");
            updateDisplay();
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

    while (mqttClient.parseMessage()) {
        String topic = mqttClient.messageTopic();
        String message;
        while (mqttClient.available()) {
            message += (char)mqttClient.read();
        }
        char debugMsg[64];
        snprintf(debugMsg, sizeof(debugMsg), "Empfangene Nachricht auf %s: %s", topic.c_str(), message.c_str());
        logDebug(debugMsg);
        if (topic == topic_cal) {
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
            logDebug("Kalibrierungsbefehl gesendet");
        }
    }

    Seq.run();
    SeqDisplay.run();
    SeqMQTT.run();

    static unsigned long lastStatusUpdate = 0;
    if (millis() - lastStatusUpdate > 30000) {
        char statusMsg[32];
        long rssi = WiFi.RSSI();
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
        char errMsg[32];
        snprintf(errMsg, sizeof(errMsg), "RTD-Fehler, Code: %d", rtd.get_error());
        Serial.println(errMsg);
        logMessage("RTD-Fehler");
        strncpy(lastError, "RTD-Fehler", sizeof(lastError));
        lastReadings.rtdValid = false;
        return;
    }
    float temp = rtd.get_last_received_reading();
    if (temp > -1000.0) {
        ph.send_read_with_temp_comp(temp);
    } else {
        ph.send_read_with_temp_comp(25.0);
        logMessage("RTD ungültig");
        strncpy(lastError, "RTD ungültig", sizeof(lastError));
        lastReadings.rtdValid = false;
    }
    if (ph.get_error() != Ezo_board::SUCCESS) {
        char errMsg[32];
        snprintf(errMsg, sizeof(errMsg), "PH-Fehler, Code: %d", ph.get_error());
        Serial.println(errMsg);
        logMessage("PH-Fehler");
        strncpy(lastError, "PH-Fehler", sizeof(lastError));
        lastReadings.phValid = false;
    }
    orp.send_read_cmd();
    if (orp.get_error() != Ezo_board::SUCCESS) {
        char errMsg[32];
        snprintf(errMsg, sizeof(errMsg), "ORP-Fehler, Code: %d", orp.get_error());
        Serial.println(errMsg);
        logMessage("ORP-Fehler");
        strncpy(lastError, "ORP-Fehler", sizeof(lastError));
        lastReadings.orpValid = false;
    }
}

void step2() {
    ph.receive_read_cmd();
    if (ph.get_error() != Ezo_board::SUCCESS) {
        char errMsg[32];
        snprintf(errMsg, sizeof(errMsg), "PH-Fehler, Code: %d", ph.get_error());
        Serial.println(errMsg);
        logMessage("PH-Fehler");
        strncpy(lastError, "PH-Fehler", sizeof(lastError));
        lastReadings.phValid = false;
    } else {
        lastReadings.ph = ph.get_last_received_reading();
        lastReadings.phValid = (lastReadings.ph >= PH_MIN && lastReadings.ph <= PH_MAX);
        if (!lastReadings.phValid) {
            logMessage("PH ungültig");
            strncpy(lastError, "PH ungültig", sizeof(lastError));
        }
    }
    rtd.receive_read_cmd();
    if (rtd.get_error() != Ezo_board::SUCCESS) {
        char errMsg[32];
        snprintf(errMsg, sizeof(errMsg), "RTD-Fehler, Code: %d", rtd.get_error());
        Serial.println(errMsg);
        logMessage("RTD-Fehler");
        strncpy(lastError, "RTD-Fehler", sizeof(lastError));
        lastReadings.rtdValid = false;
    } else {
        lastReadings.rtd = rtd.get_last_received_reading();
        lastReadings.rtdValid = (lastReadings.rtd >= RTD_MIN && lastReadings.rtd <= RTD_MAX);
        if (!lastReadings.rtdValid) {
            logMessage("RTD ungültig");
            strncpy(lastError, "RTD ungültig", sizeof(lastError));
        }
    }
    orp.receive_read_cmd();
    if (orp.get_error() != Ezo_board::SUCCESS) {
        char errMsg[32];
        snprintf(errMsg, sizeof(errMsg), "ORP-Fehler, Code: %d", orp.get_error());
        Serial.println(errMsg);
        logMessage("ORP-Fehler");
        strncpy(lastError, "ORP-Fehler", sizeof(lastError));
        lastReadings.orpValid = false;
    } else {
        lastReadings.orp = orp.get_last_received_reading();
        lastReadings.orpValid = (lastReadings.orp >= ORP_MIN && lastReadings.orp <= ORP_MAX);
        if (!lastReadings.orpValid) {
            logMessage("ORP ungültig");
            strncpy(lastError, "ORP ungültig", sizeof(lastError));
        }
    }
    lastReadings.timestamp = millis();
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
        snprintf(line1, sizeof(line1), "A0:%d A1:%d", lastReadings.analog0, lastReadings.analog1);
        snprintf(line2, sizeof(line2), "%s", statusMsg);
    } else if (displayMode == DISPLAY_MODE_MQTT) {
        snprintf(line1, sizeof(line1), "MQTT: %s", mqttPaused ? "Pausiert" : "Aktiv");
        snprintf(line2, sizeof(line2), "RSSI:%ld dBm %s", WiFi.RSSI(), statusMsg);
    } else if (displayMode == DISPLAY_MODE_VERSION) {
        snprintf(line1, sizeof(line1), "Sketch: %s", SKETCH_VERSION);
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
            char debugMsg[64];
            snprintf(debugMsg, sizeof(debugMsg), "Veröffentliche auf %s: %s", topic, payload);
            logDebug(debugMsg);
        } else if (!mqttBuffer.isFull()) {
            strncpy(msg.topic, topic, sizeof(msg.topic));
            strncpy(msg.payload, payload, sizeof(msg.payload));
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
        dtostrf(lastReadings.orp, 6, 2, buffer);
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
        strncpy(lastError, "Analog ungültig", sizeof(lastError));
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
    while (!mqttBuffer.isEmpty() && mqttConnected && millis() - lastPublish >= 50) {
        MQTTMessage msg = mqttBuffer.shift();
        mqttClient.beginMessage(msg.topic);
        mqttClient.print(msg.payload);
        mqttClient.endMessage();
        lastPublish = millis();
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
        delay(1000); // Nachricht für 1 Sekunde anzeigen
        updateDisplay(); // Zurück zur aktuellen Anzeige
    }
}

void logDebug(const char* message) {
    if (DEBUG_MODE) {
        Serial.println(message);
        Serial.flush();
    }
}

void idleStep() {}