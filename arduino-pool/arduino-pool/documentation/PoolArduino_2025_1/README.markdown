```markdown
# PoolArduino_2025_1

Ein Arduino-Projekt für die Überwachung von Pool-Wasserparametern (pH, ORP, Temperatur) und analogen Sensoren mit einem Arduino UNO R4 WiFi, einem D1 Robot LCD Keypad Shield (16x2 LCD) und MQTT-Kommunikation. Der Sketch unterstützt einen optimierten I2C-Scan, tastengesteuerte Statusanzeige (vier Modi) und Kalibrierung über MQTT.

- **Version**: v1.2
- **Autor**: Martin
- **Datum**: Juni 2025

## Features
- **Sensoren**:
  - EZO pH (I2C-Adresse: `0x63`), ORP (`0x62`), RTD/Temperatur (`0x66`).
  - Analoge Eingänge: A2 (Sensor 0), A1 (Sensor 1).
- **Display**: 16x2 LCD (D1 Robot LCD Keypad Shield, parallel) mit vier Anzeigemodi:
  - Modus 1: pH, ORP, Temperatur.
  - Modus 2: Analoge Werte (A0, A1).
  - Modus 3: MQTT-Status (Aktiv/Pausiert, RSSI).
  - Modus 4: Sketch-Version (`v1.2`).
- **Tastensteuerung** (D1 Shield):
  - Right (~0): Modus 4 (Version).
  - Up (~132): Modus 1 (Sensorwerte).
  - Down (~307): Modus 2 (Analoge Werte).
  - Select (~717): Modus 3 (MQTT-Status, toggelt Pause).
  - Left (~479): Platzhalter.
- **MQTT**:
  - Veröffentlicht Sensorwerte auf `sensor2/ph`, `sensor2/rtd`, `sensor2/orp`, `sensor2/a0`, `sensor2/a1`.
  - Unterstützt Kalibrierung über `sensor2/cal` (z. B. `cal,mid,7`).
  - Client-ID konfigurierbar in `config.h` (Standard: `PoolArduino3`).
- **I2C-Scan**: Optimiert für bekannte Adressen (`0x63`, `0x66`, `0x62`) mit 50 ms Timeout.
- **Watchdog**: 8 Sekunden Timeout zur Vermeidung von Hängenbleiben.
- **Fehlerbehandlung**: Logging auf Serial Monitor und LCD, Pufferung von MQTT-Nachrichten bei Verbindungsverlust.

## Hardware
- **Mikrocontroller**: Arduino UNO R4 WiFi.
- **Display**: D1 Robot LCD Keypad Shield (parallel, 16x2, Backlight auf D10).
- **Sensoren**:
  - EZO pH Sensor (I2C, Adresse: `0x63`).
  - EZO ORP Sensor (I2C, Adresse: `0x62`).
  - EZO RTD Sensor (I2C, Adresse: `0x66`).
  - Analoge Sensoren an A2 und A1.
- **Verkabelung**:
  - I2C: A4 (SDA), A5 (SCL) mit 4.7 kΩ Pullup-Widerständen zu VCC.
  - D1 Shield: D4, D5, D6, D7, D8, D9 (LCD), A0 (Tasten).
  - Backlight: D10 (Jumper geschlossen).
- **Stromversorgung**: USB oder extern (7-12V, empfohlen 9V).

## Software
- **Arduino IDE**: Version 2.3.2 oder höher.
- **Bibliotheken**:
  - `Wire` (vorinstalliert).
  - `Ezo_i2c` (Atlas Scientific EZO I2C Library).
  - `sequencer2` (für zeitgesteuerte Abläufe).
  - `WiFiS3` (Version 1.4.1, für UNO R4 WiFi).
  - `ArduinoMqttClient` (Version 0.1.8).
  - `CircularBuffer` (für MQTT-Puffer).
  - `WDT` (für Watchdog).
  - `LiquidCrystal` (vorinstalliert, für D1 Shield LCD).
- **Hinweis**: Entferne die `hd44780`-Bibliothek, falls vorhanden, da sie mit `LiquidCrystal` kollidiert.

## Installation
1. **Repository klonen**:
   ```bash
   git clone https://github.com/DeinBenutzername/PoolArduino_2025_1.git
   cd PoolArduino_2025_1
   ```
2. **Bibliotheken installieren**:
   - Öffne die Arduino IDE.
   - Gehe zu `Sketch > Include Library > Manage Libraries`.
   - Suche und installiere: `WiFiS3` (1.4.1), `ArduinoMqttClient` (0.1.8), `CircularBuffer`.
   - Lade `Ezo_i2c` und `sequencer2` von den offiziellen Quellen (z. B. GitHub von Atlas Scientific bzw. Arduino-Community) und füge sie manuell hinzu (`Sketch > Include Library > Add .ZIP Library`).
3. **Projekt öffnen**:
   - Öffne `src/PoolArduino_2025_1.ino` in der Arduino IDE.
4. **Konfiguration anpassen**:
   - Bearbeite `src/config.h`:
     ```cpp
     const char* ssid = "Dein_WiFi_SSID";
     const char* pass = "Dein_WiFi_Passwort";
     const char* broker = "192.168.1.100";
     const int port = 1883;
     const char* mqtt_client_id = "PoolArduino3";
     ```
   - Stelle sicher, dass dein MQTT-Broker (z. B. Mosquitto) läuft und `allow_anonymous true` in `mosquitto.conf` gesetzt ist.
5. **Kompilieren und Hochladen**:
   - Wähle `Arduino UNO R4 WiFi` als Board.
   - Wähle den korrekten Port (z. B. `/dev/ttyACM0`).
   - Klicke auf `Upload`.
   - Öffne den Serial Monitor (115200 Baud) zur Überwachung.

## Ordnerstruktur
```
PoolArduino_2025_1/
├── src/
│   ├── PoolArduino_2025_1.ino  # Hauptsketch
│   ├── config.h               # WiFi- und MQTT-Konfiguration
│   ├── lcd.h                  # LCD-Header
│   ├── lcd.cpp                # LCD-Implementierung
│   ├── limits.h               # Sensor-Grenzwerte
├── README.md                  # Diese Dokumentation
```

## Bedienung
1. **Start**:
   - Nach dem Hochladen zeigt das LCD:
     - `PoolArduino v1.2` (1 Sekunde).
     - `Status: Starte...` (1 Sekunde).
     - Dann Modus 1: `pH:7.00 T:25.0C ORP:200mV W:OK M:OK`.
   - Serial Monitor (115200 Baud) zeigt:
     ```
     Starte...
     Watchdog initialisiert
     D1 LCD initialisiert
     Scanne I2C-Bus (bekannte Adressen)...
     I2C-Gerät pH bei 0x63
     I2C-Gerät RTD bei 0x66
     I2C-Gerät ORP bei 0x62
     I2C-Scan: 3 Geräte gefunden
     WiFi verbunden
     IP: 192.168.1.180
     MQTT verbunden
     ```
2. **Tastensteuerung**:
   - **Right** (~0): Modus 4 (`Sketch: v1.2 W:OK M:OK`).
   - **Up** (~132): Modus 1 (Sensorwerte: `pH:7.00 T:25.0C ORP:200mV W:OK M:OK`).
   - **Down** (~307): Modus 2 (Analoge Werte: `A0:190 A1:512 W:OK M:OK`).
   - **Select** (~717): Modus 3 (`MQTT: Aktiv RSSI:-65 dBm W:OK M:OK`), toggelt MQTT-Pause.
   - **Left** (~479): Keine Funktion (Platzhalter).
3. **MQTT**:
   - Veröffentlicht alle 30 Sekunden (wenn nicht pausiert):
     - `sensor2/ph`: pH-Wert (z. B. `7.00`).
     - `sensor2/rtd`: Temperatur (°C, z. B. `25.0`).
     - `sensor2/orp`: ORP (mV, z. B. `200.0`).
     - `sensor2/a0`, `sensor2/a1`: Analoge Werte (0-1023).
   - Kalibrierung über `sensor2/cal`:
     - `cal,mid,7`: pH-Mittelkalibrierung (6.86).
     - `cal,low,4`: pH-Tiefkalibrierung (4.01).
     - `cal,high,10`: pH-Hochkalibrierung (9.18).
     - `cal,orp,256`: ORP-Kalibrierung (256 mV).
     - `cal,clear`: Löscht Kalibrierung (pH und ORP).
   - Verwende MQTT Explorer oder ähnliche Tools, um Daten zu überwachen.

## Fehlerbehebung
- **LCD zeigt nichts**:
  - Prüfe D10-Jumper (Backlight) und V0-Potentiometer (Kontrast).
  - Serial Monitor: Suche nach `D1 LCD-Init fehlgeschlagen`.
  - Stelle sicher, dass `LiquidCrystal` verwendet wird, nicht `hd44780`.
- **Tasten funktionieren nicht**:
  - Prüfe `buttonValue` im Serial Monitor:
    ```
    Button-Wert: 0 (Right), 132 (Up), 307 (Down), 479 (Left), 717 (Select)
    ```
  - Passe Schwellwerte in `loop()` an, falls nötig:
    ```cpp
    if (buttonValue < 25) { // Right
    } else if (buttonValue < 200) { // Up
    } else if (buttonValue < 400) { // Down
    } else if (buttonValue < 600) { // Left
    } else if (buttonValue < 900) { // Select
    }
    ```
- **I2C-Fehler**:
  - Serial Monitor: Suche nach `I2C-Timeout: Prüfe Pullups` oder `Keine I2C-Geräte gefunden`.
  - Prüfe 4.7 kΩ Pullup-Widerstände an A4 (SDA) und A5 (SCL).
  - Stelle sicher, dass Sensoren korrekt angeschlossen sind (`0x63`, `0x66`, `0x62`).
- **WiFi/MQTT-Verbindung fehlschlägt**:
  - Prüfe `config.h` (`ssid`, `pass`, `broker`, `port`, `mqtt_client_id`).
  - Stelle sicher, dass `mosquitto.conf` `allow_anonymous true` enthält.
  - Serial Monitor: Suche nach `WiFi fehlgeschlagen` oder `MQTT-Verbindung fehlgeschlagen` und Fehlercode.
  - Prüfe, ob `mqtt_client_id` eindeutig ist.
- **Sensorwerte ungültig**:
  - Serial Monitor: Suche nach `PH ungültig`, `RTD ungültig`, `ORP ungültig`, `Analog ungültig`.
  - Prüfe `limits.h` (`PH_MIN`, `PH_MAX`, etc.).
  - Kalibriere Sensoren über MQTT (`sensor2/cal`).

## Erweiterungsmöglichkeiten
- **Neue Tastenfunktion**: Implementiere eine Funktion für die Left-Taste (~479), z. B. für manuelle Kalibrierung oder Reset.
- **Zusätzliche Sensoren**: Füge weitere I2C- oder analoge Sensoren hinzu, aktualisiere `limits.h` und MQTT-Themen.
- **Webinterface**: Integriere eine Webserver-Funktion (z. B. mit `WiFiS3`), um Sensorwerte über HTTP anzuzeigen.
- **OTA-Updates**: Implementiere Over-the-Air-Updates für den Arduino UNO R4 WiFi.
- **Datenlogging**: Speichere Sensorwerte auf einer SD-Karte (benötigt zusätzliches Modul).
- **Versionsverwaltung**: Definiere `SKETCH_VERSION` in `config.h` für zentralisierte Konfiguration.

## Lizenz
MIT License

Copyright (c) 2025 Martin

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
```