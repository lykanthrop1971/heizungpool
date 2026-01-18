#include "MovingRuntime.h"
#include <SD.h>
#include <SPI.h>
#include "RTC.h"

// Konstruktor: Initialisiert die SD-Karte und die Referenz auf den Pumpenstatus
MovingRuntime::MovingRuntime(bool& statusPump, int sdChipSelect, const char* logFile)
  : _statusPump(statusPump), _lastPumpeState(statusPump), _sdChipSelect(sdChipSelect), _logFile(logFile) {
  eventIndex = 0;
}

// Beginne die Initialisierung: SD-Karte und Pumpeneinstellungen
void MovingRuntime::begin() {
  // SD-Karte initialisieren
  if (!SD.begin(_sdChipSelect)) {
    Serial.println("SD-Karten Initialisierung fehlgeschlagen!");
    return;
  }
  // Lade Pumpenereignisse von der SD-Karte
  loadEventsFromFile();
}

// Die `update()`-Methode überprüft, ob der Pumpenstatus sich geändert hat und berechnet die Laufzeiten
void MovingRuntime::update() {
  //unsigned long currentMillis = millis();
  RTCTime currentTime_RT;
  RTC.getTime(currentTime_RT);
  events[eventIndex].timestamp = currentTime_RT.getUnixTime();
  // Prüfe, ob sich der Zustand der Pumpe geändert hat
  if (_statusPump != _lastPumpeState) {
    addEvent(_statusPump);
    _lastPumpeState = _statusPump;
  }
}

// Füge ein neues Pumpenereignis hinzu
void MovingRuntime::addEvent(bool isOn) {
  //unsigned long currentTime = millis();
  RTCTime currentTime_RT;
  RTC.getTime(currentTime_RT);
  events[eventIndex].timestamp = currentTime_RT.getUnixTime();
  events[eventIndex].isOn = isOn;
  eventIndex = (eventIndex + 1) % MAX_EVENTS;  // Zyklischer Puffer
  saveEventsToFile();                          // Speichere nach jedem Event
}

// Berechne die gleitende Laufzeit innerhalb des Zeitfensters (Overflow-sicher)
unsigned long MovingRuntime::calculateMovingRuntime(unsigned long currentTime, unsigned long timeWindow) {
  unsigned long runtime = 0;
  bool isPumpeOn = false;
  unsigned long lastOnTime = 0;

  for (int i = 0; i < MAX_EVENTS; i++) {
    unsigned long eventTime = events[i].timestamp;

    // Verwende eine Overflow-sichere Berechnung der Zeitdifferenz
    if ((currentTime - eventTime) <= timeWindow) {
      if (events[i].isOn) {
        isPumpeOn = true;
        lastOnTime = eventTime;
      } else if (isPumpeOn) {
        runtime += (eventTime - lastOnTime);
        isPumpeOn = false;
      }
    }
  }

  if (isPumpeOn) {
    runtime += (currentTime - lastOnTime);
  }
  return runtime;
}


// Speichert die Ereignisse auf der SD-Karte
void MovingRuntime::saveEventsToFile() {
  // File file = SD.open(_logFile, FILE_WRITE);
  SD.remove(_logFile);
  File file = SD.open(_logFile, FILE_WRITE);
  if (file) {
    file.println(eventIndex);
    for (int i = 0; i < MAX_EVENTS; i++) {
      file.println(events[i].timestamp);
      file.println(events[i].isOn);
      Serial.print(i);
      Serial.print(" ");
      Serial.print(events[i].timestamp);
      Serial.print("   ");
      Serial.println(events[i].isOn);
      RTCTime currentTime_RT;
      RTC.getTime(currentTime_RT);
      Serial.println(currentTime_RT.getUnixTime());
    }
    file.close();
  }
}

// Lädt die Ereignisse von der SD-Karte
void MovingRuntime::loadEventsFromFile() {
  if (SD.exists(_logFile)) {
    File file = SD.open(_logFile, FILE_READ);
    if (file) {
      eventIndex = file.parseInt();
      for (int i = 0; i < MAX_EVENTS; i++) {
        events[i].timestamp = file.parseInt();
        events[i].isOn = file.parseInt();
      }
      file.close();
    }
  }
}
