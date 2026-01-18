#ifndef MovingRuntime_h
#define MovingRuntime_h

#include "Arduino.h"
#include <SD.h>  // Bibliothek für SD-Karten-Unterstützung

#define MAX_EVENTS 100  // Maximale Anzahl von gespeicherten Pumpenereignissen
#define INTERVALL_1M (60UL )  // 1 Minute in Millisekunden
#define INTERVALL_24H (24UL * 60UL * 60UL )  // 24 Stunden in Millisekunden
#define INTERVALL_7D (7UL * 24UL * 60UL * 60UL )  // 7 Tage in Millisekunden
#define INTERVALL_6H (7UL * 6UL * 60UL * 60UL )  // 7 Tage in Millisekunden

class MovingRuntime {
  public:
    MovingRuntime(bool &statusPump, int sdChipSelect, const char* logFile);
    void begin();  // Setup-Routine für die Bibliothek
    void update(); // Loop-Routine zur Überwachung und Berechnung
    unsigned long calculateMovingRuntime(unsigned long currentTime, unsigned long timeWindow);

  private:
    bool &_statusPump;  // Referenz auf die Pumpenstatus-Variable
    bool _lastPumpeState;
    int _sdChipSelect;
    const char* _logFile;
    int eventIndex;  // Aktueller Index im Puffer
    struct PumpEvent {
      unsigned long timestamp;
      bool isOn;
    };
    PumpEvent events[MAX_EVENTS];  // Ringpuffer für Pumpenereignisse

    void addEvent(bool isOn);
    void saveEventsToFile();
    void loadEventsFromFile();
};

#endif
