#include <Wire.h>
#include "RTpumps.h"
#include <SD.h>

// Variable für die Dosierpumpen-Status
int dosierPumpe1State = 0;      // Dosierpumpe 1
int dosierPumpe2State = 0;      // Dosierpumpe 2

const int chipSelect = 10;  // Pin für SD-Karte
File laufzeitFile;

const char* LZ_filename = "LZ_P1.TXT";

// Variablen für Laufzeiten
unsigned long laufzeitPumpe1_24h = 0;  // Laufzeit in den letzten 24h (Millisekunden)
unsigned long laufzeitPumpe2_24h = 0;
unsigned long laufzeitPumpe1_7d = 0;  // Laufzeit in den letzten 7 Tagen (Millisekunden)
unsigned long laufzeitPumpe2_7d = 0;

unsigned long letzteMessung = 0;  // Letzte Zeit der Messung

// Zeitpunkte für die letzte Messung (Millisekunden)
unsigned long letzte24hAktualisierung = 0;
unsigned long letzte7dAktualisierung = 0;

// Intervalle (in Millisekunden)
const unsigned long INTERVALL_24H = 24UL * 60UL * 60UL * 1000UL;       // 24 Stunden in Millisekunden
const unsigned long INTERVALL_7D = 7UL * 24UL * 60UL * 60UL * 1000UL;  // 7 Tage in Millisekunden

// Zeitstempel für Laufzeiten speichern
struct Laufzeit {
  unsigned long startZeit;
  unsigned long dauer;
};

// Maximal 100 Einträge speichern (anpassbar je nach Bedarf)
const int maxLaufzeitEintraege = 100;
Laufzeit pumpe1Laufzeiten[maxLaufzeitEintraege];
Laufzeit pumpe2Laufzeiten[maxLaufzeitEintraege];
int pumpe1Index = 0;
int pumpe2Index = 0;

void initRTpumps() {
  // Laufzeiten aus der SD-Karte laden
  // SD-Karte initialisieren
  if (!SD.begin(chipSelect)) {
    Serial.println("SD-Karten Initialisierung fehlgeschlagen!");
    return;
  }

  if (SD.exists(LZ_filename)) {
    laufzeitFile = SD.open(LZ_filename, FILE_READ);
    if (laufzeitFile) {
      laufzeitPumpe1_24h = laufzeitFile.parseInt();
      laufzeitPumpe2_24h = laufzeitFile.parseInt();
      laufzeitPumpe1_7d = laufzeitFile.parseInt();
      laufzeitPumpe2_7d = laufzeitFile.parseInt();
      laufzeitFile.close();
    }
  } 
  letzteMessung = millis();
}

void initRTpumps_() {
  if (!SD.begin(chipSelect)) {
    Serial.println("SD-Karten Initialisierung fehlgeschlagen!");
    return;
  }

  // Puffer aus der SD-Karte laden
  if (SD.exists(LZ_filename)) {
    laufzeitFile = SD.open(LZ_filename, FILE_READ);
    if (laufzeitFile) {
      pumpe1Index = laufzeitFile.parseInt();  // Lade den aktuellen Index
      pumpe2Index = laufzeitFile.parseInt();

      // Lade den Puffer für Pumpe 1
      for (int i = 0; i < pumpe1Index; i++) {
        pumpe1Laufzeiten[i].startZeit = laufzeitFile.parseInt();
        pumpe1Laufzeiten[i].dauer = laufzeitFile.parseInt();
      }

      // Lade den Puffer für Pumpe 2
      for (int i = 0; i < pumpe2Index; i++) {
        pumpe2Laufzeiten[i].startZeit = laufzeitFile.parseInt();
        pumpe2Laufzeiten[i].dauer = laufzeitFile.parseInt();
      }

      laufzeitFile.close();
    }
  }
  letzteMessung = millis();
}

void speichernLaufzeiten() {
  SD.remove(LZ_filename);
  File laufzeitFile = SD.open(LZ_filename, FILE_WRITE);
  if (laufzeitFile) {
    laufzeitFile.println(laufzeitPumpe1_24h);
    laufzeitFile.println(laufzeitPumpe2_24h);
    laufzeitFile.println(laufzeitPumpe1_7d);
    laufzeitFile.println(laufzeitPumpe2_7d);
    laufzeitFile.close();
    Serial.println("Laufzeiten gespeichert.");
  } else {
    Serial.println("Fehler beim Schreiben der Datei.");
  }
}

void speichernLaufzeiten_() {
  SD.remove(LZ_filename);
  File laufzeitFile = SD.open(LZ_filename, FILE_WRITE);
  if (laufzeitFile) {
    // Speichere den aktuellen Index
    laufzeitFile.println(pumpe1Index);
    laufzeitFile.println(pumpe2Index);

    // Speichere den Puffer für Pumpe 1
    for (int i = 0; i < pumpe1Index; i++) {
      laufzeitFile.println(pumpe1Laufzeiten[i].startZeit);
      laufzeitFile.println(pumpe1Laufzeiten[i].dauer);
    }

    // Speichere den Puffer für Pumpe 2
    for (int i = 0; i < pumpe2Index; i++) {
      laufzeitFile.println(pumpe2Laufzeiten[i].startZeit);
      laufzeitFile.println(pumpe2Laufzeiten[i].dauer);
    }

    laufzeitFile.close();
    Serial.println("Laufzeiten gespeichert.");
  } else {
    Serial.println("Fehler beim Schreiben der Datei.");
  }
}


void updateRTpumps() {
  unsigned long aktuelleMillis = millis();

  // Laufzeit für Pumpe 1 und Pumpe 2 in der aktuellen Periode berechnen
  if (dosierPumpe1State == HIGH) {
    laufzeitPumpe1_24h += (aktuelleMillis - letzteMessung);
    laufzeitPumpe1_7d += (aktuelleMillis - letzteMessung);
  }

  if (dosierPumpe2State == HIGH) {
    laufzeitPumpe2_24h += (aktuelleMillis - letzteMessung);
    laufzeitPumpe2_7d += (aktuelleMillis - letzteMessung);
  }

  // Aktualisiere die letzte Messzeit
  letzteMessung = aktuelleMillis;

  // Laufzeitdaten nach 24h zurücksetzen
  if (aktuelleMillis - letzte24hAktualisierung >= INTERVALL_24H) {
    laufzeitPumpe1_24h = 0;
    laufzeitPumpe2_24h = 0;
    letzte24hAktualisierung = aktuelleMillis;
  }

  // Laufzeitdaten nach 7 Tagen zurücksetzen
  if (aktuelleMillis - letzte7dAktualisierung >= INTERVALL_7D) {
    laufzeitPumpe1_7d = 0;
    laufzeitPumpe2_7d = 0;
    letzte7dAktualisierung = aktuelleMillis;
  }

  // Daten ausgeben (zum Testen)
  Serial.print("Laufzeit Pumpe 1 (24h): ");
  Serial.print(laufzeitPumpe1_24h / 1000);  // Ausgabe in Sekunden
  Serial.println(" Sekunden");

  Serial.print("Laufzeit Pumpe 2 (24h): ");
  Serial.print(laufzeitPumpe2_24h / 1000);  // Ausgabe in Sekunden
  Serial.println(" Sekunden");

  Serial.print("Laufzeit Pumpe 1 (7 Tage): ");
  Serial.print(laufzeitPumpe1_7d / 1000);  // Ausgabe in Sekunden
  Serial.println(" Sekunden");

  Serial.print("Laufzeit Pumpe 2 (7 Tage): ");
  Serial.print(laufzeitPumpe2_7d / 1000);  // Ausgabe in Sekunden
  Serial.println(" Sekunden");
  speichernLaufzeiten();

  //delay(1000);  // 1 Sekunde warten
}

void aktualisiereLaufzeit(unsigned long aktuelleMillis, Laufzeit puffer[], int &index, unsigned long &laufzeit24h, unsigned long &laufzeit7d, int pumpenStatus) {
  // Füge neue Laufzeit hinzu, falls Pumpe aktiv ist
  if (pumpenStatus == HIGH) {
    unsigned long dauer = aktuelleMillis - letzteMessung;
    puffer[index] = {aktuelleMillis, dauer};
    index = (index + 1) % maxLaufzeitEintraege;  // zyklischer Puffer
  }

  // Berechne Laufzeiten innerhalb der letzten 24 Stunden und 7 Tage
  laufzeit24h = 0;
  laufzeit7d = 0;

  for (int i = 0; i < maxLaufzeitEintraege; i++) {
    if (puffer[i].startZeit == 0) continue;  // Leere Einträge überspringen

    if (aktuelleMillis - puffer[i].startZeit <= INTERVALL_24H) {
      laufzeit24h += puffer[i].dauer;
    }
    if (aktuelleMillis - puffer[i].startZeit <= INTERVALL_7D) {
      laufzeit7d += puffer[i].dauer;
    }
  }
}

void updateRTpumps_() {
  unsigned long aktuelleMillis = millis();

  // Aktualisiere Pumpenlaufzeiten
  aktualisiereLaufzeit(aktuelleMillis, pumpe1Laufzeiten, pumpe1Index, laufzeitPumpe1_24h, laufzeitPumpe1_7d, dosierPumpe1State);
  aktualisiereLaufzeit(aktuelleMillis, pumpe2Laufzeiten, pumpe2Index, laufzeitPumpe2_24h, laufzeitPumpe2_7d, dosierPumpe2State);

  letzteMessung = aktuelleMillis;
  speichernLaufzeiten();

  // Testausgabe
  Serial.print("Laufzeit Pumpe 1 (24h): ");
  Serial.print(laufzeitPumpe1_24h / 1000);  
  Serial.println(" Sekunden");

  Serial.print("Laufzeit Pumpe 2 (24h): ");
  Serial.print(laufzeitPumpe2_24h / 1000);  
  Serial.println(" Sekunden");

  Serial.print("Laufzeit Pumpe 1 (7 Tage): ");
  Serial.print(laufzeitPumpe1_7d / 1000);  
  Serial.println(" Sekunden");

  Serial.print("Laufzeit Pumpe 2 (7 Tage): ");
  Serial.print(laufzeitPumpe2_7d / 1000);  
  Serial.println(" Sekunden");
}
