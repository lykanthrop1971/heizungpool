// lcd.cpp
#include <Arduino.h>
#include <LiquidCrystal.h>
#include "lcd.h"

#ifdef USE_D1_SHIELD
// D1 LCD Keypad Shield pinout (e.g., DFRobot)
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // RS=8, EN=9, D4=4, D5=5, D6=6, D7=7
#else
// Alternative LCD pinout (adjust if needed)
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // Same as D1 for now, adjust for other LCD
#endif
bool lcdInitialized = false;

void initLCD() {
    pinMode(10, OUTPUT);
    digitalWrite(10, HIGH); // Ensure backlight is on
    lcd.begin(16, 2);
    lcdInitialized = true;
    #ifdef USE_D1_SHIELD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("D1 LCD Init OK");
    lcd.setCursor(0, 1);
    lcd.print("Testing...");
    delay(2000); // Show init message for 2 seconds
    #endif
    Serial.println("LCD init complete"); // Debugging
}

void errorLCD(const char* message) {
    if (!lcdInitialized) {
        Serial.println("ErrorLCD: LCD not initialized");
        return;
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Error:");
    lcd.setCursor(0, 1);
    lcd.print(message);
    Serial.print("ErrorLCD: ");
    Serial.println(message); // Debugging
}

void updateLCD(float ph, float orp, float rtd, const char* status) {
    if (!lcdInitialized) {
        Serial.println("updateLCD: LCD not initialized");
        return;
    }
    lcd.clear();
    char line1[17], line2[17];
    snprintf(line1, sizeof(line1), "pH:%.2f T:%.1fC", ph, rtd);
    snprintf(line2, sizeof(line2), "ORP:%.0fmV %s", orp, status);
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
    Serial.println("updateLCD: Display updated"); // Debugging
}

void updateLCD2(int analog0, int analog1, const char* status) {
    if (!lcdInitialized) {
        Serial.println("updateLCD2: LCD not initialized");
        return;
    }
    lcd.clear();
    char line1[17], line2[17];
    snprintf(line1, sizeof(line1), "A0:%d A1:%d", analog0, analog1);
    snprintf(line2, sizeof(line2), "%s", status);
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
    Serial.println("updateLCD2: Display updated"); // Debugging
}