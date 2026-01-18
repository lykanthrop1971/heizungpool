// lcd.h
#ifndef LCD_H
#define LCD_H

#include <LiquidCrystal.h>

extern LiquidCrystal lcd;
extern bool lcdInitialized;

void initLCD();
void errorLCD(const char* message);
void updateLCD(float ph, float orp, float rtd, const char* status);
void updateLCD2(int analog0, int analog1, const char* status);

#endif