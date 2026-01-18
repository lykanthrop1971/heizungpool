#ifndef LCD_H
#define LCD_H

#include <LiquidCrystal.h>

extern LiquidCrystal lcd;
extern bool lcdInitialized;

void initLCD();

#endif