```cpp
#include "lcd.h"

// D1 Robot LCD Keypad Shield Pinbelegung
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
bool lcdInitialized = false;

void initLCD() {
    lcd.begin(16, 2);
    lcdInitialized = true;
}
```