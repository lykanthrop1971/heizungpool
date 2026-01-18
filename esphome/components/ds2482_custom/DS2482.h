#pragma once
#include <Arduino.h>
#include <Wire.h>

// Minimal DS2482-100/800 I2C to 1-Wire bridge driver
// Based on public-domain code by various authors, trimmed for ESPHome use

#define DS2482_CMD_RESET                0xF0
#define DS2482_CMD_SET_READ_PTR         0xE1
#define DS2482_CMD_WRITE_CONFIG         0xD2
#define DS2482_CMD_1WIRE_RESET          0xB4
#define DS2482_CMD_1WIRE_SINGLE_BIT     0x87
#define DS2482_CMD_1WIRE_WRITE_BYTE     0xA5
#define DS2482_CMD_1WIRE_READ_BYTE      0x96
#define DS2482_CMD_1WIRE_TRIPLET        0x78
#define DS2482_CMD_SELECT_CHANNEL       0xC3

#define DS2482_PTR_STATUS               0xF0
#define DS2482_PTR_DATA                 0xE1
#define DS2482_PTR_CONFIG               0xC3

class DS2482 {
public:
  DS2482(uint8_t addr = 0x18);
  bool begin();
  bool reset();
  bool selectChannel(uint8_t ch);

  bool wireReset();
  void wireWriteByte(uint8_t b);
  uint8_t wireReadByte();
  bool wireWriteBit(uint8_t bit);
  uint8_t wireReadBit();

private:
  uint8_t _addr;
  bool writeByte(uint8_t b);
  bool writeByte(uint8_t b1, uint8_t b2);
  uint8_t readByte();
};
