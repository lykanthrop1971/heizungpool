#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_DS248x.h>

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include <vector>

namespace esphome {
namespace ds2482_custom {

class DS2482Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void set_address(uint8_t address) { this->address_ = address; }
  void setup() override;
  void update() override;
  void dump_config() override;

  // Called from Python codegen
  void add_sensor(uint8_t channel, sensor::Sensor *sens);

 protected:
  struct SensorEntry {
    uint8_t channel;
    sensor::Sensor *sensor;
  };

  Adafruit_DS248x ds_;
  uint8_t address_{0x18};
  std::vector<SensorEntry> sensors_;

  bool convert_phase_{true};  // true: start convert, false: read scratchpad
};

}  // namespace ds2482_custom
}  // namespace esphome
