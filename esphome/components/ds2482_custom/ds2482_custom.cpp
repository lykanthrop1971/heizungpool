#include "ds2482_custom.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ds2482_custom {

static const char *TAG = "ds2482";

void DS2482Component::add_sensor(uint8_t channel, sensor::Sensor *sens) {
  SensorEntry e{};
  e.channel = channel;
  e.sensor = sens;
  sensors_.push_back(e);
}

void DS2482Component::setup() {
  ESP_LOGI(TAG, "Initializing DS248x at 0x%02X…", this->address_);
  if (!this->ds_.begin(&Wire, this->address_)) {
    ESP_LOGE(TAG, "DS248x not found at 0x%02X", this->address_);
    mark_failed();
    return;
  }
  // reasonable defaults
  this->ds_.activePullup(true);
  this->ds_.strongPullup(false);   // enable later if you use parasite power
  this->ds_.overdriveSpeed(false);

  ESP_LOGI(TAG, "Configured channel sensors: %u", (unsigned)sensors_.size());
  for (auto &e : sensors_) {
    ESP_LOGI(TAG, "  CH%u -> %s", e.channel, e.sensor->get_name().c_str());
  }
}

void DS2482Component::update() {
  if (convert_phase_) {
    // Phase 1: start conversion on each channel
    for (auto &entry : sensors_) {
      if (!ds_.selectChannel(entry.channel)) continue;
      if (!ds_.OneWireReset()) continue;
      ds_.OneWireWriteByte(0xCC); // SKIP ROM => first device on bus
      ds_.OneWireWriteByte(0x44); // CONVERT T
    }
    // no blocking wait here; we'll read next cycle
    convert_phase_ = false;
  } else {
    // Phase 2: read scratchpad and publish
    for (auto &entry : sensors_) {
      if (!ds_.selectChannel(entry.channel)) continue;
      if (!ds_.OneWireReset()) continue;
      ds_.OneWireWriteByte(0xCC); // SKIP ROM
      ds_.OneWireWriteByte(0xBE); // READ SCRATCHPAD

      uint8_t data[9] = {0};
      for (int i = 0; i < 9; i++) {
        uint8_t b = 0;
        if (!ds_.OneWireReadByte(&b)) { b = 0; }
        data[i] = b;
      }

      // (Optional) TODO: verify CRC in data[8]
      int16_t raw = (static_cast<int16_t>(data[1]) << 8) | data[0];
      float temp_c = raw / 16.0f;
      entry.sensor->publish_state(temp_c);
    }
    convert_phase_ = true;
  }
}

void DS2482Component::dump_config() {
  ESP_LOGCONFIG(TAG, "DS248x at 0x%02X", this->address_);
  LOG_I2C_DEVICE(this);
}

}  // namespace ds2482_custom
}  // namespace esphome
