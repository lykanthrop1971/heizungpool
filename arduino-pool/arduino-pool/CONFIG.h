#ifndef CONFIG_H
#define CONFIG_H

// WiFi credentials
const char ssid[] = "FRITZ!Box 7590 KS";
const char pass[] = "70754088624202801803";

// MQTT configuration
//const char* broker = "192.168.1.56";
const char* broker = "192.168.1.128";
// MQTT authentication
const char* mqtt_user = "mqttbroker";
const char* mqtt_password = "mqttbroker";

const int port = 1883;
const char* mqtt_client_id = "PoolArduino"; // MQTT Client-ID
const char* topic_ph = "sensor/ph";
const char* topic_rtd = "sensor/rtd";
const char* topic_orp = "sensor/orp";
const char* topic_a0 = "sensor/a0";
const char* topic_a1 = "sensor/a1";
const char* topic_cal = "sensor/cal";
// DAC control (0-10V) via MQTT
static const char* topic_dac_set   = "pool/dac/set";    // payload: "7.50" or "7,50" or "V=7.5"
static const char* topic_dac_state = "pool/dac/state";  // publishes applied voltage, e.g. "7.50"
#endif