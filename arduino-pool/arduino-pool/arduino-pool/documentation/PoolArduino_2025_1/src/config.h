#ifndef CONFIG_H
#define CONFIG_H

// WiFi credentials
const char* ssid = "Dein_WiFi_SSID";
const char* pass = "Dein_WiFi_Passwort";

// MQTT configuration
const char* broker = "Dein_MQTT_Broker_IP";
const int port = 1883;
const char* mqtt_client_id = "PoolArduino3";
const char* topic_ph = "sensor2/ph";
const char* topic_rtd = "sensor2/rtd";
const char* topic_orp = "sensor2/orp";
const char* topic_a0 = "sensor2/a0";
const char* topic_a1 = "sensor2/a1";
const char* topic_cal = "sensor2/cal";

#endif