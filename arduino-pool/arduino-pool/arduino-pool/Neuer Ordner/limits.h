#ifndef LIMITS_H
#define LIMITS_H

const float limit_ph_min_ctrl = 7.0;
const float limit_ph_max_ctrl = 7.4;

const float limit_ph_min_error = 5;
const float limit_ph_max_error = 8;

const float allowed_ph_6h = 600;
const float allowed_ph_24h = 1800;

const float limit_ph_pump_24h_max = 3600000; // Max. 1h = 3600 * 1000 ms
const float limit_ph_pump_7d_max = 3* limit_ph_pump_24h_max;

const unsigned int Arduino_Alive_Sec_before_ph = 60; // Zeit ArduinoAlive in Sek. bevor pH-Dosierung erlaubt 


int checkLimits();
#endif