#ifndef RTPUMPS_H
#define RTPUMPS_H

extern int dosierPumpe1State;
extern int dosierPumpe2State;
extern unsigned long laufzeitPumpe1_24h;
extern unsigned long laufzeitPumpe2_24h;
extern unsigned long laufzeitPumpe1_7d;  
extern unsigned long laufzeitPumpe2_7d;

void initRTpumps();
void updateRTpumps();
void speichernLaufzeiten();

#endif