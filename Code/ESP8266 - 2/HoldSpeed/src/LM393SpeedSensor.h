#ifndef LM393SPEEDSENSOR_H
#define LM393SPEEDSENSOR_H

#include <Arduino.h>

class LM393SpeedSensor {
public:
    LM393SpeedSensor(int sensorPin);

    void begin();
    void reset();
    unsigned long rps;

private:
    int sensorPin;
    volatile unsigned long pulseCount;
    unsigned long prevTime;

    // Declare the static member function
    static void ICACHE_RAM_ATTR updatePulseCount();

    void getRPM();
    static void updateRPM();

    // Static pointer to an instance of the class
    static LM393SpeedSensor* instance;
};

#endif
