#include "LM393SpeedSensor.h"

// Initialize the static pointer
LM393SpeedSensor *LM393SpeedSensor::instance = nullptr;

LM393SpeedSensor::LM393SpeedSensor(int sensorPin)
{
    this->sensorPin = sensorPin;
    pulseCount = 0;
    prevTime = 0;

    // Set the static pointer to this instance
    instance = this;
}

void LM393SpeedSensor::begin()
{
    pinMode(sensorPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(sensorPin), updatePulseCount, FALLING);
    attachInterrupt(digitalPinToInterrupt(sensorPin), updatePulseCount, RISING);
}

void LM393SpeedSensor::reset()
{
    instance->rps = 0;
}

void LM393SpeedSensor::getRPM()
{
    unsigned long currentTime = millis();
    float delayTime = currentTime - prevTime;

    if (delayTime >= 0)
    {
        // Calculate RPM using the formula: RPM = (60 / T) * 1000
        // T is the time taken for one revolution in milliseconds
        // In this case, T = delayTime
        // The factor of 30 is used to account for the number of teeth on the gear
        instance->rps = (15 / delayTime * 1000);
        prevTime = currentTime;
    }
}

// Define the static member function
void ICACHE_RAM_ATTR LM393SpeedSensor::updatePulseCount()
{
    if (instance)
    {
        instance->getRPM(); // Calculate RPM when the interrupt is triggered
    }
}
