#include "LM393SpeedSensor.h"

// Initialize the static pointer
LM393SpeedSensor* LM393SpeedSensor::instance = nullptr;

LM393SpeedSensor::LM393SpeedSensor(int sensorPin) {
    this->sensorPin = sensorPin;
    pulseCount = 0;
    prevTime = 0;

    // Set the static pointer to this instance
    instance = this;
}

void LM393SpeedSensor::begin() {
    pinMode(sensorPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(sensorPin), updatePulseCount, FALLING);
}

void LM393SpeedSensor::reset() {
    instance->rps = 0;
}

void LM393SpeedSensor::getRPM() {
    unsigned long currentTime = millis();
    float delayTime = currentTime - prevTime;

    if (delayTime >= 0) {
        instance->rps = (14 / delayTime * 1000);
        prevTime = currentTime;
    }
}

// Define the static member function
void ICACHE_RAM_ATTR LM393SpeedSensor::updatePulseCount() {
    if (instance) {
        instance->getRPM(); // Calculate RPM when the interrupt is triggered
    }
}
