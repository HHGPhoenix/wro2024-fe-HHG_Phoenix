#include <pigpio.h>
#include <unistd.h>
#include <iostream>
#include <chrono>

#define TRIGGER_PIN 17
#define ECHO_PIN 4
#define SERVO_PIN 27
#define SPEED_OF_SOUND 34300 // in cm/s

void trigger_sonar()
{
    gpioWrite(TRIGGER_PIN, PI_ON);
    usleep(10);
    gpioWrite(TRIGGER_PIN, PI_OFF);
}

double read_distance()
{
    auto start = std::chrono::high_resolution_clock::now();
    while (gpioRead(ECHO_PIN) == 0)
    {
        start = std::chrono::high_resolution_clock::now();
    }

    auto stop = std::chrono::high_resolution_clock::now();
    while (gpioRead(ECHO_PIN) == 1)
    {
        stop = std::chrono::high_resolution_clock::now();
    }

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();
    double distance = (duration / 2.0 / 10000.0) * SPEED_OF_SOUND;

    return distance / 100;
}

void control_servo(double distance)
{
    int pulse_width;

    if (distance < 10)
    {
        pulse_width = 500; // turn servo to the left
    }
    else if (distance > 20)
    {
        pulse_width = 2500; // turn servo to the right
    }
    else
    {
        pulse_width = 1500; // keep servo in the middle
    }

    gpioServo(SERVO_PIN, pulse_width);
}

int main()
{
    if (gpioInitialise() < 0)
    {
        std::cerr << "Failed to initialize GPIO\n";
        return 1;
    }

    gpioSetMode(TRIGGER_PIN, PI_OUTPUT);
    gpioSetMode(ECHO_PIN, PI_INPUT);
    gpioSetMode(SERVO_PIN, PI_OUTPUT);
    gpioServo(SERVO_PIN, 1500);

    while (1)
    {
        usleep(5000); // delay 10ms
        trigger_sonar();
        double distance = read_distance();
        control_servo(distance);
        std::cout << "Distance: " << distance << " cm\n";
    }

    gpioTerminate();
    return 0;
}