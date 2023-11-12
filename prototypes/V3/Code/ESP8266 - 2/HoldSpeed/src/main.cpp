#include <L298N.h>
#include <LM393SpeedSensor.h>

// Motor Pins
#define ENA 14
#define IN1 15
#define IN2 12

// Speed Sensor Pin
#define speedSensorSignal 13

// Internal LED Pin
#define InternalLed 2

// Variables
int slots = 8;
float desiredSpeed = 0;
float Speed = 0;

bool started = false;
bool turned = false;
bool forward = true;

// Objects
L298N motor(ENA, IN1, IN2);
LM393SpeedSensor speedSensor(speedSensorSignal);


void holdSpeed() {
  float currentSpeed = speedSensor.rps;
  float error = desiredSpeed - currentSpeed / 10;
  float output = desiredSpeed + error * 5;

  // PWM limit
  if (output < 0) {
    output = 0;
  } else if (output > 255) {
    output = 255;
  }  

  // Turn the motor
  if (forward) {
    motor.forward();
  } else {
    motor.backward();
  }
  motor.setSpeed(output);
}


void setup() {
  Serial.begin(115200);
  pinMode(InternalLed, OUTPUT);
  digitalWrite(InternalLed, HIGH);
}

void loop() {
  // Wait for the "START" command
  if (!started) {
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      if (command == "START") {
        Serial.println("Received START command. Performing action...");
        digitalWrite(InternalLed, LOW);
        speedSensor.begin();
        turned = false;
        started = true;
      }
      // Identity response
      else if (command == "IDENT") {
        Serial.println("HoldSpeed");
      }

      delay(10);
    }
  }

  if (started) {
    // command checker
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');

      // check for stop command
      if (command == "STOP") {
        Serial.println("Received STOP command. Performing action...");
        digitalWrite(InternalLed, HIGH);
        speedSensor.reset();
        motor.setSpeed(0);
        motor.stop();
        started = false;
      } 
      // check for speed command
      if (command.startsWith("SPEED")) {
        int numberStart = 5;
        int numberLength = command.length();
        String numberStr = command.substring(numberStart, numberLength);
        Speed = numberStr.toInt();
        if (Speed < 0) {
          forward = false;
        } else {
          forward = true;
        }

        desiredSpeed = abs(Speed);

        Serial.print("Received SPEED: ");
        Serial.println(desiredSpeed);
      }
    }

    holdSpeed();

    delay(5);
  }
}