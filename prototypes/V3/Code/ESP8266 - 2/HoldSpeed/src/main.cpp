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
int slots = 14;
int kp = 5;
int standing = 0;
int lastspeed = 0;
float desiredSpeed = 0;
float Speed = 0;
float previousError = 0;
float kd = 0.5;

bool started = false;
bool turned = false;
bool forward = true;

// Objects
L298N motor(ENA, IN1, IN2);
LM393SpeedSensor speedSensor(speedSensorSignal);


void holdSpeed() {
  float currentSpeed = speedSensor.rps;
  float error = desiredSpeed - currentSpeed / 10;
  float derivative = error - previousError;
  float output = desiredSpeed + error * kp + derivative * kd;
  // 255 = 50 + (35 - 0 / 10) * 10
  Serial.print("Current Speed: " + String(currentSpeed));
  Serial.print(" | Error: " + String(error));
  Serial.print(" | Output: " + String(output));
  Serial.println(" | Desired Speed: " + String(desiredSpeed));

  // PWM limit
  if (output < 0) {
    output = 0;
  } else if (output > 255) {
    output = 255;
  }

  // Turn the motor
  if (output > 0) {
    motor.forward();
  } else {
    motor.backward();
  }
  motor.setSpeed(abs(output));

  previousError = error;
}


void setup() {
  Serial.begin(1000000);
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
        motor.setSpeed(255);
        motor.forward();
        delay(100);
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
      else if (command.startsWith("SPEED")) {
        int numberStart = 5;
        int numberLength = command.length();
        String numberStr = command.substring(numberStart, numberLength);
        Speed = numberStr.toInt();
        if (Speed < 0) {
          forward = false;
        } else {
          forward = true;
        }

        desiredSpeed = Speed;

        Serial.print("Received SPEED: ");
        Serial.println(desiredSpeed);
      }
      //check for KP command
      else if (command.startsWith("KP")) {
        int numberStart = 2;
        int numberLength = command.length();
        String numberStr = command.substring(numberStart, numberLength);
        kp = numberStr.toInt();

        Serial.print("Received KP: ");
        Serial.println(kp);
      }
      //check for KD command
      else if (command.startsWith("KD")) {
        int numberStart = 2;
        int numberLength = command.length();
        String numberStr = command.substring(numberStart, numberLength);
        kd = numberStr.toInt();

        Serial.print("Received KD: ");
        Serial.println(kd);
      }
    }

    holdSpeed();

    delay(5);
  }
}