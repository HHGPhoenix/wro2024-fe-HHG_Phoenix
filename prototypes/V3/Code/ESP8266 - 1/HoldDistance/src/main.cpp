#include <ESP.h>
#include <Ultrasonic.h>
#include <Servo.h>

// Pin definition sensor 1
#define TrigPin1 5
#define EchoPin1 4

// Pin definition sensor 2
#define TrigPin2 14
#define EchoPin2 12

// other Pin definitions
#define InternalLed 2 // LED on ESP8266 board
#define ServoPin 13 // Servo Pin

// Variables
int KP = 1; // Proportional constant
int maxDistanceCm = 200; // Max distance in cm to measure
int desiredDistance = 50; // Desired distance in cm
int activeSensor = 0; // select sensor 1 or 2
bool started = false; // switch between start and stop
bool manual = false; // switch between manual and automatic mode
bool firstCornerDetected = false; // special detection for drive direction
int ServoMiddlePosition = 90;
int distanceEdgeDetection = -1; // distance in cm to detect an edge
int edgeDetectionCounter = 0;

float distance1 = 0;
float distance2 = 0;

// Ultrasonic setup
Ultrasonic ultraschall1(TrigPin1, EchoPin1, 100000); // Trigger Pin, Echo Pin
Ultrasonic ultraschall2(TrigPin2, EchoPin2, 100000); // Trigger Pin, Echo Pin

Servo servo;

void setup() {
  Serial.begin(115200);
  pinMode(InternalLed, OUTPUT);
  digitalWrite(InternalLed, HIGH);
  servo.attach(ServoPin);
}

void loop() {
  // Wait for the "START" command 
  if (!started) {
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      if (command == "START") {
        Serial.println("Received START command. Performing action..."); // Feedback for Raspberry Pi
        servo.write(ServoMiddlePosition); // Reset servo position
        digitalWrite(InternalLed, LOW); // Turn on the internal LED

        firstCornerDetected = false; // Reset corner detection

        // Check for start in small section
        distance1 = ultraschall1.read();
        distance2 = ultraschall2.read();
        // Check if both sensors read a valid value
        if (distance1 > 0 && distance2 > 0) {
          // Check if the difference between the two sensors is greater than 5 cm
          if (abs(distance1 - distance2) > 10) {
            // Determine the drive direction based on the small section
            if (distance1 < distance2) {
              firstCornerDetected = true;
              delay(500);
              Serial.println("Drive direction clockwise");
            } else {
              firstCornerDetected = true;
              delay(500);
              Serial.println("Drive direction counterclockwise");
            }
          }
        }

        started = true; // Start the main loop
      }
      // Identity response
      else if (command == "IDENT") {
        Serial.println("HoldDistance");
      }

      delay(10);
    }
  }

  // Main loop if started and not in manual mode
  if (started) {
    if (!manual) {
      // command checker
      if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');

        // check for stop command
        if (command == "STOP") {
          Serial.println("Received STOP command. Performing action...");
          servo.write(ServoMiddlePosition);
          digitalWrite(InternalLed, HIGH);
          started = false;
        } 
        // check for change in desired distance
        else if (command.startsWith("D")) {
          int numberStart = 1; // Skip the "D" character
          int numberLength = command.length();
          String numberStr = command.substring(numberStart, numberLength);
          desiredDistance = numberStr.toInt();
          Serial.print("Received desired distance: ");
          Serial.println(desiredDistance);
        }
        // check for sensor switch
        else if (command.startsWith("S")) {
          int numberStart = 1; // Skip the "S" character
          int numberLength = command.length();
          String numberStr = command.substring(numberStart, numberLength);
          activeSensor = numberStr.toInt();

          if (activeSensor > 2) {
            activeSensor = 2;
          } else if (activeSensor < 1) {
            activeSensor = 1;
          }

          Serial.print("Received active sensor: ");
          Serial.println(activeSensor);
        }
        // check for KP change
        else if (command.startsWith("KP")) {
          int numberStart = 2; // Skip the "KP" characters
          int numberLength = command.length();
          String numberStr = command.substring(numberStart, numberLength);
          KP = numberStr.toInt();
          Serial.print("Received KP: ");
          Serial.println(KP);
        }
        // check for manual mode
        else if (command == "MANUAL") {
          Serial.println("Received MANUAL command. Activating manual mode...");
          manual = true;
        }
        // check for change in edge detection distance
        else if (command.startsWith("ED")) {
          int numberStart = 2; // Skip the "ED" characters
          int numberLength = command.length();
          String numberStr = command.substring(numberStart, numberLength);
          distanceEdgeDetection = numberStr.toInt();
          Serial.print("Received edge detection distance: ");
          Serial.println(distanceEdgeDetection);
        }
      }

      // Hold specified distance with specified sensor
      else if (activeSensor == 1) {
        distance1 = ultraschall1.read(); // get distance in cm
        if (distance1 > 0) {
          // calculate error and correction
          float error = desiredDistance - distance1;
          float correction = error * KP;

          //  limit correction to servo range
          if (correction > 70) {
            correction = 70;
          } else if (correction < -70) {
            correction = -70;
          }

          servo.write(ServoMiddlePosition - correction); // Set servo position

          if (!firstCornerDetected && (distanceEdgeDetection > 0)) {
            if (distance1 > distanceEdgeDetection) {
              firstCornerDetected = true;
              Serial.println("Drive direction clockwise");
            }
            // read other sensor sometimes
            if (edgeDetectionCounter == 4) {
              distance2 = ultraschall2.read();
              if (distance2 > distanceEdgeDetection) {
                firstCornerDetected = true;
                Serial.println("Drive direction counterclockwise");
              }
            } else {
              edgeDetectionCounter++;
            }
          }
        }
      } else if (activeSensor == 2) {
        distance2 = ultraschall2.read();
        if (distance2 > 0) {
          // calculate error and correction
          float error = desiredDistance - distance2;
          float correction = error * KP;

          //  limit correction to servo range
          if (correction > 70) {
            correction = 70;
          } else if (correction < -70) {
            correction = -70;
          }
          
          servo.write(ServoMiddlePosition + correction); // Set servo position

          if (!firstCornerDetected && (distanceEdgeDetection > 0)) {
            if (distance2 > distanceEdgeDetection) {
              firstCornerDetected = true;
              Serial.println("Drive direction counterclockwise");
            }
            // read other sensor sometimes
            if (edgeDetectionCounter == 4) {
              distance1 = ultraschall1.read();
              if (distance1 > distanceEdgeDetection) {
                firstCornerDetected = true;
                Serial.println("Drive direction clockwise");
              }
            } else {
              edgeDetectionCounter++;
            }
          }
        }
      } else {
        delay(10);
      }
    }

  // Main loop if started and in manual mode
    else {
      // command checker
      if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');

        // check for stop command
        if (command == "STOP") {
          Serial.println("Received STOP command. Performing action...");
          servo.write(ServoMiddlePosition);
          digitalWrite(InternalLed, HIGH);
          started = false;
        } 
        // check for manual mode
        else if (command == "MANUAL") {
          Serial.println("Received MANUAL command. Deactivating manual mode...");
          manual = false;
        } else {
          int numberLength = command.length();
          String numberStr = command.substring(0, numberLength);
          int ServoAngle = numberStr.toInt();
          servo.write(ServoAngle);
        }
        delay(10); // wait so the loop isn't too fast
      }
    }
  }
}