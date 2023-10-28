#include <NewPing.h>
#include <Servo.h>

#define TRIGGER_PIN1 2
#define ECHO_PIN1 3
#define TRIGGER_PIN2 4
#define ECHO_PIN2 5
#define MAX_DISTANCE 200
#define SERVO_PIN 9

NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);
Servo servo;

float desiredDistance = 30.00; // Initial desired distance in centimeters with two decimal places
float measuredDistance = 0.00; // Initialize measured distance

int activeSensor = 1; // 1 represents the first sensor (pins 2 and 3), 2 represents the second sensor (pins 4 and 5)

void setup() {
  Serial.begin(9600); // Set the serial speed to 9600 baud
  servo.attach(SERVO_PIN);
}

void loop() {
  if (Serial.available() > 0) {
    // Check if there is serial data available
    char input = Serial.read();

    if (input == 'D') {
      // If the received character is 'D', it signifies that a new desired distance is coming
      if (Serial.available() >= 5) {
        // Make sure at least five characters are available for reading, including the decimal point
        desiredDistance = Serial.parseFloat(); // Read the new desired distance with two decimal places
      }
    } else if (input == 'S') {
      // If the received character is 'S', it signifies a sensor switch command
      int sensorToActivate = Serial.parseInt();
      if (sensorToActivate == 1 || sensorToActivate == 2) {
        activeSensor = sensorToActivate;
      }
    }
  }

  if (activeSensor == 1) {
    measuredDistance = sonar1.ping_cm(); // Measured distance from the first sensor
  } else if (activeSensor == 2) {
    measuredDistance = sonar2.ping_cm(); // Measured distance from the second sensor
  }

  if (measuredDistance > 0) {
    float error = desiredDistance - measuredDistance;
    int servoAngle = map(error, -30, 30, 0, 120); // Adjust the mapping range as needed
    servo.write(servoAngle);

    delay(100); // Adjust the delay as needed
  }
}
