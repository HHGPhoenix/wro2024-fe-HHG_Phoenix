#include <Servo.h>

Servo servo;

int minAngle = 0;
int maxAngle = 180;
int angleStep = 1; // Adjust the step size as needed

void setup() {
  Serial.begin(9600);
  servo.attach(9); // Attach the servo to pin 9
}

void loop() {
  for (int angle = minAngle; angle <= maxAngle; angle += angleStep) {
    servo.write(angle);
    delay(10); // Adjust the delay as needed to set the speed of the sweep
  }

  for (int angle = maxAngle; angle >= minAngle; angle -= angleStep) {
    servo.write(angle);
    delay(10);
  }

  // Once the sweep is complete, report the minimum and maximum angles
  Serial.print("Minimum Angle: ");
  Serial.println(minAngle);
  Serial.print("Maximum Angle: ");
  Serial.println(maxAngle);

  while (true) {
    // Keep the Arduino running without performing any further action
  }
}