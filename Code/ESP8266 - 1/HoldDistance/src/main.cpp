#include <ESP.h>
#include <Ultrasonic.h>
#include <Wire.h>
#include <Servo.h>

// Pin definition sensor 1
#define TrigPin1 5
#define EchoPin1 4

// Pin definition sensor 2
#define TrigPin2 14
#define EchoPin2 12

// other Pin definitions
#define InternalLed 2 // LED on ESP8266 board
#define ServoPin 13	  // Servo Pin

// Variables
float KP = 1;					  // Proportional constant
float desiredDistance = 50;		  // Desired distance in cm
int commandedDistance = 50;		  // Distance in cm commanded by the Raspberry Pi
int activeSensor = 0;			  // select sensor 1 or 2
float smoothingSteps = 1;		  // smoothing multiplier for sensor values
int timeoutcounter = 0;			  // counter for timeout of sensors
bool started = false;			  // switch between start and stop
bool manual = false;			  // switch between manual and automatic mode
bool firstCornerDetected = false; // special detection for drive direction
int ServoMiddlePosition = 80;	  // middle position of the servo
int angle_right = 45;			  // max angle to the right
int angle_left = 55;			  // max angle to the left
int distanceEdgeDetection = -1;	  // distance in cm to detect an edge
int edgeDetectionCounter = 0;	  // counter for edge detection
float distance1 = 0;			  // distance sensor 1
int distance1counter = 0;		  // counter for distance sensor 1
float distance2 = 0;			  // distance sensor 2
int distance2counter = 0;		  // counter for distance sensor 2
const int numReadings = 5;		  // number of readings to keep track of
float readings[numReadings];	  // the readings from the analog input
int readIndex = 0;				  // the index of the current reading
float total = 0;				  // the running total
float average = 0;				  // the average
const int numReadings2 = 5;		  // number of readings to keep track of
float readings2[numReadings2];	  // the readings from the analog input
int readIndex2 = 0;				  // the index of the current reading
float total2 = 0;				  // the running total
float average2 = 0;				  // the average
String command;					  // command string
char c;							  // character for command string

// Ultrasonic setup
Ultrasonic ultraschall1(TrigPin1, EchoPin1, 100000); // Trigger Pin, Echo Pin
Ultrasonic ultraschall2(TrigPin2, EchoPin2, 100000); // Trigger Pin, Echo Pin

Servo servo;

void setup()
{
	Serial.begin(921600); // Start serial communication
	pinMode(InternalLed, OUTPUT);
	digitalWrite(InternalLed, HIGH);
	servo.attach(ServoPin);
	servo.write(ServoMiddlePosition); // Set servo to middle position
}

void loop()
{
	// Wait for the "START" command
	if (!started)
	{
		while (Serial.available() > 0)
		{
			c = Serial.read();
			if (c == '\n')
			{
				if (command == "START")
				{
					Serial.println("Received START command. Performing action..."); // Feedback for Raspberry Pi
					servo.write(ServoMiddlePosition);								// Reset servo position
					digitalWrite(InternalLed, LOW);									// Turn on the internal LED

					firstCornerDetected = false; // Reset corner detection

					distance1 = 0;
					distance2 = 0;

					timeoutcounter = 0;
					// Check for start in small section
					while (distance1 == 0)
					{
						distance1 = ultraschall1.read();
						timeoutcounter++;
						if (timeoutcounter > 5)
						{
							break;
						}
					}

					timeoutcounter = 0;
					while (distance2 == 0)
					{
						distance2 = ultraschall2.read();
						timeoutcounter++;
						if (timeoutcounter > 5)
						{
							break;
						}
					}

					// Check if both sensors read a valid value
					if (distance1 > 0 && distance2 > 0)
					{
						// Check if the difference between the two sensors is greater than 5 cm
						if (abs(distance1 - distance2) > 10)
						{
							// Determine the drive direction based on the small section
							if (distance1 < distance2)
							{
								firstCornerDetected = true;
								delay(500);
								Serial.println("Drive direction clockwise");
							}
							else
							{
								firstCornerDetected = true;
								delay(500);
								Serial.println("Drive direction counterclockwise");
							}
						}
					}

					started = true; // Start the main loop
				}

				// Identity response
				else if (command == "IDENT")
				{
					Serial.println("HoldDistance");
				}
				// heartbeat response
				else if (command == "H")
				{
					delay(5); // Add a delay before sending the heartbeat response
					Serial.println("HB");
				}
				command = ""; // Reset the command string
			}
			else
			{
				command += c; // Append the character to the command string
			}
		}
		delay(10); // wait so the loop isn't too fast
	}
	// Main loop if started and not in manual mode
	else if (started)
	{
		if (!manual)
		{
			while (Serial.available() > 0)
			{
				c = Serial.read();
				if (c == '\n')
				{
					// check for stop command
					if (command == "STOP")
					{
						Serial.println("Received STOP command. Performing action...");
						servo.write(ServoMiddlePosition);
						digitalWrite(InternalLed, HIGH);
						started = false;
						activeSensor = 0;
					}
					// check for change in desired distance
					else if (command.startsWith("D"))
					{
						int numberStart = 1; // Skip the "D" character
						int numberLength = command.length();
						String numberStr = command.substring(numberStart, numberLength);
						commandedDistance = numberStr.toInt();
						Serial.print("RD: ");
						Serial.println(commandedDistance);
					}
					// check for sensor switch
					else if (command.startsWith("S"))
					{
						int numberStart = 1; // Skip the "S" character
						int numberLength = command.length();
						String numberStr = command.substring(numberStart, numberLength);
						activeSensor = numberStr.toInt();

						if (activeSensor > 2)
						{
							activeSensor = 2;
						}
						else if (activeSensor < 1)
						{
							activeSensor = 1;
						}

						Serial.print("RS: ");
						Serial.println(activeSensor);
					}
					// check for KP change
					else if (command.startsWith("KP"))
					{
						int numberStart = 2; // Skip the "KP" characters
						int numberLength = command.length();
						String numberStr = command.substring(numberStart, numberLength);
						KP = numberStr.toFloat();
						Serial.print("Received KP: ");
						Serial.println(KP);
					}
					// check for manual mode
					else if (command == "MANUAL")
					{
						Serial.println("Received MANUAL command. Activating manual mode...");
						manual = true;
					}
					// check for change in edge detection distance
					else if (command.startsWith("ED"))
					{
						int numberStart = 2; // Skip the "ED" characters
						int numberLength = command.length();
						String numberStr = command.substring(numberStart, numberLength);
						distanceEdgeDetection = numberStr.toInt();
						Serial.print("Received edge detection distance: ");
						Serial.println(distanceEdgeDetection);
					}
					// check for smoothing steps
					else if (command.startsWith("MM"))
					{
						int numberStart = 2; // Skip the "SM" characters
						int numberLength = command.length();
						String numberStr = command.substring(numberStart, numberLength);
						smoothingSteps = numberStr.toFloat();
						Serial.print("Received smoothing multiplier: ");
						Serial.println(smoothingSteps);
					}
					// heartbeat response
					else if (command == "H")
					{
						Serial.println("HB");
					}
					else if (command.startsWith("ANGL"))
					{
						int numberStart = 4; // Skip the "ANGL" characters
						int numberLength = command.length();
						String numberStr = command.substring(numberStart, numberLength);
						angle_left = numberStr.toInt();

						if (angle_left > 55)
						{
							angle_left = 55;
						}

						Serial.print("angle_left: ");
						Serial.println(angle_left);
					}
					else if (command.startsWith("ANGR"))
					{
						int numberStart = 4; // Skip the "ANGR" characters
						int numberLength = command.length();
						String numberStr = command.substring(numberStart, numberLength);
						angle_right = numberStr.toInt();

						if (angle_right > 45)
						{
							angle_right = 45;
						}

						Serial.print("angle_right: ");
						Serial.println(angle_right);
					}

					// Serial.println(command);
					command = "";
				}
				else
				{
					command += c; // Append the character to the command string
				}
			}

			// Hold specified distance with specified sensor
			if (activeSensor == 1)
			{
				distance1 = ultraschall1.read(); // get distance in cm

				// Serial.print("D1: ");
				// Serial.println(distance1);

				if (commandedDistance > 0)
				{
					if (desiredDistance > (commandedDistance + smoothingSteps - smoothingSteps / 2))
					{
						desiredDistance = desiredDistance - smoothingSteps;
					}
					else if (desiredDistance < (commandedDistance - smoothingSteps + smoothingSteps / 2))
					{
						desiredDistance = desiredDistance + smoothingSteps;
					}
					else
					{
						desiredDistance = commandedDistance;
					}
				}

				Serial.print("desiredDistance: ");
				Serial.println(desiredDistance);

				// Serial.print("commandedDistance: ");
				// Serial.println(commandedDistance);

				// Serial.print("desiredDistance: ");
				// Serial.println(desiredDistance);

				if (distance1 > 0)
				{
					// calculate error and correction
					float error = desiredDistance - distance1;
					float correction = error * KP;

					// subtract the last reading:
					total = total - readings[readIndex];
					// read from the sensor:
					readings[readIndex] = correction;
					// add the reading to the total:
					total = total + readings[readIndex];
					// advance to the next position in the array:
					readIndex = readIndex + 1;

					// if we're at the end of the array...
					if (readIndex >= numReadings)
					{
						// ...wrap around to the beginning:
						readIndex = 0;
					}

					// calculate the average:
					average = total / numReadings;
					// send it to the computer as ASCII digits

					// limit correction to servo range
					if (average > angle_right)
					{
						average = angle_right;
					}
					else if (average < -angle_left)
					{
						average = -angle_left;
					}

					servo.write(int(ServoMiddlePosition - average)); // Set servo position

					if (!firstCornerDetected && (distanceEdgeDetection > 0))
					{
						if (distance1 > distanceEdgeDetection)
						{
							firstCornerDetected = true;
							Serial.println("Drive direction clockwise");
							activeSensor = 1;
						}
						// read other sensor sometimes
						if (edgeDetectionCounter == 1)
						{
							distance2 = ultraschall2.read();

							if (distance2 > distanceEdgeDetection)
							{
								firstCornerDetected = true;
								Serial.println("Drive direction counterclockwise");
								activeSensor = 2;
							}
						}
						else
						{
							edgeDetectionCounter++;
						}
					}
				}
				delay(10); // wait so the loop isn't too fast
			}
			else if (activeSensor == 2)
			{
				distance2 = ultraschall2.read();

				if (commandedDistance > 0)
				{
					if (desiredDistance > (commandedDistance + smoothingSteps - smoothingSteps / 2))
					{
						desiredDistance = desiredDistance - smoothingSteps;
					}
					else if (desiredDistance < (commandedDistance - smoothingSteps + smoothingSteps / 2))
					{
						desiredDistance = desiredDistance + smoothingSteps;
					}
					else
					{
						desiredDistance = commandedDistance;
					}
				}

				if (distance2 > 0)
				{
					// calculate error and correction
					float error = desiredDistance - distance2;
					float correction = error * KP;

					// subtract the last reading:
					total2 = total2 - readings2[readIndex2];
					// read from the sensor:
					readings2[readIndex2] = correction;
					// add the reading to the total:
					total2 = total2 + readings2[readIndex2];
					// advance to the next position in the array:
					readIndex2 = readIndex2 + 1;

					// if we're at the end of the array...
					if (readIndex2 >= numReadings2)
					{
						readIndex2 = 0;
					}

					// calculate the average:
					average2 = total2 / numReadings2;

					// limit correction to servo range
					if (average2 > angle_left)
					{
						average2 = angle_left;
					}
					else if (average2 < -angle_right)
					{
						average2 = -angle_right;
					}

					servo.write(int(ServoMiddlePosition + average2)); // Set servo position

					if (!firstCornerDetected && (distanceEdgeDetection > 0))
					{
						if (distance2 > distanceEdgeDetection)
						{
							firstCornerDetected = true;
							Serial.println("Drive direction counterclockwise");
							activeSensor = 2;
						}
						// read other sensor sometimes
						if (edgeDetectionCounter == 1)
						{
							distance1 = ultraschall1.read();
							if (distance1 > distanceEdgeDetection)
							{
								firstCornerDetected = true;
								Serial.println("Drive direction clockwise");
								activeSensor = 1;
							}
						}
						else
						{
							edgeDetectionCounter++;
						}
					}
				}
				delay(10); // wait so the loop isn't too fast
			}
		}
		// Main loop if started and in manual mode
		else
		{
			// command checker
			while (Serial.available() > 0)
			{
				c = Serial.read();
				if (c == '\n')
				{
					// check for stop command
					if (command == "STOP")
					{
						Serial.println("Received STOP command. Performing action...");
						servo.write(ServoMiddlePosition);
						digitalWrite(InternalLed, HIGH);
						started = false;
					}
					// check for manual mode
					else if (command == "MANUAL")
					{
						Serial.println("Received MANUAL command. Deactivating manual mode...");
						manual = false;
					}
					else if (command.startsWith("ANG"))
					{
						int numberLength = command.length();
						String numberStr = command.substring(3, numberLength);
						int ServoAngle = numberStr.toInt();
						servo.write(ServoAngle);
					}
					delay(10); // wait so the loop isn't too fast

					command = "";
				}
				else
				{
					command += c;
				}
			}
		}
	}
}