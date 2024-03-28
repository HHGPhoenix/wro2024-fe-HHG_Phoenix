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
int KP = 1;						  // Proportional constant
int maxDistanceCm = 200;		  // Max distance in cm to measure
int desiredDistance = 50;		  // Desired distance in cm
int commandedDistance = 50;		  // Distance in cm commanded by the Raspberry Pi
int activeSensor = 0;			  // select sensor 1 or 2
int smoothingSteps = 1;			  // smoothing multiplier for sensor values
bool started = false;			  // switch between start and stop
bool manual = false;			  // switch between manual and automatic mode
bool firstCornerDetected = false; // special detection for drive direction
int ServoMiddlePosition = 90;
int distanceEdgeDetection = -1; // distance in cm to detect an edge
int edgeDetectionCounter = 0;
float distance1 = 0;
int distance1counter = 0;
float distance2 = 0;
int distance2counter = 0;
String command;
char c;

// Ultrasonic setup
Ultrasonic ultraschall1(TrigPin1, EchoPin1, 100000); // Trigger Pin, Echo Pin
Ultrasonic ultraschall2(TrigPin2, EchoPin2, 100000); // Trigger Pin, Echo Pin

Servo servo;

void setup()
{
	Serial.begin(1000000); // Start serial communication
	pinMode(InternalLed, OUTPUT);
	digitalWrite(InternalLed, HIGH);
	servo.attach(ServoPin);
}

void loop()
{
	// Wait for the "START" command
	if (!started)
	{
		while (Serial.available() > 0)
		{
			c = Serial.read();
			// Serial.println(c);
			if (c == '\n')
			{
				if (command == "START")
				{
					Serial.println("Received START command. Performing action..."); // Feedback for Raspberry Pi
					servo.write(ServoMiddlePosition);								// Reset servo position
					digitalWrite(InternalLed, LOW);									// Turn on the internal LED

					firstCornerDetected = false; // Reset corner detection

					// Check for start in small section
					distance1 = ultraschall1.read();
					distance2 = ultraschall2.read();
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
				//Serial.print("c: ");
				//Serial.println(c);
				if (c == '\n')
				{
					//command = String(command);
					//Serial.println("command");
					//Serial.println(command);
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
						KP = numberStr.toInt();
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
						smoothingSteps = numberStr.toInt();
						Serial.print("Received smoothing multiplier: ");
						Serial.println(smoothingSteps);
					}
					// heartbeat response
					else if (command == "H")
					{
						Serial.println("HB");
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

				//Serial.print("D1: ");
				//Serial.println(distance1);

				if (commandedDistance > 0)
				{
					if (desiredDistance > (commandedDistance + smoothingSteps - 1))
					{
						desiredDistance = desiredDistance - smoothingSteps;
					}
					else if (desiredDistance < (commandedDistance - smoothingSteps + 1))
					{
						desiredDistance = desiredDistance + smoothingSteps;
					}
					else
					{
						desiredDistance = commandedDistance;
					}
				}

				// Serial.print("commandedDistance: ");
				// Serial.println(commandedDistance);

				// Serial.print("desiredDistance: ");
				// Serial.println(desiredDistance);

				if (distance1 > 0)
				{
					// calculate error and correction
					float error = desiredDistance - distance1;
					float correction = error * KP * -1;

					// Serial.print("correction: ");
					// Serial.println(correction);

					//  limit correction to servo range
					if (correction > 50)
					{
						correction = 50;
					}
					else if (correction < -60)
					{
						correction = -60;
					}

					servo.write(ServoMiddlePosition - correction); // Set servo position

					if (!firstCornerDetected && (distanceEdgeDetection > 0))
					{
						if (distance1 > distanceEdgeDetection)
						{
							firstCornerDetected = true;
							Serial.println("Drive direction clockwise");
						}
						// read other sensor sometimes
						if (edgeDetectionCounter == 4)
						{
							distance2 = ultraschall2.read();
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
					if (desiredDistance > (commandedDistance + smoothingSteps - 1))
					{
						desiredDistance = desiredDistance - smoothingSteps;
					}
					else if (desiredDistance < (commandedDistance - smoothingSteps + 1))
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

					//  limit correction to servo range
					if (correction > 60)
					{
						correction = 60;
					}
					else if (correction < -50)
					{
						correction = -50;
					}

					servo.write(ServoMiddlePosition + correction); // Set servo position

					if (!firstCornerDetected && (distanceEdgeDetection > 0))
					{
						if (distance2 > distanceEdgeDetection)
						{
							firstCornerDetected = true;
							Serial.println("Drive direction counterclockwise");
						}
						// read other sensor sometimes
						if (edgeDetectionCounter == 4)
						{
							distance1 = ultraschall1.read();
							if (distance1 > distanceEdgeDetection)
							{
								firstCornerDetected = true;
								Serial.println("Drive direction clockwise");
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
			if (Serial.available() > 0)
			{
				String command = Serial.readStringUntil('\n');

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
				else
				{
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