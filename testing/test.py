import time
import board
import adafruit_adxl34x
from RobotCarClasses import *

i2c = board.I2C()  # uses board.SCL and board.SDA

Motor1 = Motor(1000, 27, 17, 5)
Motor1.start()

Utils = Utility(None, None, None, Motor1)


# For ADXL345
accelerometer = adafruit_adxl34x.ADXL345(i2c)

accelerometer.offset = (5, 5, 7)

vx = 0.0
vy = 0.0
vz = 0
time2 = 0
VELOCITY = 0.1
P = -250
Brake = 10
correction = 0

dt = 0.01

direction = "f"

offsetx = accelerometer.acceleration[0]
offsety = accelerometer.acceleration[1]

while True:
    try:
        x = (accelerometer.acceleration[0] - offsetx)
        y = accelerometer.acceleration[1] - offsety
            
        vx = vx + x * dt
        vy = vy + y * dt
        
        error = vx - VELOCITY
        correction = P * error
        PWM = 25 + correction
        
        if PWM < 0: 
            PWM = 0
        if PWM > 40:
            PWM = 40
        vardir = 1
        """
        if PWM >= 25:
            direction = "f"
            vardir = 1
        elif PWM < 25:
            PWM = (25 - PWM) * Brake
            direction = "r"
            vardir = -1
        """
        # Open a text file for writing
        with open("sensor_data.txt", "a") as data_file:
            # Print correction value
            print("Correction:", correction)
            # Write values of x, y, vx, and vy to the text file
            data_file.write(f"x; {x}; y; {y}; vx; {vx}; vy; {vy}; correction; {correction}; PWM; {PWM}; dir; {vardir}\n")
            
        Motor1.drive("f", abs(PWM))
        
        time.sleep(dt)
        
    except:
        Utils.cleanup()

        
