from RobotCarClasses import *
import time

Utils = Utility()

Farbsensor = ColorSensor(Utils)
Farbsensor.start_measurement()

while True:
    print(Farbsensor.color_temperature)
    time.sleep(0.1)