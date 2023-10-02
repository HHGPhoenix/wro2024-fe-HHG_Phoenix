from RobotCarClasses import *

Ultraschall1 = SuperSonicSensor(24, 23, 1)
Ultraschall1.start_measurement()

Ultraschall2 = SuperSonicSensor(8, 25, 2)
Ultraschall2.start_measurement()

while True:
    #print(Ultraschall1.distance)
    #print(Ultraschall2.distance)
    time.sleep(0.01)