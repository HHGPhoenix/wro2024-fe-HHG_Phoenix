from RobotCarClasses import *
import os

#clear terminal
os.system('cls' if os.name == 'nt' else 'clear')

#init sensors
Ultraschall1 = SuperSonicSensor(24, 23, 1)

Ultraschall2 = SuperSonicSensor(8, 25, 2)


ADC = AnalogDigitalConverter()
Display = DisplayOled(ADC)
Display.start_update()

Utils = Utility()
Utils.transferSensorData(Ultraschall1, Ultraschall2)


while True:
    StartTime = time.time()
    Utils.toggle_supersonic_sensor(1)
    dist1 = Ultraschall1.distance
    
    Utils.toggle_supersonic_sensor(2)
    dist2 = Ultraschall2.distance
    StopTime = time.time()
    print(f"Time: {StopTime - StartTime}")
    
    Display.write(f"Dist1: {dist1}", f"Dist2: {dist2}")