from RobotCarClasses import *
import time

# rev.1 users set port=0
# substitute spi(device=0, port=0) below if using that interface

ADC = AnalogDigitalConverter()
Display = DisplayOled(ADC)

Display.write("Hello World!", "HHG_Phoenix")
Display.start_update()

Farbsensor = ColorSensor()
Farbsensor.start_measurement()

#Ultraschall1 = SuperSonicSensor(24, 23)
#Ultraschall1.start_measurement()

Ultraschall2 = SuperSonicSensor(8, 25)
Ultraschall2.start_measurement()


while True:
    #print(Farbsensor.color_temperature)
    Display.write(f"ColorT: {Farbsensor.color_temperature}", f"Dist: {Ultraschall2.distance}")
    time.sleep(0.01)