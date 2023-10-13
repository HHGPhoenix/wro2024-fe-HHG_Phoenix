import multiprocessing as mp
from RobotCarClasses import *
import time

Farbsensor = ColorSensor()


Motor1 = Motor(1000, 27, 17, 22)
Motor1.start()

Ultraschall1 = SuperSonicSensor(24, 23, 1)
#Ultraschall1.start_measurement()

Ultraschall2 = SuperSonicSensor(8, 25, 2)


Servo1 = Servo(18, 50)

StartButton = Button(7)

StopButton = Button(26)


#Gyro = Gyroscope()
ADC = AnalogDigitalConverter()
Display = DisplayOled(ADC)


Utils = Utility()
Funcs = Functions(Utils, Ultraschall1, Ultraschall2, Farbsensor, Motor1, Servo1, StartButton, StopButton, Display)
Utils.transferSensorData(Ultraschall1, Ultraschall2, Farbsensor, Motor1, Servo1, StartButton, StopButton, Funcs, Display)
with open(Utils.file_path, 'w'):
    pass  # Using 'pass' as a placeholder for no content
time.sleep(1)

#p = mp.Process(target=Display.start_update())
#p.start()
p1 = mp.Process(target=StopButton.start_StopButton())
p1.start()
p2 = mp.Process(target=Farbsensor.start_measurement())
p2.start()
p3 = mp.Process(target=Ultraschall2.start_measurement())
p3.start()

Utils.StartRun(70, 0)

while Utils.running and Funcs.rounds < 3:
        time.sleep(0.01)
        try:
            Funcs.HoldDistance(40, False, 5, 70, "f", 2500)
            
        except Exception as e:
            print(e)
            Utils.StopRun()

time.sleep(10)
Utils.StopRun()