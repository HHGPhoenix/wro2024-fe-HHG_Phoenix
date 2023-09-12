from __future__ import print_function
import pixy
from ctypes import *
from pixy import *
from RobotCarClasses import *


GPIO.setmode(GPIO.BOARD)

Motor1 = Motor(1000, 13, 15, 11)
Motor1.start()

Servo1 = Servo(7, 50)

Pixy1 = PixyCam()
Pixy1.start_reading()


while 1:
    count = Pixy1.count
    
    P = 1
    
    x = Pixy1.output[0].m_x
    y = Pixy1.output[0].m_y
    
    
    if Pixy1.count == 0:
        Motor1.stop()
        
    elif Pixy1.count:
        Motor1.start()
        
        if x > 158 or x < 158:
            Error = x - 158
            Correction = P * Error
            
            if Correction > 100:
                Correction = 100
            elif Correction < -100:
                Correction = -100
            Servo1.steer(Correction)
            
        width = Pixy1.output[0].m_width
        height = Pixy1.output[0].m_height
        size = width * height
        print(f"Correction: {Correction}, X,Y Cords: {x,y}, size: {size}")
        
        middlesizevalue = 2500
        
        if size < middlesizevalue or size > middlesizevalue:
            Error = size - middlesizevalue
            Correction = P * Error
            
            if   Correction > 4:
                Motor1.drive("f", 100)
            elif Correction < 4:
                Motor1.drive("r", 100)
       



#middle: 158, 104 #158, 104 *2 for resulution      