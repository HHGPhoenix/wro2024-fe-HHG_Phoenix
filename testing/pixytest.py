from __future__ import print_function
import pixy 
from ctypes import *
from pixy import *
from RobotCarClasses import *

# Pixy2 Python SWIG get blocks example #

print("Pixy2 Python SWIG Example -- Get Blocks")

pixy.init ()
pixy.change_prog ("color_connected_components");

class Blocks (Structure):
    _fields_ = [ ("m_signature", c_uint),
        ("m_x", c_uint),
        ("m_y", c_uint),
        ("m_width", c_uint),
        ("m_height", c_uint),
        ("m_angle", c_uint),
        ("m_index", c_uint),
        ("m_age", c_uint) ]


GPIO.setmode(GPIO.BOARD)

Motor1 = Motor(1000, 13, 15, 11)
Motor1.start()

Servo1 = Servo(7, 50)


blocks = BlockArray(100)
frame = 0

while 1:
    count = pixy.ccc_get_blocks (100, blocks)

    if count > 0:
      #print('frame %3d:' % (frame))
      frame = frame + 1
      #for index in range (0, count):
        #print('[BLOCK: SIG=%d X=%3d Y=%3d WIDTH=%3d HEIGHT=%3d]' % (blocks[index].m_signature, blocks[index].m_x, blocks[index].m_y, blocks[index].m_width, blocks[index].m_height))
        
    P = 1
    
    x = blocks[0].m_x
    y = blocks[0].m_y
    
    
    if pixy.ccc_get_blocks(100, blocks) == 0:
        Motor1.stop()
        
    elif pixy.ccc_get_blocks(100, blocks):
        Motor1.start()
        

    
        if x > 158 or x < 158:
            Error = x - 158
            Correction = P * Error
            
            #print(f"Correction: {Correction}, X,Y Cords: {x,y}")
            
            if Correction > 100:
                Correction = 100
            elif Correction < -100:
                Correction = -100
            Servo1.steer(Correction)
            
        width = blocks[0].m_width
        height = blocks[0].m_height
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