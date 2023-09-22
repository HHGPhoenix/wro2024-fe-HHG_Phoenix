import adafruit_tcs34725
import time
import busio
import board

i2c = busio.I2C(3, 2)
#while not i2c.try_lock():
#    pass
print(i2c.scan())