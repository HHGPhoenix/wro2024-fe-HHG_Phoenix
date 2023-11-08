#!/usr/bin/python3

import os
import time
from RobotCarClasses import *
import logging
from PIL import Image


def is_another_instance_running():
    return os.path.exists(lock_file)


lock_file = '/tmp/StandbyScript.lock'


if is_another_instance_running():
        logging.debug("Another instance is running. Exiting.")
        print("Another instance is running. Exiting.")
        os.kill(os.getpid(), signal.SIGKILL)


# Initialize the serial interface for I2C
serial = i2c(port=0, address=0x3C)  # Adjust the address if necessary

# Initialize the OLED device
device = sh1106(serial)

# Load your image
image = Image.open("Logo.png")

# Resize the image to match the OLED display's resolution if needed
# The SH1106 commonly has a resolution of 128x64 pixels
image = image.resize(device.size)

# Display the image on the OLED screen
with canvas(device) as draw:
    draw.rectangle(device.bounding_box, outline="white", fill="white")
    draw.bitmap((0, 0), image, fill=0)

WaitTime = time.time() + 10
while time.time() < WaitTime:
    if is_another_instance_running():
        logging.debug("Another instance is running. Exiting.")
        print("Another instance is running. Exiting.")
        os.kill(os.getpid(), signal.SIGKILL)


logging.basicConfig(filename='/home/pi/AutoStart/logfile.log', level=logging.DEBUG)
logging.debug("StandbyScript.py started")


ADC = AnalogDigitalConverter()
Display = DisplayOled(ADC)
Display.start_update()


print("Starting StandbyScript.py")
while True:
    if is_another_instance_running():
        logging.debug("Another instance is running. Exiting.")
        print("Another instance is running. Exiting.")
        os.kill(os.getpid(), signal.SIGKILL)
    logging.debug("StandbyScript.py is running")
    print("StandbyScript.py is running")
    time.sleep(1)