from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import sh1106
from PIL import Image
import time
import os

# Specify the file path
file_path = "/tmp/StandbyScript.lock"
with open(file_path, 'w'):
    pass  # Using 'pass' as a placeholder for no content

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
time.sleep(15)

os.remove(file_path)