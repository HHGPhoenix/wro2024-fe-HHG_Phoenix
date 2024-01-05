from picamera2 import Picamera2
from libcamera import controls

picam2 = Picamera2()
picam2.start()
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})