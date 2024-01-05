from flask import Flask, render_template, Response
import cv2
import numpy as np
from picamera2 import Picamera2
from libcamera import controls
import time
import psutil

app = Flask(__name__)

# Open the video camera. Make sure the correct camera index is used. It might be 0 or 1.
picam2 = Picamera2()

config = picam2.create_still_configuration(main={"size": (1280, 720)}, raw={"size": (1280, 720)}, controls={"FrameRate": 30})
picam2.configure(config)

picam2.start()
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})

# Define the color boundaries
lower_green = np.array([35, 45, 55])
upper_green = np.array([85, 255, 255])
lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([180, 255, 255])

@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')

def gen():
    """Video streaming generator function."""
    while True:
        StartTime = time.time()
        time.sleep(0.000001)
        frame = picam2.capture_array()
        
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Convert the image from BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create a mask of pixels within the green color range
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        # Create a mask of pixels within the red color range
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        # Here you can add code to process the masks if needed
            # Find contours in the green mask
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find contours in the red mask
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Initialize the block array
        block_array = []

        # Draw a bounding box around each green contour
        for contour in contours_green:
            x, y, w, h = cv2.boundingRect(contour)
            if w > 50 and h > 50:  # Only consider boxes larger than 50x50
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, 'Green Object', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)
                block_array.append({'color': 'green', 'x': x, 'y': y, 'w': w, 'h': h})

        # Draw a bounding box around each red contour
        for contour in contours_red:
            x, y, w, h = cv2.boundingRect(contour)
            if w > 50 and h > 50:  # Only consider boxes larger than 50x50
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(frame, 'Red Object', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,255), 2)
                block_array.append({'color': 'red', 'x': x, 'y': y, 'w': w, 'h': h})
        
        # Print the block array
        for block in block_array:
            print(block)
        
            
        # Encode the frame for streaming
        (flag, encodedImage) = cv2.imencode(".jpg", frame)
        yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage) + b'\r\n')

        print(f"FPS: {1.0 / (time.time() - StartTime)} | CPU: {psutil.cpu_percent()} | RAM: {psutil.virtual_memory().percent}")
        

@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', threaded=True)