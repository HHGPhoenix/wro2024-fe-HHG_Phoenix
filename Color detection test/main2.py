import cv2
import numpy as np
from picamera2 import Picamera2, Preview

picam2 = Picamera2()

config = picam2.create_still_configuration()
picam2.configure(config)

picam2.start()

# Define the color ranges for green and red
lower_green = np.array([35, 100, 100])
upper_green = np.array([85, 255, 255])

lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])

lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([180, 255, 255])

# Define the kernel for dilation
kernel = np.ones((5, 5), np.uint8)

while True:
    # Read one frame from the camera
    frame = picam2.capture_array()

    # Convert the image from BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask of pixels within the green color range
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    # Create a mask of pixels within the red color range
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # Dilate the masks to merge nearby areas
    mask_green = cv2.dilate(mask_green, kernel, iterations=1)
    mask_red = cv2.dilate(mask_red, kernel, iterations=1)

    # Find contours in the green mask
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find contours in the red mask
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize the block array
    block_array = []

    # Process each green contour
    for contour in contours_green:
        x, y, w, h = cv2.boundingRect(contour)
        if w > 50 and h > 50:  # Only consider boxes larger than 50x50
            block_array.append({'color': 'green', 'x': x, 'y': y, 'w': w, 'h': h})

    # Process each red contour
    for contour in contours_red:
        x, y, w, h = cv2.boundingRect(contour)
        if w > 50 and h > 50:  # Only consider boxes larger than 50x50
            block_array.append({'color': 'red', 'x': x, 'y': y, 'w': w, 'h': h})

    # Print the block array
    for block in block_array:
        print(block)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and destroy all windows
cap.release()
picam2.stop_preview()