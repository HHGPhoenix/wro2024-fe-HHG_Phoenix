import cv2
import pyautogui
import numpy as np

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        screenshot = pyautogui.screenshot()
        frame = cv2.cvtColor(np.array(screenshot), cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        pixel_hsv = hsv[y, x]
        print("HSV Color: ", pixel_hsv)

# Create a blank image window
frame = None
cv2.namedWindow("Color Picker")

# Set the mouse callback function
cv2.setMouseCallback("Color Picker", mouse_callback)

screenshot = pyautogui.screenshot()

while True:
    # Capture the screen
    frame = cv2.cvtColor(np.array(screenshot), cv2.COLOR_RGB2BGR)

    # Display the frame
    cv2.imshow("Color Picker", frame)

    # Check for key press
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# Close all windows
cv2.destroyAllWindows()
