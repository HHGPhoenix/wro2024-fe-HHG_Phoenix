import cv2
import numpy as np

rect_endpoint_tmp = []
drawing = False

def draw_rectangle(event, x, y, flags, param):
    global rect_endpoint_tmp, drawing

    if event == cv2.EVENT_LBUTTONDOWN:
        if drawing is False:
            drawing = True
            rect_endpoint_tmp = [(x, y)]

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing is True:
            rect_endpoint_tmp[1:] = [(x, y)]  # Update the second element

    elif event == cv2.EVENT_LBUTTONUP:
        if drawing is True:
            drawing = False
            rect_endpoint_tmp[1:] = [(x, y)]  # Update the second element

# Open the camera
cap = cv2.VideoCapture(1)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error opening video stream or file")

cv2.namedWindow('Frame')
cv2.setMouseCallback('Frame', draw_rectangle)

while True:
    # Read one frame from the camera
    ret, frame = cap.read()

    if ret:
        # Convert the image from BGR to HSV
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if len(rect_endpoint_tmp) == 2:
            cv2.rectangle(frame, rect_endpoint_tmp[0], rect_endpoint_tmp[1], (0, 255, 0), 2)
            roi = frame[rect_endpoint_tmp[0][1]:rect_endpoint_tmp[1][1], rect_endpoint_tmp[0][0]:rect_endpoint_tmp[1][0]]
            if roi.size != 0:
                avg_color_per_row = np.average(roi, axis=0)
                avg_color = np.average(avg_color_per_row, axis=0)
                print("Average color: ", avg_color)

        # Convert the image back to BGR for displaying
        frame = cv2.cvtColor(frame, cv2.COLOR_HSV2BGR)

        # Display the resulting frame
        cv2.imshow('Frame', frame)

        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release the camera
cap.release()

# Close all OpenCV windows
cv2.destroyAllWindows()