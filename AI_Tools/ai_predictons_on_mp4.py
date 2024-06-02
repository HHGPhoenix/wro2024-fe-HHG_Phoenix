import cv2
import tensorflow as tf
import numpy as np
from tensorflow.keras.models import load_model

# Load the model
model = load_model('cube_classifier.keras')

# Assuming these are your class labels in the same order as the output
class_labels = ['Green', 'Nothing', 'Red']

# Open the video file
video_path = 'AI_testing\input.mp4'  # replace with your video path
cap = cv2.VideoCapture(video_path)

# Check if video opened successfully
if not cap.isOpened():
    print("Error opening video file")

# Create a resizable window
cv2.namedWindow('Frame', cv2.WINDOW_NORMAL)

# Desired resolution
desired_width = 1920
desired_height = 1080

# Read until video is completed
while cap.isOpened():
    # Capture frame-by-frame
    ret, frame = cap.read()
    if ret:
        # Calculate aspect ratio of the video
        height, width, _ = frame.shape
        # Calculate aspect ratio of the desired resolution
        aspect_ratio = desired_width / desired_height

        # Calculate new width and height
        new_width = int(height * aspect_ratio)
        new_height = height if new_width <= width else int(width / aspect_ratio)

        # Resize the frame
        frame = cv2.resize(frame, (new_width, new_height))

        # Create a blank image with desired resolution
        blank_image = np.zeros((desired_height, desired_width, 3), np.uint8)

        # Calculate the starting position to place the frame in the center of the blank image
        x_offset = (desired_width - new_width) // 2
        y_offset = (desired_height - new_height) // 2

        # Place the frame in the center of the blank image
        blank_image[y_offset:y_offset+new_height, x_offset:x_offset+new_width] = frame

        # Preprocess the frame for the model
        img_array = cv2.resize(blank_image, (320, 143)) / 255.0
        img_array = np.expand_dims(img_array, axis=0)
        # Preprocess the frame for the model
        frame = cv2.resize(frame, (320, 143))
        img_array = frame / 255.0
        img_array = np.expand_dims(img_array, axis=0)

        # Predict the class of the image
        prediction = model.predict(img_array)

        # Find the index of the highest probability
        predicted_index = np.argmax(prediction)

        # Get the class label
        predicted_class = class_labels[predicted_index]

        print(f'Predicted class: {predicted_class}')

        # Display the resulting frame with prediction
        cv2.putText(blank_image, f'Predicted class: {predicted_class}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('Frame', blank_image)

        # Make the window fullscreen
        cv2.setWindowProperty('Frame', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        # Press Q on keyboard to exit
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    else:
        break

# Release the video capture and video write objects
cap.release()

# Closes all the frames
cv2.destroyAllWindows()