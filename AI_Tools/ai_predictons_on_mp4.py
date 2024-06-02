import tkinter as tk
from tkinter import filedialog
import cv2
import tensorflow as tf
import numpy as np
from tensorflow.keras.models import load_model

# Function to load and process video
def process_video(model_path, video_path):
    # Load the model
    model = load_model(model_path)

    # Assuming these are your class labels in the same order as the output
    class_labels = ['Green', 'Nothing', 'Red']

    # Open the video file
    cap = cv2.VideoCapture(video_path)

    # Check if video opened successfully
    if not cap.isOpened():
        print("Error opening video file")
        return

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
            # img_array = cv2.resize(blank_image, (320, 143)) / 255.0
            img_array = cv2.resize(blank_image, (224, 224)) / 255.0
            
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
            
            # Add "Press 'Q' to quit" text
            cv2.putText(blank_image, "Press 'Q' to quit", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Display probabilities of each class
            y_pos = 110
            for i, prob in enumerate(prediction[0]):
                cv2.putText(blank_image, f'{class_labels[i]}: {prob:.2f}', (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                y_pos += 40
            
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
    

# Tkinter UI
class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Model and Video Selector")

        self.model_path = ""
        self.video_path = ""

        self.label1 = tk.Label(root, text="Select Model File:")
        self.label1.pack()

        self.model_button = tk.Button(root, text="Browse", command=self.load_model)
        self.model_button.pack()

        self.model_label = tk.Label(root, text="")
        self.model_label.pack()

        self.label2 = tk.Label(root, text="Select Video File:")
        self.label2.pack()

        self.video_button = tk.Button(root, text="Browse", command=self.load_video)
        self.video_button.pack()

        self.video_label = tk.Label(root, text="")
        self.video_label.pack()

        self.start_button = tk.Button(root, text="Start", command=self.start_processing)
        self.start_button.pack()

    def load_model(self):
        self.model_path = filedialog.askopenfilename(filetypes=[("Keras Model", "*.keras"), ("All Files", "*.*")])
        self.model_label.config(text=self.model_path)

    def load_video(self):
        self.video_path = filedialog.askopenfilename(filetypes=[("MP4 files", "*.mp4"), ("All Files", "*.*")])
        self.video_label.config(text=self.video_path)

    def start_processing(self):
        if self.model_path and self.video_path:
            process_video(self.model_path, self.video_path)
        else:
            print("Please select both model and video files.")

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
