import cv2
import numpy as np
import socket
import struct
from threading import Thread
import time
from picamera2 import Picamera2
from libcamera import controls


picam2 = Picamera2()

config = picam2.create_still_configuration(main={"size": (1280, 720)}, raw={"size": (1280, 720)}, controls={"FrameRate": 30})
picam2.configure(config)

picam2.start()
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})

# Set the VNC server address and port
VNC_SERVER_IP = '192.168.178.71'
VNC_SERVER_PORT = 5900

# Set the webcam index (0 by default, change if you have multiple webcams)
WEBCAM_INDEX = 0

# Set the screen resolution
SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480  # Adjust as needed

# Function to send image frames to VNC server
def send_frame(frame, client_socket):
    # Serialize the frame and send it to the server
    data = cv2.imencode('.jpg', frame)[1].tobytes()
    size = len(data)
    client_socket.sendall(struct.pack('>L', size) + data)

# Function to capture webcam feed and send it to VNC server
def webcam_to_vnc():
    while True:
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((VNC_SERVER_IP, VNC_SERVER_PORT))

            while True:
                frame = picam2.capture_array()

                # Resize the frame to match the screen resolution
                frame = cv2.resize(frame, (SCREEN_WIDTH, SCREEN_HEIGHT))

                # Send the frame to the VNC server
                send_frame(frame, client_socket)

                # Introduce a delay to control the frame rate
                time.sleep(0.1)

        except ConnectionResetError:
            print("Connection to VNC server reset. Reconnecting...")
            # Add a delay before attempting to reconnect
            time.sleep(1)

        finally:
            # Release the webcam and close the connection
            picam2.stop()
            client_socket.close()


# Function to receive control signals from VNC server
def receive_control():
    # Implement this function to receive control signals from the VNC server
    # You can use the `pyautogui` library to simulate mouse and keyboard events
    pass

if __name__ == "__main__":
    # Start a thread for capturing and sending webcam frames
    webcam_thread = Thread(target=webcam_to_vnc)
    webcam_thread.start()

    # Start a thread for receiving control signals from the VNC server
    control_thread = Thread(target=receive_control)
    control_thread.start()

    # Wait for the webcam thread to finish (indefinite loop)
    webcam_thread.join()
