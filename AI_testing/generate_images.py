import cv2
import os
import time

global file_path, output_path
file_path = r"C:\Users\felix\Downloads\Videos Runs\alles Licht, fenster zu, Blöcke, kein Filter.mp4"
# output_path = r"D:\Datasets\WRO Beta test\not_categorized\alles Licht, fenster zu, Blöcke, kein Filter"
output_path = r"C:\Users\felix\Downloads\Videos Runs\test_images"


def generate_images():
    # Load the video
    cap = cv2.VideoCapture(file_path)

    time.sleep(1)
    
    try:
        while True:
            ret, frameraw = cap.read()
            if not ret:
                break
            
            print(frameraw.shape)

            frame = frameraw.copy()
            
            print(frame.shape)  
            
            # Save the frame
            frame_number = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
            print(f"Frame {frame_number}")
            output_file = os.path.join(output_path, f"frame{frame_number}.jpg")
            print("Saving to", output_file)
            # Check if directory exists
            if not os.path.exists(output_path):
                os.makedirs(output_path)  # Uncommented this line
                print("Directory created")

            return_value = cv2.imwrite(output_file, frame)
            print("Return value:", return_value)
            cv2.imshow("Frame", frame)
            cv2.waitKey(1)  # Add a small delay to allow the window to update
            
            time.sleep(0.01)
    finally:
        cap.release()
        

if __name__ == "__main__":
    generate_images()