import cv2
import os
import time
from tkinter import Tk, Label, Button, StringVar, messagebox, ttk
from tkinter.filedialog import askopenfilename, askdirectory

global file_path, output_path, progress
file_path = ""
output_path = ""
progress = None

def main():
    global progress, status_text
    root = Tk()
    status_text = StringVar()
    Label(root, textvariable=status_text).pack()
    Button(root, text="Select Input File", command=select_input_file).pack()
    Button(root, text="Select Output Directory", command=select_output_directory).pack()
    Button(root, text="Start", command=generate_images).pack()
    progress = ttk.Progressbar(root, length=100, mode='determinate')
    progress.pack()
    root.mainloop()

def select_input_file():
    global file_path
    file_path = askopenfilename()
    status_text.set("Input file selected")

def select_output_directory():
    global output_path
    output_path = askdirectory()
    status_text.set("Output directory selected")

def generate_images():
    global status_text, progress
    if not file_path or not output_path:
        messagebox.showerror("Error", "Please select both input file and output directory")
        return

    # Load the video
    cap = cv2.VideoCapture(file_path)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    progress['maximum'] = total_frames

    time.sleep(1)

    try:
        while cap.isOpened():
            try:
                ret, frameraw = cap.read()
                if not ret:
                    break
            except Exception as e:
                print(e)
                break

            frame = frameraw.copy()

            # Save the frame
            frame_number = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
            output_file = os.path.join(output_path, f"frame{frame_number}.jpg")

            # Check if directory exists
            if not os.path.exists(output_path):
                os.makedirs(output_path)
                status_text.set("Directory created")

            return_value = cv2.imwrite(output_file, frame)
            status_text.set(f"Frame {frame_number} saved, return value: {return_value}")

            progress['value'] = frame_number
            progress.update()

            # cv2.waitKey(1)  # Add a small delay to allow the window to update

            time.sleep(0.001)

            # Stop the loop when the last frame is reached
            if frame_number == total_frames:
                break
    finally:
        cap.release()
        status_text.set("Image generation completed")


if __name__ == "__main__":
    main()