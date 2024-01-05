from flask import Flask, render_template, Response
import cv2
import numpy as np
import threading

class Camera():
    def __init__(self, video_stream=False, video_source=0):
        self.frame = None
        self.frame_lock = threading.Lock()
        
        self.video_stream = video_stream
        
        self.cap = cv2.VideoCapture(video_source)
        
        # Define the color ranges for green and red
        self.lower_green = np.array([35, 100, 100])
        self.upper_green = np.array([85, 255, 255])

        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])

        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])

        self.kernel = np.ones((5, 5), np.uint8)

        
    def get_coordinates(self):
        rval, frame = self.cap.read()
        if rval:  # Only process the frame if it was read correctly
            # Convert the image from BGR to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Create a mask of pixels within the green color range
            mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)

            # Create a mask of pixels within the red color range
            mask_red1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            mask_red2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)

            # Dilate the masks to merge nearby areas
            mask_green = cv2.dilate(mask_green, self.kernel, iterations=1)
            mask_red = cv2.dilate(mask_red, self.kernel, iterations=1)

            # Find contours in the green mask
            contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Find contours in the red mask
            contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            block_array = []

            # Process each green contour
            for contour in contours_green:
                x, y, w, h = cv2.boundingRect(contour)
                if w > 50 and h > 50:  # Only consider boxes larger than 50x50
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.putText(frame, 'Green Object', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)
                    block_array.append({'color': 'green', 'x': x, 'y': y, 'w': w, 'h': h})

            # Process each red contour
            for contour in contours_red:
                x, y, w, h = cv2.boundingRect(contour)
                if w > 50 and h > 50:  # Only consider boxes larger than 50x50
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
                    cv2.putText(frame, 'Red Object', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,255), 2)
                    block_array.append({'color': 'red', 'x': x, 'y': y, 'w': w, 'h': h})
                    
            return block_array, frame
        
        return None, None
        

    def process_blocks(self):
        while True:
            block_array, self.frame = self.get_coordinates()
            
            if block_array is not None:
                # Print the block array
                for block in block_array:
                    print(block)
                    
    def start_processing(self):
        thread = threading.Thread(target=self.process_blocks)
        thread.daemon = False
        thread.start()
        
        self.flask_server()
        
    def video_frames(self):
        if self.video_stream:
            while True:
                with self.frame_lock:
                    if self.frame is not None:
                        (flag, encodedImage) = cv2.imencode(".jpg", self.frame)
                        yield (b'--frame\r\n'
                                b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage) + b'\r\n')
                    else:
                        yield (b'--frame\r\n'
                                b'Content-Type: image/jpeg\r\n\r\n' + b'\r\n')
                        
    def flask_server(self):
        if self.video_stream:
            app = Flask(__name__)
            
            @app.route('/')
            def index():
                """Video streaming home page."""
                return render_template('index.html')


            @app.route('/video_feed')
            def video_feed():
                """Video streaming route. Put this in the src attribute of an img tag."""
                return Response(Cam.video_frames(),
                                mimetype='multipart/x-mixed-replace; boundary=frame')

            if __name__ == '__main__':
                app.run(host='0.0.0.0', threaded=True)
                
        

Cam = Camera(video_stream=False)
Cam.start_processing()
