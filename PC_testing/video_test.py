import threading
import cv2
import numpy as np
from sklearn.linear_model import LinearRegression
from math import tan, radians, cos, atan, degrees
import time

#A class for detecting red and green blocks in the camera stream           
class Camera():
    def __init__(self, video_stream=False, video_source=0):
        # Variable initialization
        self.freeze = False
        self.frame = None
        # self.frame_lock = threading.Lock()
        # self.video_stream = video_stream
        # self.picam2 = Picamera2()
        
        # # Configure and start the camera
        # config = self.picam2.create_still_configuration(main={"size": (1280, 720)}, raw={"size": (1280, 720)}, controls={"FrameRate": 34})
        # self.picam2.configure(config)
        # self.picam2.start()
        # self.picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
        
        # Define the color ranges for green and red in HSV color space
        self.lower_green = np.array([53, 130, 40])
        self.upper_green = np.array([93, 220, 150])

        self.lower_red1 = np.array([0, 160, 120])
        self.upper_red1 = np.array([5, 220, 200])

        self.lower_red2 = np.array([173, 160, 120])
        self.upper_red2 = np.array([180, 220, 200])

        # Define the kernel for morphological operations
        self.kernel = np.ones((5, 5), np.uint8)
        self.desired_distance_wall = -1
        self.block_distance = -1
        
        self.edge_distances = []
        self.avg_edge_distance = 0
        
        self.focal_length = 373.8461538461538
        self.known_height = 0.1
        self.camera_angle = 15
        self.distance_multiplier = 2.22
        
        
    def get_edges(self, frame):
        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Threshold the grayscale image to get a binary image
        _, binary = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY)
        
        binary = cv2.dilate(binary, self.kernel, iterations=1)

       # Perform Canny edge detection
        edges = cv2.Canny(binary, 50, 120, apertureSize=3)

        # Perform Probabilistic Hough Line Transform
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=100, maxLineGap=30)

        # Initialize an empty list to store the groups of lines
        line_groups = []

        # Define a function to calculate the slope and intercept of a line
        def get_slope_intercept(line):
            x1, y1, x2, y2 = line[0]
            try:
                if x2 - x1 != 0:
                    slope = (y2 - y1) / (x2 - x1)
                else:
                    slope = float('inf')
            except:
                slope = 0
                
            intercept = y1 - slope * x1
            return slope, intercept

        # Define a threshold for the difference in slopes and intercepts
        slope_threshold = 0.1
        intercept_threshold = 10.0

        # Group the lines
        try:
            for line in lines:
                slope1, intercept1 = get_slope_intercept(line)
                for group in line_groups:
                    slope2, intercept2 = get_slope_intercept(group[0])
                    if abs(slope1 - slope2) < slope_threshold and abs(intercept1 - intercept2) < intercept_threshold:
                        group.append(line)
                        break
                else:
                    line_groups.append([line])

            # Initialize a Linear Regression model
            model = LinearRegression()

            # Initialize a dictionary to store the lines grouped by their angles
            lines_by_angle = {}

            for group in line_groups:
                # Prepare the data for Linear Regression
                x = np.array([val for line in group for val in line[0][::2]]).reshape(-1, 1)
                y = np.array([val for line in group for val in line[0][1::2]])

                # Fit the model to the data
                model.fit(x, y)

                # Get the slope and intercept of the line
                slope = model.coef_[0]
                intercept = model.intercept_

                # Calculate the start and end points of the line
                x1 = int(min(x))
                y1 = int(slope * x1 + intercept)
                x2 = int(max(x))
                y2 = int(slope * x2 + intercept)

                # Calculate the angle of the line
                try:
                    angle = atan((y2 - y1) / (x2 - x1))
                except:
                    angle = 0
                    
                angle = degrees(angle)

                # Group the lines by their angles with a 5 degree tolerance
                grouped = False
                for existing_angle in lines_by_angle.keys():
                    if abs(angle - existing_angle) <= 5:
                        lines_by_angle[existing_angle].append(((x1, y1), (x2, y2)))
                        grouped = True
                        break
                if not grouped:
                    lines_by_angle[angle] = [((x1, y1), (x2, y2))]

                # Draw the line
                cv2.line(binary, (x1, y1), (x2, y2), (125, 125, 125), 4)
            
            known_height_image = self.known_height * cos(radians(self.camera_angle))

            for angle, lines in lines_by_angle.items():
                apparent_height = 0
                if len(lines) > 1:
                    num_points = 30
                    # Calculate the x and y increments for each point along the lines
                    y_increment_left = (lines[1][0][1] - lines[0][0][1]) / num_points
                    y_increment_right = (lines[1][1][1] - lines[0][1][1]) / num_points

                    distances = []
                    # Loop through each point along the lines
                    for i in range(num_points):
                        # Calculate the y coordinates of the points on the left and right lines
                        y_left = lines[0][0][1] + i * y_increment_left
                        y_right = lines[1][0][1] + i * y_increment_right

                        # Calculate the y distance between the points on the left and right lines
                        distance = abs(y_left - y_right)

                        # Add the distance to the list
                        distances.append(distance)

                    # Return the average of the distances
                    apparent_height = np.mean(distances)

                self.real_distance = 0
                if apparent_height != 0:
                    # Calculate the distance to the boundary in the image plane
                    image_distance = (known_height_image * self.focal_length) / apparent_height

                    # Adjust for the camera angle
                    self.real_distance = image_distance * self.distance_multiplier
                    cv2.putText(binary, f"{round(self.real_distance * 100, 3)} cm", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (125, 125, 125), 4)
                    break
                    
                elif apparent_height == 0:
                    self.real_distance = 0
                    
        except:
            self.real_distance = 0
            
        self.real_distance = self.real_distance * 100
                    
        if self.real_distance != 0:
            if self.real_distance > 50 and self.real_distance < 350:
                #print(self.edge_distances)
                if len(self.edge_distances) > 5:
                    self.edge_distances.pop(0)
                    
                    
                self.edge_distances.append(round(self.real_distance, 3))
                self.avg_edge_distance = np.mean(self.edge_distances)
                #print(self.avg_edge_distance)
                
        
        return binary
    
    
    #Get the coordinates of the blocks in the camera stream
    def get_coordinates(self):
        #frameraw = self.picam2.capture_array()
        ret, frameraw = self.cap.read()
        
        #frameraw = cv2.cvtColor(frameraw, cv2.COLOR_RGB2BGR)
        frame = frameraw.copy()
        
        #frameraw = frameraw[150:, :]
        
        
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
        
        #print(f"middle hsv: {hsv[0, 360]}, inverted: {hsv[300, 1000]}")
        
        cv2.circle(frame, (640, 720), 10, (255, 0, 0), -1)
        cv2.putText(frame, f"{self.desired_distance_wall}", (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 4)
        cv2.putText(frame, f"Freeze: {self.freeze}", (100, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 4)
        cv2.putText(frame, f"Distance: {self.block_distance}", (700, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 4)
        #cv2.circle(frame, (1000, 300), 10, (255, 0, 0), -1)
        
        block_array = []

        # Process each green contour
        for contour in contours_green:
            x, y, w, h = cv2.boundingRect(contour)
            if w > 20 and h > 50:  # Only consider boxes larger than 50x50
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(frame, 'Green Object', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)
                block_array.append({'color': 'green', 'x': x, 'y': y, 'w': w, 'h': h, 'mx': x+w/2, 'my': y+h/2, 'size': w*h, 'distance': self.get_distance_to_block({'x': x, 'y': y, 'w': w, 'h': h})})
                cv2.line(frame, (640, 720), (int(x+w/2), int(y+h/2)), (0, 255, 0), 2)

        # Process each red contour
        for contour in contours_red:
            x, y, w, h = cv2.boundingRect(contour)
            if w > 20 and h > 50:  # Only consider boxes larger than 50x50
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
                cv2.putText(frame, 'Red Object', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,255), 2)
                block_array.append({'color': 'red', 'x': x, 'y': y, 'w': w, 'h': h, 'mx': x+w/2, 'my': y+h/2, 'size': w*h, 'distance': self.get_distance_to_block({'x': x, 'y': y, 'w': w, 'h': h})})
                cv2.line(frame, (640, 720), (int(x+w/2), int(y+h/2)), (0, 0, 255), 2)

        return block_array, frameraw, frame
    
    
    def get_distance_to_block(self, block):
        # Calculate the distance to the block
        image_distance = (self.focal_length * self.known_height * cos(radians(self.camera_angle))) / block['h']
        self.real_distance = image_distance * self.distance_multiplier
        
        self.block_distance = self.real_distance * 100
        return self.real_distance * 100
        
        
    #Functrion running in a new thread that constantly updates the coordinates of the blocks in the camera stream
    def process_blocks(self):
        self.video_writer = None
        
        video_path = r"C:\Users\felix\Downloads\Videos Runs\alles Licht, fenster zu, Blöcke, kein Filter.mp4"
        self.cap = cv2.VideoCapture(video_path)

        try:
            while True:
                StartTime = time.time()
                self.block_array, frameraw, frame = self.get_coordinates()
                time.sleep(0.08)

                StopTime = time.time()
                #print(f"Time needed: {StopTime - StartTime}")
                framebinary = self.get_edges(frameraw)
                
                # Display the frame
                cv2.imshow("Video", framebinary)

                # Wait for a specified amount of time (in milliseconds) before displaying the next frame
                cv2.waitKey(1)  # Change the argument to adjust the delay between frames

                # if self.video_writer is None:
                #     # Create a VideoWriter object to save the frames as an mp4 file
                #     fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                #     self.video_writer = cv2.VideoWriter('output.mp4', fourcc, 20, (frameraw.shape[1], frameraw.shape[0]), True)

                # # Write the frame to the video file
                # self.video_writer.write(frameraw)
        
        finally:
            self.cap.release()
            cv2.destroyAllWindows()
            
if __name__ == '__main__':
    camera = Camera()
    camera.process_blocks()