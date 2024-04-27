import threading
import cv2
import numpy as np
from sklearn.linear_model import LinearRegression
from math import tan, radians, cos, atan, degrees
import time
import matplotlib.pyplot as plt
import mplcursors
import threading
from flask import Flask, render_template, Response, jsonify

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
        self.lower_green = np.array([53, 100, 40])
        self.upper_green = np.array([93, 220, 150])

        self.lower_red1 = np.array([0, 160, 120])
        self.upper_red1 = np.array([5, 220, 200])

        self.lower_red2 = np.array([173, 160, 100])
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

                # Flatten the array
                x_flat = x.flatten()
                
                # Calculate the start and end points of the line
                x1 = int(min(x_flat))
                y1 = int(slope * x1 + intercept)
                x2 = int(max(x_flat))
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
                avg_ycoord_bottom = 0
                
                if len(lines) > 1:
                    avg_ycoord_1 = (lines[0][0][1] + lines[0][1][1]) / 2
                    avg_ycoord_2 = (lines[1][0][1] + lines[1][1][1]) / 2
                    
                    if avg_ycoord_1 > avg_ycoord_2:
                        avg_ycoord_bottom = avg_ycoord_1
                    else:
                        avg_ycoord_bottom = avg_ycoord_2
                        
                else:
                    avg_ycoord_bottom = (lines[0][0][1] + lines[0][1][1]) / 2

                self.real_distance = 0
                if avg_ycoord_bottom != 0:
                    # Calculate the distance to the boundary in the image plane
                    image_distance = (known_height_image * self.focal_length) / avg_ycoord_bottom

                    # Adjust for the camera angle
                    self.real_distance = image_distance * self.distance_multiplier
                    self.real_distance = avg_ycoord_bottom / 100
                    cv2.putText(binary, f"{round(self.real_distance * 100, 3)} cm", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (125, 125, 125), 4)
                    
                    break
                    
                elif avg_ycoord_bottom == 0:
                    self.real_distance = 0 
                    
        except:
            self.real_distance = 0
            
        self.real_distance = self.real_distance * 100
                    
        if self.real_distance != 0:
            if self.real_distance > 0 and self.real_distance < 350:
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
        
        frame = frame[100:, :]
        
        frameraw = frameraw[100:500, 300:980]
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
        
        
    def process_blocks(self):
        self.video_writer = None
        self.blockPositions = {}

        video_path = r"C:\Users\felix\Downloads\Videos Runs\alles Licht, Fenster auf, keine Blöcke.mp4"
        self.cap = cv2.VideoCapture(video_path)

        self.avg_edge_distance_values = []

        freeze = False  # Variable to control whether the video and plotting are frozen
        show_binary = False  # Variable to control whether the binary or normal frame is shown

        def plot_data():
            plt.ion()  # Turn on interactive mode
            fig, ax = plt.subplots(figsize=(10, 5))  # Create a figure and an axes with specified size
            line, = ax.plot([], [])  # Initialize a line object
            cursor = mplcursors.cursor(line, hover=True)  # Enable the cursor

            while True:
                if not freeze:  # Only update the plot if the video and plotting are not frozen
                    # Append the avg_edge_distance to the list
                    self.avg_edge_distance_values.append(self.avg_edge_distance)

                    # Update the plot
                    ax.clear()  # Clear the axes
                    line.set_ydata(self.avg_edge_distance_values)  # Update the y-data of the line
                    line.set_xdata(range(len(self.avg_edge_distance_values)))  # Update the x-data of the line
                    ax.add_line(line)  # Add the line to the axes
                    ax.set_ylim([0, 300])  # Set the limits of the y-axis
                    ax.relim()  # Recompute the data limits
                    ax.autoscale_view()  # Rescale the view

                    cursor.connect("add", lambda sel: sel.annotation.set_text(
                        'Point {}, Y={}'.format(sel.index, sel.target[1])
                    ))  # Update the annotation for each data point

                    plt.draw()  # Redraw the figure
                    plt.pause(0.01)

        threading.Thread(target=plot_data).start()

        cv2.namedWindow("Video")
        cv2.setMouseCallback("Video", self.mouse_event)

        try:
            while True:
                key = cv2.waitKey(1) & 0xFF  # Get the last key pressed
                if key == ord('f'):  # If the last key pressed was 'f', toggle the freeze variable
                    freeze = not freeze
                elif key == ord('s'):  # If the last key pressed was 's', toggle the show_binary variable
                    show_binary = not show_binary

                if not freeze:  # Only update the video if the video and plotting are not frozen
                    StartTime = time.time()
                    self.block_array, frameraw, frame = self.get_coordinates()
                    
                    self.calculate_block_positions()
                    
                    time.sleep(0.04)

                    StopTime = time.time()
                    framebinary = self.get_edges(frameraw)

                    self.frame = framebinary if show_binary else frame  # Show the binary frame if show_binary is True, otherwise show the normal frame
                    cv2.imshow("Video", self.frame)

        finally:
            self.cap.release()
            cv2.destroyAllWindows()

    def mouse_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
            h, s, v = hsv[y, x]
        
            # Create a small color sample
            color_sample = np.zeros((200, 300, 3), dtype=np.uint8)
            color_sample[:, :] = self.frame[y, x]
        
            # Write the HSV values on the color sample
            cv2.putText(color_sample, f"HSV: ({h}, {s}, {v})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
            # Display the color sample
            cv2.imshow("Color Sample", color_sample)
            
    def calculate_block_positions(self):
        #Variables
        TIMEOUT = 0
        TIMEOUTBlock = 0.5
        corners = 0
        rounds = 0
        Sensor = 2
        direction = 1
        relative_angle = 0
        oldAngle = 0
        detect_new_block = True
        Last_Esp_Command = 0
        desired_distance_wall = 50
        timelastcorner = time.time()
        Inverted = False
        
        old_desired_distance_wall = 50
        
        coordinates_self = (640, 720) #x, y
        middledistance = 50
        GyroCornerAngle = 70

        FreezeSize = 150
        FreezeY = 330
        FreezeBlock = False
        KP = 0.22
        desired_distance_to_block_red = -700
        desired_distance_to_block_green = 700
        
        SensorFirstCornerChanged = False
        active_block_drive = False
        ESP_adjusted = False
        ESP_adjusted_back = True
        Special_red_case_adjusted_back = False
        SIZE = 0
        
        block_array = self.block_array.copy()
        if len(block_array) > 0:
            #Delete blocks not meeting requirements
            for block in block_array:
                if block["size"] > SIZE:
                    pass
                else:
                    block_array.remove(block)
                
        #print(block_array)
        if len(block_array) > 0 or active_block_drive:
            #Utils.LogDebug(f"Lenght of block_array: {len(block_array)}")
            #Sort blocks by size
            
            if not active_block_drive:
                block_array.sort(key=lambda x: x['size'], reverse=True)

                nextBlock = block_array[0]
                
            nextBlock["distance"] = self.get_distance_to_block(nextBlock)
            
            #print(nextBlock["distance"])
            block_distance_to_wall = self.avg_edge_distance - nextBlock['distance']
            if (self.avg_edge_distance < 200) and not active_block_drive and -10 < relative_angle < 40 and direction == 1:
                print(f"avg_edge_distance: {self.avg_edge_distance}, distance: {nextBlock['distance']}, block_distance_to_wall: {block_distance_to_wall}, nextblock['x']: {nextBlock['x']}, nextblock['y']: {nextBlock['y']}")
                #block_distance_to_wall = self.avg_edge_distance - nextBlock['distance']
                #Utils.LogInfo(f"avg_edge_distance: {self.avg_edge_distance}, distance: {nextBlock['distance']}, block_distance_to_wall: {block_distance_to_wall}, nextblock['x']: {nextBlock['x']}, nextblock['y']: {nextBlock['y']}")
                if (120 < self.avg_edge_distance < 180) and nextBlock['x'] < 300 and 70 < nextBlock["distance"] < 110:
                    nextBlock['position'] = "1"
                    BlockPos = corners + 1 if corners < 3 else 0
                elif 80 < block_distance_to_wall < 130 and nextBlock["distance"] < 80:
                    nextBlock['position'] = "3"
                    #print("3")
                    BlockPos = corners
                elif 30 < block_distance_to_wall < 60 and nextBlock["distance"] < 80: #or block_distance_to_wall < 80:
                    nextBlock['position'] = "2"
                    if abs(relative_angle) > 30:
                        BlockPos = corners + 1 if corners < 3 else 0
                    else:
                        BlockPos = corners

                else:
                    nextBlock['position'] = "0"
                
                if nextBlock['position'] != "0" and nextBlock["position"] != "3":
                    active_block_drive = True
                    self.blockPositions.update({BlockPos: {"position": nextBlock['position'], "color": nextBlock['color']}})
                
                elif nextBlock['position'] != "0" and nextBlock["position"] == "3":# and BlockPos not in self.blockPositions:
                    self.blockPositions.update({BlockPos: {"position": nextBlock['position'], "color": nextBlock['color']}})
                        
            elif (self.avg_edge_distance < 200) and not active_block_drive and - 40 < relative_angle < 10 and direction == 0:
                #Utils.LogInfo(f"avg_edge_distance: {self.avg_edge_distance}, distance: {nextBlock['distance']}, block_distance_to_wall: {block_distance_to_wall}, nextblock['x']: {nextBlock['x']}, nextblock['y']: {nextBlock['y']}")
                if (120 < self.avg_edge_distance < 180) and nextBlock['x'] > 780 and nextBlock['y'] > 200 and 70 < nextBlock["distance"] < 110:
                    nextBlock['position'] = "1"
                    BlockPos = corners + 1 if corners < 3 else 0
                elif 80 < block_distance_to_wall < 130 and nextBlock["distance"] < 80:
                    nextBlock['position'] = "3"
                    BlockPos = corners
                elif 130 < block_distance_to_wall < 180 and nextBlock["distance"] < 80:# or block_distance_to_wall < 80:
                    nextBlock['position'] = "2"
                    if abs(relative_angle) > 40:
                        BlockPos = corners + 1 if corners < 3 else 0
                    else:
                        BlockPos = corners
                else:
                    nextBlock['position'] = "0"
                
                if nextBlock['position'] != "0" and nextBlock["position"] != "3":
                    active_block_drive = True
                    self.blockPositions.update({BlockPos: {"position": nextBlock['position'], "color": nextBlock['color']}})
                
                elif nextBlock['position'] != "0" and nextBlock["position"] == "3":# and BlockPos not in self.blockPositions:
                    self.blockPositions.update({BlockPos: {"position": nextBlock['position'], "color": nextBlock['color']}})
            
            
if __name__ == '__main__':
    camera = Camera()
    
    print("Starting...")
    app = Flask(__name__)
    
    @app.route('/')
    def positions():
        return render_template('positions.html')
    
    @app.route('/positions_feed')
    def positions_feed():
        return jsonify(camera.blockPositions)
    
    # Run the server in a separate thread
    server_thread = threading.Thread(target=app.run, kwargs={'host':'0.0.0.0', 'threaded':True})
    server_thread.start()

    camera.process_blocks()