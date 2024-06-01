import threading
import cv2
import numpy as np
from sklearn.linear_model import LinearRegression
from math import tan, radians, cos, atan, degrees, atan2
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
        self.kernel = np.ones((7, 7), np.uint8)
        self.desired_distance_wall = -1
        self.block_distance = -1
        
        self.edge_distances = []
        self.avg_edge_distance = 0
        
        self.focal_length = 373.8461538461538
        self.known_height = 0.1
        self.camera_angle = 15
        self.distance_multiplier = 2.22
    
    def get_drive_direction(self, frame):
        frame = frame[100:, :]
        
        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        gray = cv2.dilate(gray, self.kernel, iterations=1)
    
        # Threshold the grayscale image to get a binary image
        binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 1001, 30)
        
        binary = cv2.dilate(binary, self.kernel, iterations=2)
        
        edges = cv2.Canny(binary, 20, 30, apertureSize=7)
        
        # Perform Probabilistic Hough Line Transform
        lines = cv2.HoughLinesP(edges, 4, np.pi/180, 100, minLineLength=50, maxLineGap=30)
        
        for line in lines:
            cv2.line(frame, (line[0][0], line[0][1]), (line[0][2], line[0][3]), (0, 255, 0), 2)
        
        min_angle = 90  # Initialize minimum angle to 90 degrees
        vertical_line = None  # Initialize the vertical line

        for line in lines:
            for x1, y1, x2, y2 in line:
                # Calculate the angle of the line with respect to the vertical axis
                angle = degrees(atan2(y2 - y1, x2 - x1))  # Swap x and y
                angle = abs(angle - 90)  # Adjust the range to [0, 180]

                # If this line is more vertical than the previous most vertical line
                if angle < min_angle:
                    min_angle = angle
                    vertical_line = line

        if vertical_line is not None:
            x1, y1, x2, y2 = vertical_line[0]
            average_x = (x1 + x2) / 2

            cv2.line(frame, (x1, y1), (x2, y2), (244, 255, 0), 2)

            if average_x < 640:
                return 1
            else:
                return 0

        return -1
        
        
    def get_edges(self, frame):
        frame = frame[100:, :]
        
        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        gray = cv2.dilate(gray, self.kernel, iterations=1)

        # Threshold the grayscale image to get a binary image
        binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 1001, 30)
        
        binary = cv2.dilate(binary, self.kernel, iterations=2)
        
        # Get the average pixel value of the left and right side of the image
        left_avg = np.average(binary[:, :binary.shape[1]//2])
        right_avg = np.average(binary[:, binary.shape[1]//2:])
        
        self.avg_brightness_values_left.append(left_avg)  # Append the average brightness to the list
        self.avg_brightness_values_right.append(right_avg)  # Append the average brightness to the list

        # Perform Canny edge detection
        edges = cv2.Canny(binary, 50, 120, apertureSize=3)
        
        # Draw the edges onto the binary image
        # binary = cv2.bitwise_and(binary, binary, mask=edges)

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
        intercept_threshold = 10

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
                
            lines_by_angle = {abs(k): v for k, v in sorted(lines_by_angle.items(), key=lambda item: abs(item[0]))}
            
            lines = lines_by_angle[list(lines_by_angle.keys())[0]]
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
                self.real_distance =  7193 * (avg_ycoord_bottom ** -0.917)
                
                cv2.putText(binary, f"{round(self.real_distance, 3)} cm", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (125, 125, 125), 4)
                
            elif avg_ycoord_bottom == 0:
                self.real_distance = 0 
                    
        except Exception as e:
            self.real_distance = 0
                    
        if self.real_distance != 0:
            if self.real_distance > 50 and self.real_distance < 220:
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
        
        # frame = frame[100:, :]
        
        #frameraw = frameraw[100:500, 300:980]
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

        video_path = r"C:\Users\felix\Downloads\Videos Runs\alles Licht, Fenster zu, keine Blöcke, keine Filter.mp4"
        self.cap = cv2.VideoCapture(video_path)

        self.avg_edge_distance_values = []
        self.distance_next_block = -1
        self.avg_brightness_values_left = []  # Initialize the list for average brightness values
        self.avg_brightness_values_right = []  # Initialize the list for average brightness values

        freeze = False  # Variable to control whether the video and plotting are frozen
        show_binary = False  # Variable to control whether the binary or normal frame is shown

        def plot_data():
            plt.ion()  # Turn on interactive mode
            fig, ax = plt.subplots(figsize=(10, 5))  # Create a figure and an axes with specified size
            line1, = ax.plot([], [], label='avg_edge_distance')  # Initialize a line object for avg_edge_distance
            line2, = ax.plot([], [], label='distance_next_block')  # Initialize a line object for distance_next_block
            line3, = ax.plot([], [], label='avg_brightness_left')  # Initialize a line object for avg_brightness
            line4, = ax.plot([], [], label='avg_brightness_right')  # Initialize a line object for avg_brightness
            cursor = mplcursors.cursor([line1, line2], hover=True)  # Enable the cursor for both lines
        
            self.distance_next_block_values = []  # Initialize the list for distance_next_block values
        
            while True:
                if not freeze:  # Only update the plot if the video and plotting are not frozen
                    # Append the avg_edge_distance and distance_next_block to their respective lists
                    self.avg_edge_distance_values.append(self.avg_edge_distance)
                    self.distance_next_block_values.append(self.distance_next_block)
        
                    # Update the plot
                    ax.clear()  # Clear the axes
                    line1.set_ydata(self.avg_edge_distance_values)  # Update the y-data of the line for avg_edge_distance
                    line1.set_xdata(range(len(self.avg_edge_distance_values)))  # Update the x-data of the line for avg_edge_distance
                    
                    line2.set_ydata(self.distance_next_block_values)  # Update the y-data of the line for distance_next_block
                    line2.set_xdata(range(len(self.distance_next_block_values)))  # Update the x-data of the line for distance_next_block
                    
                    line3.set_ydata(self.avg_brightness_values_left)  # Update the y-data of the line for avg_brightness
                    line3.set_xdata(range(len(self.avg_brightness_values_left)))  # Update the x-data of the line for avg_brightness
                    
                    line4.set_ydata(self.avg_brightness_values_right)  # Update the y-data of the line for avg_brightness
                    line4.set_xdata(range(len(self.avg_brightness_values_right)))  # Update the x-data of the line for avg_brightness
                    
                    ax.add_line(line1)  # Add the line for avg_edge_distance to the axes
                    ax.add_line(line2)  # Add the line for distance_next_block to the axes
                    ax.add_line(line3)  # Add the line for avg_brightness to the axes
                    ax.add_line(line4)  # Add the line for avg_brightness to the axes
                    
                    ax.set_ylim([0, 300])  # Set the limits of the y-axis
                    ax.relim()  # Recompute the data limits
                    ax.autoscale_view()  # Rescale the view
                    ax.legend()  # Add a legend
        
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
                    
                    time.sleep(0.06)

                    StopTime = time.time()
                    framebinary = self.get_edges(frameraw)
                    print(self.get_drive_direction(frameraw))

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
            
            self.distance_next_block = nextBlock["distance"]
            
            #print(nextBlock["distance"])
            block_distance_to_wall = self.avg_edge_distance - nextBlock['distance']
            if (self.avg_edge_distance < 200) and not active_block_drive and -10 < relative_angle < 40 and direction == 1:
                print(f"avg_edge_distance: {self.avg_edge_distance}, distance: {nextBlock['distance']}, block_distance_to_wall: {block_distance_to_wall}, nextblock['x']: {nextBlock['x']}, nextblock['y']: {nextBlock['y']}")
                #block_distance_to_wall = self.avg_edge_distance - nextBlock['distance']
                #Utils.LogInfo(f"avg_edge_distance: {self.avg_edge_distance}, distance: {nextBlock['distance']}, block_distance_to_wall: {block_distance_to_wall}, nextblock['x']: {nextBlock['x']}, nextblock['y']: {nextBlock['y']}")
                if (120 < self.avg_edge_distance < 180) and nextBlock['x'] < 300 and 45 < nextBlock["distance"] < 110:
                    nextBlock['position'] = "1"
                    BlockPos = corners + 1 if corners < 3 else 0
                elif 80 < block_distance_to_wall < 130 and 40 < nextBlock["distance"] < 80:
                    nextBlock['position'] = "3"
                    #print("3")
                    BlockPos = corners
                elif 30 < block_distance_to_wall < 80 and 40 < nextBlock["distance"] < 80: #or block_distance_to_wall < 80:
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
                    
        else:
            self.distance_next_block = -1
            
            
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