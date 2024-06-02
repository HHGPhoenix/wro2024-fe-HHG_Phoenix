import time
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import sh1106
from gpiozero import CPUTemperature
import psutil
import busio
import board, adafruit_tcs34725, adafruit_mpu6050
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn



#A class for writing to a OLED Display
class DisplayOled():
    def __init__(self, Utils):
        # serial = i2c(port=0, address=0x3C)
        # self.device = sh1106(serial)
        
        #Variable init
        self.first_line = ""
        self.second_line = ""
        self.threadStop = 0
        self.Utils = Utils
        
        # Wake the screen by drawing an outline
        # with canvas(self.device) as draw:
            # draw.rectangle(self.device.bounding_box, outline="white", fill="black")
    
       
    #Clear the Display 
    def clear(self):
        with canvas(self.device) as draw:
            draw.rectangle(self.device.bounding_box, outline="white", fill="black")
            
    
    #Write lines in variables so they get written by the update function   
    def write(self, first_line="", second_line="", reset=False, xCoord=0, yCoord=17):
        if reset:
            self.first_line = ""
            self.second_line = ""
        
        if first_line != "":
            self.first_line = first_line
        if second_line != "":
            self.second_line = second_line
        

    #Update the Display
    def update(self):
        while self.threadStop == 0:
            #Get CPU temperature, CPU usage, RAM usage and Disk usage
            cpuTemp = CPUTemperature()
            self.cpu_usage = psutil.cpu_percent(interval=0)
            self.ram = psutil.virtual_memory()
            self.disk = psutil.disk_usage('/')
            
            #Format them to always have the same number of decimal points
            cpu_temp_formatted = self.Utils.convert_to_decimal_points(cpuTemp.temperature, 1)
            cpu_usage_formatted = self.Utils.convert_to_decimal_points(self.cpu_usage, 1)
            ram_usage_formatted = self.Utils.convert_to_decimal_points(self.ram.percent, 1)
            disk_usage_formatted = self.Utils.convert_to_decimal_points(self.disk.percent, 1)
            voltage_value_formatted = self.Utils.convert_to_decimal_points(self.Utils.ADC.voltage, 2)
            
            #Draw all the data on the Display
            with canvas(self.device) as draw:
                    #top
                    draw.text((0, 0), f"{cpu_temp_formatted}°C", fill="white", align="left")
                    draw.text((40, 0), f"DISK:{(int(float(disk_usage_formatted)))}%", fill="white")
                    draw.text((92, 0), f"{voltage_value_formatted}V", fill="white")
                    
                    #bottom
                    draw.text((0, 50), f"CPU:{cpu_usage_formatted}%", fill="white")
                    draw.text((75, 50), f"RAM:{ram_usage_formatted}%", fill="white")
                    
                    #custom
                    draw.multiline_text((0, 15), f"{self.first_line}\n{self.second_line}", fill="white", align="center", anchor="ma")
                    
            time.sleep(2)


#A class for reading a MPU6050 Gyroscope
class Gyroscope():
    def __init__(self):
        i2c = busio.I2C(board.D1, board.D0)
        self.sensor = adafruit_mpu6050.MPU6050(i2c)
        self.sensor._gyro_range = 1
        
        self.threadStop = 0
        self.GyroStart = False

        #Initialize variables for storing the angle and time
        self.angle = 0.0  # Initial angle
        self.last_time = time.time()
    

    #Read the gyroscope data and calculate the angle
    def get_angle(self):
        while self.threadStop == 0:
            if self.GyroStart == True:
                time.sleep(0.005)
                offset_x = 0.29
                offset_y = 0.0
                offset_z = 0.0
                #Read gyroscope data
                gyro_data = self.sensor.gyro
                gyro_data = [gyro_data[0] + offset_x, gyro_data[1] + offset_y, gyro_data[2] + offset_z]

                # Get the current time
                current_time = time.time()

                # Calculate the time elapsed since the last measurement
                delta_time = current_time - self.last_time
                
                #bugfix for time-jumps
                if delta_time >= 0.5:
                    delta_time = 0.003

                # Integrate the gyroscope readings to get the change in angle
                if gyro_data[0] < 0.02 and gyro_data[0] > -0.02:
                    gyro_data = 0
                else:
                    gyro_data = gyro_data[0]
                    
                delta_angle = gyro_data * delta_time * 60

                # Update the angle
                self.angle += delta_angle

                # Update the last time for the next iteration
                self.last_time = current_time
                
            else:
                self.angle = 0.0
                self.last_time = time.time()
                time.sleep(0.1)
   
   
#A class for reading a ADS1015 ADC        
class AnalogDigitalConverter():
    def __init__(self, channel=3):
        #Variables
        self.threadStop = 0
        self.channel = channel
        self.voltage = 12
        
        #ADC init
        i2c = busio.I2C(board.D1, board.D0)
        self.ads = ADS.ADS1015(i2c)
        self.ads.active = True
        
        #Channel init
        if channel == 0:
            self.chan = AnalogIn(self.ads, ADS.P0)
        elif channel == 1:
            self.chan = AnalogIn(self.ads, ADS.P1)
        elif channel == 2:
            self.chan = AnalogIn(self.ads, ADS.P2)
        elif channel == 3:
            self.chan = AnalogIn(self.ads, ADS.P3)
        else:
            raise Exception(f"No valid ADC channel specified: {channel}")
       
    
    #Read the voltage from the ADC    
    def read(self):
        while self.threadStop == 0:
            self.voltage = self.chan.voltage * 4.395
    
    
# A class for reading a TCS34725 ColorSensor         
class ColorSensor():
    def __init__(self):
        # Variable init
        self.color_rgb, self.color_temperature, self.lux = 0, 0, 0
        
        # Colorsensor init
        i2c = board.I2C()
        self.sensor = adafruit_tcs34725.TCS34725(i2c)
        
        
    #Read the sensor data    
    def read(self):
        #Write sensor data to variables
        while self.threadStop == 0:
            self.color_temperature = self.sensor.color_temperature
            # print(self.color_temperature)
            # self.color_rgb = self.sensor.color_rgb_bytes
            # self.lux = self.sensor.lux
        
    
    # Stop the thread for reading the sensor   
    def stop_measurement(self):
        self.sensor.active = False
        self.threadStop = 1
        