import serial
import time

# Define the serial port and baud rate
serial_port = '/dev/ttyUSB0'
baud_rate = 115200  # You should set this to the appropriate baud rate

try:
    # Open the serial port
    ser = serial.Serial(serial_port, baud_rate)

    while True:
        # Read and print serial data
        time.sleep(0.1)
        if ser.in_waiting:
            data = ser.readline()
            print(f"Received: {data.decode('utf-8').strip()}")

        # Input data and send it to the serial port
        user_input = input("Enter data to send (or press Enter to continue): ")
        if user_input:
            ser.write(user_input.encode('utf-8') + b'\n')

except serial.SerialException as e:
    print(f"Error: {e}")
finally:
    # Close the serial port when done
    ser.close()
