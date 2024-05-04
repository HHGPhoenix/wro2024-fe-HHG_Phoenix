import serial
import time

def send_data(data):
    try:
        # Open the serial port
        ser = serial.Serial('/dev/ttyUSB0', baudrate=921600)
        
        # Write the data to the serial port
        ser.write(data.encode("utf-8"))
        
        # Read the response from the serial port
        response = ser.readline().decode().strip()
        
        # Close the serial port
        ser.close()
        
        print("Data sent successfully!")
        print("Response:", response)
    except serial.SerialException as e:
        print(f"Error: {e}")

while True:
    # Get user input
    # user_input = input("Enter the data to send: ")
    user_input = f"IDENT\n"

    # Call the send_data function with the user input
    send_data(user_input)
    time.sleep(1)