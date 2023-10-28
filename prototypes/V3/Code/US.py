import serial
import time

# Open a connection to the Arduino over the USB serial port
arduino = serial.Serial('COM5', 9600)  # Replace 'COM3' with the correct port on your system

try:
    while True:
        command = input("Enter a command (D for distance, S1/S2 to switch sensors): ")
        
        if command.startswith('S'):
            arduino.write(command.encode())  # Send the sensor switch command
        elif command == 'D':
            desired_distance = float(input("Enter desired distance (in cm): "))
            arduino.write(f"D {desired_distance:.2f}\n".encode())  # Send the desired distance with two decimal places
        else:
            print("Invalid command. Use 'D' for distance or 'S1'/'S2' to switch sensors.")
        
        time.sleep(1)  # Adjust the delay as needed

except KeyboardInterrupt:
    print("Exiting the script.")
finally:
    arduino.close()  # Close the serial connection when done
