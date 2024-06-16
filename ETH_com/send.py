import socket
import time

# Create a socket object
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Define the IP address and port of the receiver
ip_address = '192.168.1.2'  # Replace with the actual IP address
port = 1234  # Replace with the actual port number

# Connect to the receiver
sock.connect((ip_address, port))

while True:
    message = input("Enter your message to send (or 'exit' to quit): ")
    
    if message == 'exit':
        break
    
    # Send the message
    sock.sendall(message.encode())

    # Start the timer
    start_time = time.time()

    # Wait for the echo
    echo = sock.recv(1024).decode()

    # Stop the timer
    end_time = time.time()

    # Calculate the round-trip time
    rtt = end_time - start_time

    print(f"Received echo: {echo}")
    print(f"Round-trip time: {rtt} seconds")

# Close the socket
sock.close()