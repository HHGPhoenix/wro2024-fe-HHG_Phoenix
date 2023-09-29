import socket
import time

# Create a socket for the client
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('172.16.5.4', 25565))  # Connect to the server

# Simulate sending data in real-time (you can replace this with your data source)
try:
    with open(r'C:\Users\Felix\OneDrive - Helmholtz-Gymnasium\Desktop\GitHub\wro2024-fe-HHG_Phoenix\wro2024-fe-HHG_Phoenix/testing/sensor_data.txt', 'r') as f:
        data = f.readlines()
        
    for line in data:
        client_socket.send(line.encode())
        time.sleep(0.1)  # Adjust the delay as needed

finally:
    print('Closing socket')
    client_socket.close()  # Close the socket when done