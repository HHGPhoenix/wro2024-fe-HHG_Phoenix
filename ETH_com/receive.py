import socket

# Create a socket object
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Set the IP address and port number to listen on
ip_address = '0.0.0.0'  # Listen on all interfaces
port = 1234  # Replace with your desired port number

# Bind the socket to the IP address and port
sock.bind((ip_address, port))

# Listen for incoming connections
sock.listen(1)

try:
    while True:
        # Accept a connection from a client
        print("Waiting for a connection...")
        client_socket, client_address = sock.accept()
        print(f"Connected to {client_address}")
        
        try:
            while True:
                # Receive data from the client
                data = client_socket.recv(4096)
                if not data:
                    break  # If no data is received, exit the inner loop
                
                # Echo the data back to the client
                client_socket.sendall(data)
                
        except ConnectionResetError:
            print("Connection was closed by the client.")
        finally:
            # Close the client socket
            client_socket.close()
            print(f"Connection to {client_address} closed.")

finally:
    # Close the server socket
    sock.close()