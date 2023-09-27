import socket
import matplotlib.pyplot as plt

# Create a socket for the server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('25.58.153.31', 12345))  # Choose a port number
server_socket.listen(1)  # Listen for one incoming connection

fig, axs, variables = None, None, {}  # Initialize the figure, axes, and data variables

def reset_plot():
    global fig, axs, variables
    if fig:
        plt.close(fig)  # Close the current figure
    fig, axs = None, None  # Reset figure and axes
    variables = {}  # Clear the data variables

while True:
    try:
        # Accept a connection from the client
        plt.style.use('dark_background')
        client_socket, client_address = server_socket.accept()
        print(f"Connected to {client_address}")

        reset_plot()  # Reset the plot and data when a new client connects

        while True:
            # Receive data from the client
            data_received = client_socket.recv(1024).decode()

            if not data_received:
                # Client disconnected, reset the plot and data, and exit inner loop
                reset_plot()
                print(f"Disconnected from {client_address}")
                client_socket.close()
                break

            # Split the received data into lines
            lines = data_received.strip().split('\n')

            # Process each line of received data
            for line in lines:
                # Split the line into separate values
                values = line.strip().split('; ')

                # Check that the values list has an even number of elements
                if len(values) % 2 != 0:
                    continue

                # Loop through each variable in the line
                for i in range(0, len(values), 2):
                    # Extract the variable name and value and convert the value to a float
                    var_name = values[i]
                    try:
                        var_value = float(values[i+1].replace(',', '.'))
                    except ValueError:
                        # Handle invalid data gracefully (e.g., skip or log the error)
                        print(f"Invalid value: {values[i+1]} for variable {var_name}")
                        continue

                    # If the variable doesn't exist in the dictionary yet, create a new list for it
                    if var_name not in variables:
                        variables[var_name] = []

                    # Append the value to the list for the variable
                    variables[var_name].append(var_value)

                # Plot the data for each variable in real-time
                if fig is None:
                    num_variables = len(variables)
                    colors = plt.cm.rainbow([i / num_variables for i in range(num_variables)])
                    fig, axs = plt.subplots(num_variables, 1, sharex=True)
                    if num_variables == 1:
                        axs = [axs]  # Ensure axs is a list even for a single subplot

                    for i, (var_name, var_values) in enumerate(variables.items()):
                        axs[i].set_ylabel(var_name)

                    # Set the background color to dark gray

                else:
                    for i, (var_name, var_values) in enumerate(variables.items()):
                        axs[i].clear()  # Clear the previous plot data
                        axs[i].plot(range(len(var_values)), var_values, color=colors[i])
                        axs[i].set_ylabel(var_name)

                    # Remove extra subplots if the number of variables has decreased
                    num_existing_subplots = len(axs)
                    if num_existing_subplots > num_variables:
                        for i in range(num_variables, num_existing_subplots):
                            fig.delaxes(axs[i])

                axs[-1].set_xlabel('Line number')
                plt.pause(0.01)  # Pause to update the plot

    except Exception as e:
        print(f"An error occurred: {e}")
        reset_plot()  # Restart the graph
