import matplotlib.pyplot as plt

# Open the file and read the data
with open(r'C:\Users\Felix\OneDrive - Helmholtz-Gymnasium\Desktop\GitHub\wro2024-fe-HHG_Phoenix\wro2024-fe-HHG_Phoenix\testing\sensor_data.txt', 'r') as f:
    data = f.readlines()

# Initialize a dictionary to store the data
variables = {}

# Loop through each line in the file
for line in data:
    # Split the line into separate values
    values = line.strip().split('; ')

    # Check that the values list has an even number of elements
    if len(values) % 2 != 0:
        continue

    # Loop through each variable in the line
    for i in range(0, len(values), 2):
        # Extract the variable name and value and convert the value to a float
        var_name = values[i]
        var_value = float(values[i+1].replace(',', '.'))

        # If the variable doesn't exist in the dictionary yet, create a new list for it
        if var_name not in variables:
            variables[var_name] = []

        # Append the value to the list for the variable
        variables[var_name].append(var_value)

# Set the plot style to a dark background
plt.style.use('dark_background')

# Plot the data for each variable on a separate plot with a different color
colors = plt.cm.rainbow([i/len(variables) for i in range(len(variables))])
fig, axs = plt.subplots(len(variables), 1, sharex=True)
for i, (var_name, var_values) in enumerate(variables.items()):
    axs[i].plot(range(len(var_values)), var_values, color=colors[i])
    axs[i].set_ylabel(var_name)

# Add a legend and axis labels to the last plot
axs[-1].set_xlabel('Line number')

# Show all plots in one window
plt.show()