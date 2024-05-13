import matplotlib.pyplot as plt

# Initialize a dictionary to store different types of values
values = {"ColorTemperature": []}

# Open the log file and read lines
with open(r'D:\Developement\wro2024-fe-HHG_Phoenix\Code\RPi\DataLog.log', 'r') as file:
    for line in file:
        # Split the line into parts
        parts = line.strip().split(', ')

        for part in parts:
            # Split each part into label and value
            label, value = part.split(': ')
            
            if label not in values:
                continue

            # Convert value to float and append to the appropriate list
            values[label].append(float(value))

# Plot the values
for label, value_list in values.items():
    plt.plot(value_list, label=label)

plt.xlabel('Time')
plt.ylabel('Value')
plt.title('Values over Time')
plt.legend()
plt.grid(True)
plt.show()