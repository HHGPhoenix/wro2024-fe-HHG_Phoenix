import matplotlib.pyplot as plt
import numpy as np

# Initialize a dictionary to store different types of values
values = {"avg_edge_distance": [], "distance": [], "self.block_distance_to_wall": [], " self.nextBlock['x']": []}

# Open the log file and read lines
with open(r'C:\Users\Admin\OneDrive\Dokumente\GitHub\wro2024-fe-HHG_Phoenix\Code\RPi\Debug.log', 'r') as file:
    for line in file:
        # Split the line into parts
        parts = line.strip().split(', ')

        for part in parts:
            # Check if part can be split into label and value
            if ': ' not in part:
                continue

            # Split each part into label and value
            label, value = part.split(': ')
            
            if label not in values:
                continue

            # Convert value to float and append to the appropriate list
            values[label].append(float(value))

# Find the minimum and maximum values across all lists in the dictionary
min_value = np.min([np.min(v) for v in values.values() if v])
max_value = np.max([np.max(v) for v in values.values() if v])

# Plot the values
for label, value_list in values.items():
    plt.plot(value_list, label=label)

plt.xlabel('Time')
plt.ylabel('Value')
plt.title('Values over Time')
plt.legend()
plt.grid(True)

# Set the limits of the y-axis
plt.ylim(min_value, max_value)

plt.show()