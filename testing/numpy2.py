import numpy as np
from scipy.stats import linregress

# Define your data points
y_values = np.array([5.1, 6.6, 8.1])
x_values = np.array([-100, 0, 100])

# Perform linear regression
slope, intercept, r_value, p_value, std_err = linregress(x_values, y_values)

# Print the results
print(f"Slope: {slope}")
print(f"Intercept: {intercept}")
print(f"R-squared: {r_value**2}")

# Create the equation of the line
equation = f"y = {slope:.2f}x + {intercept:.2f}"

print(f"Equation: {equation}")
