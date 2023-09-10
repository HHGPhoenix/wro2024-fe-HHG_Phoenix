def interpolate_value(value, min_value, max_value, min_percentage, max_percentage):
    # Ensure that value is within the range [min_value, max_value]
    value = max(min(value, max_value), min_value)

    # Calculate the percentage based on the given values
    percentage = ((value - min_value) / (max_value - min_value)) * 100

    # Interpolate the result between min_percentage and max_percentage
    interpolated_value = ((max_percentage - min_percentage) * (percentage - 0) / (100 - 0)) + min_percentage

    return interpolated_value

# Define your input values
min_value = 5.1
max_value = 8.2
target_value = 6.6

# Calculate the interpolated percentage for the target value
interpolated_percentage = interpolate_value(target_value, min_value, max_value, 100, -100)

print(f"The percentage corresponding to {target_value} is {interpolated_percentage:.2f}%.")