import tensorflow as tf
import numpy as np
# import matplotlib.pyplot as plt
from tensorflow.keras.preprocessing import image
from tensorflow.keras.models import load_model, Model

# Load the trained model
# model = load_model('cube_classifier.tflite')
model = tf.lite.Interpreter('cube_classifier.tflite')
model.allocate_tensors()

# Get input and output tensors.
input_details = model.get_input_details()
output_details = model.get_output_details()

# Load the provided image for prediction
img_path = r'Dataset stuff/Testing/Red/frame455.jpg'  # Path to the uploaded image
img = tf.keras.preprocessing.image.load_img(img_path, target_size=(320, 143))
img_array = tf.keras.preprocessing.image.img_to_array(img) / 255.0
img_array = np.expand_dims(img_array, axis=0)

# Set the tensor to point to the input data to be inferred
model.set_tensor(input_details[0]['index'], img_array)

# Run the inference
model.invoke()

# The function `get_tensor()` returns a copy of the tensor data.
# Use `tensor()` in order to get a pointer to the tensor.
prediction = model.get_tensor(output_details[0]['index'])

# Print the prediction probabilities
print(f'Prediction probabilities: {prediction}')

# Assuming these are your class labels in the same order as the output
class_labels = ['Green', 'Nothing', 'Red']

# Find the index of the highest probability
predicted_index = np.argmax(prediction)

# Get the class label
predicted_class = class_labels[predicted_index]

print(f'The predicted class is: {predicted_class}')