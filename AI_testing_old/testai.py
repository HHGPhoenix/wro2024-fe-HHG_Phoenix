import tensorflow as tf
import numpy as np
# import matplotlib.pyplot as plt
from tensorflow.keras.preprocessing import image
from tensorflow.keras.models import load_model, Model

# Load the trained model
model = load_model('cube_classifier_vgg16.keras')

# Load the provided image for prediction
img_path = r'Dataset stuff/Testing/Green/frame328.jpg'  # Path to the uploaded image
img = tf.keras.preprocessing.image.load_img(img_path, target_size=(224, 224))
img_array = tf.keras.preprocessing.image.img_to_array(img) / 255.0
img_array = np.expand_dims(img_array, axis=0)

# Predict the class of the image
prediction = model.predict(img_array)

# Print the prediction probabilities
print(f'Prediction probabilities: {prediction}')

# Assuming these are your class labels in the same order as the output
class_labels = ['Green', 'Nothing', 'Red']

# Find the index of the highest probability
predicted_index = np.argmax(prediction)

# Get the class label
predicted_class = class_labels[predicted_index]

print(f'The predicted class is: {predicted_class}')

# # Initialize the model with a dummy input to define the input shape
# dummy_input = np.zeros((1, 128, 128, 3))
# model.predict(dummy_input)

# # Create a model that outputs the activations of the first layer
# layer_outputs = [layer.output for layer in model.layers[:1]]  # Get the output of the first layer
# activation_model = Model(inputs=wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww.input(), outputs=layer_outputs)

# # Get the activations
# activations = activation_model.predict(img_array)

# # Assuming the first layer is a Conv2D layer
# first_layer_activation = activations[0]

# # Number of filters in the first layer
# num_filters = first_layer_activation.shape[-1]

# # Plot the activations of the first layer
# plt.figure(figsize=(20, 20))
# for i in range(num_filters):
#     plt.subplot(8, 8, i + 1)  # Adjust the subplot grid size according to the number of filters
#     plt.imshow(first_layer_activation[0, :, :, i], cmap='viridis')
#     plt.axis('off')

# plt.show()
