import tensorflow as tf
import numpy as np
from tensorflow.keras.preprocessing import image
from tensorflow.keras.models import load_model, Model
from tensorflow.keras.preprocessing.image import ImageDataGenerator

# Assuming your training images are in 'Dataset stuff\Training' directory
train_datagen = ImageDataGenerator(rescale=1./255)
train_generator = train_datagen.flow_from_directory(
    r'C:/Users/felix/OneDrive - Helmholtz-Gymnasium/Flix,Emul Ordner/Dataset_Pos_V1/Grouped + All - Kopie',
    target_size=(320, 143),
    batch_size=32,
    class_mode='categorical'
)

# Get class labels
class_labels = list(train_generator.class_indices.keys())

# Load the trained model
model = load_model('cube_classifier_507de46d-1f5b-499b-af6b-92ecf479c765_0.61.keras')

# Load the provided image for prediction
img_path = r'Dataset stuff\Testing\Green\frame340.jpg'  # Path to the uploaded image 
img = tf.keras.preprocessing.image.load_img(img_path, target_size=(320, 143))  # Resize the image to (320, 143)
img_array = tf.keras.preprocessing.image.img_to_array(img) / 255.0
img_array = np.expand_dims(img_array, axis=0)

# Predict the class of the image
prediction = model.predict(img_array)

# Get the index of the highest probability
predicted_class_index = np.argmax(prediction)

# Get the label of the predicted class
predicted_class_label = class_labels[predicted_class_index]

# Print the prediction probabilities along with their labels
print(f'Prediction probabilities: {prediction}')
print(f'Predicted class: {predicted_class_label}')