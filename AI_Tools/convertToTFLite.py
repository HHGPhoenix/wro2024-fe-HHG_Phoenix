import tensorflow as tf

# Save the model first
model = tf.keras.models.load_model('cube_classifier.keras')

# Convert the model
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()

# Save the model.
with open('cube_classifier.tflite', 'wb') as f:
    f.write(tflite_model)