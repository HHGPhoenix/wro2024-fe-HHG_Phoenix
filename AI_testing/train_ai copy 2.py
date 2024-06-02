import tensorflow as tf
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, MaxPooling2D, Flatten, Dense, Dropout
from tensorflow.keras.callbacks import EarlyStopping
from sklearn.utils import class_weight
from tensorflow.keras.optimizers import Adam
import numpy as np
from tensorflow.keras.losses import SparseCategoricalCrossentropy

# Data augmentation
train_datagen = ImageDataGenerator(
    rescale=1./255,
    shear_range=0.2,
    zoom_range=0.2,
    horizontal_flip=True,
    validation_split=0.2
)

# Load training and validation data
train_generator = train_datagen.flow_from_directory(
    r'Dataset stuff\WRO Beta test',
    target_size=(320, 143),  # changed from (128, 128) to (320, 143)
    batch_size=32,
    class_mode='categorical',
    subset='training'
)

validation_generator = train_datagen.flow_from_directory(
    r'Dataset stuff\Testing',
    target_size=(320, 143),  # changed from (128, 128) to (320, 143)
    batch_size=32,
    class_mode='categorical',
    subset='validation'
)

# # Add dropout layers to the model
# model = Sequential([
#     Conv2D(32, (3, 3), activation='relu', input_shape=(320, 143, 3)),
#     MaxPooling2D(pool_size=(2, 2)),
#     Dropout(0.25),
#     Conv2D(64, (3, 3), activation='relu'),
#     MaxPooling2D(pool_size=(2, 2)),
#     Dropout(0.25),
#     Conv2D(128, (3, 3), activation='relu'),
#     MaxPooling2D(pool_size=(2, 2)),
#     Dropout(0.25),
#     Flatten(),
#     Dense(128, activation='relu'),
#     Dropout(0.5),
#     Dense(3, activation='relu')
# ])

model = Sequential([
    Flatten(input_shape=(320, 143, 3)),
    Dense(128, activation='relu'),
    Dense(3)
])


# Add early stopping
early_stopping = EarlyStopping(monitor='val_loss', patience=5)

print(np.unique(train_generator.classes))

model.compile(optimizer='adam',
              loss=tf.keras.losses.CategoricalCrossentropy(from_logits=True),
              metrics=['accuracy'])

# Fit the model
history = model.fit(
    train_generator,
    steps_per_epoch=train_generator.samples // train_generator.batch_size,
    validation_data=validation_generator,
    validation_steps=validation_generator.samples // validation_generator.batch_size,
    epochs=1,
    callbacks=[early_stopping]  # use early stopping
)

loss, accuracy = model.evaluate(validation_generator, steps=validation_generator.samples // validation_generator.batch_size)
print(f'Validation accuracy: {accuracy:.2f}')

model.save('cube_classifier.h5')

# import tensorflow as tf

# model2 = tf.keras.models.load_model('cube_classifier.h5', compile=False)

# Converting a tf.Keras model to a TensorFlow Lite model.
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()


# Save the model to disk
open("cube_classifier.tflite", "wb").write(tflite_model)