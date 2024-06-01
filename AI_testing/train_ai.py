import tensorflow as tf
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.callbacks import EarlyStopping, ModelCheckpoint

# Same as before...
input_dim = (640, 285, 3)
num_classes = 3
num_epochs = 10
batch_size = 32

# Add data augmentation to the training data generator
train_datagen = ImageDataGenerator(
    rescale=1./255,
    rotation_range=20,
    width_shift_range=0.2,
    height_shift_range=0.2,
    horizontal_flip=True
)

train_generator = train_datagen.flow_from_directory(
    r".\WRO Beta test",
    target_size=input_dim[:2],
    batch_size=batch_size,
    class_mode='categorical',
    classes=['Red', 'Green', 'Nothing']
)

# Same as before...
test_datagen = ImageDataGenerator(rescale=1./255)
test_generator = test_datagen.flow_from_directory(
    r'.\WRO Beta test\Testing',
    target_size=input_dim[:2],
    batch_size=batch_size,
    class_mode='categorical',
)

# Add dropout layers to the model
model = tf.keras.Sequential([
    tf.keras.layers.Conv2D(16, (3, 3), 1, activation='relu', input_shape=input_dim),
    tf.keras.layers.MaxPooling2D(),
    
    tf.keras.layers.Conv2D(32, (3, 3), 1, activation='relu'),
    tf.keras.layers.MaxPooling2D(),
    
    tf.keras.layers.Conv2D(16, (3, 3), 1, activation='relu'),
    tf.keras.layers.MaxPooling2D(),
    
    tf.keras.layers.Flatten(),
    
    tf.keras.layers.Dense(256, activation='relu'),
    tf.keras.layers.Dense(3, activation='sigmoid'),
])

# Same as before...
model.compile(optimizer='adam',
              loss=tf.keras.losses.CategoricalCrossentropy(from_logits=True),
              metrics=['accuracy'])

# Add early stopping and model checkpointing
callbacks = [
    EarlyStopping(patience=5, restore_best_weights=True),
    ModelCheckpoint('model.keras', save_best_only=True)
]

# Train the model with validation data
model.fit(train_generator, epochs=num_epochs, validation_data=test_generator, callbacks=callbacks)

# Evaluate the model
test_loss, test_acc = model.evaluate(test_generator)
print('Test accuracy:', test_acc)