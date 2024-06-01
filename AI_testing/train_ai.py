import tensorflow as tf
from tensorflow.keras.preprocessing.image import ImageDataGenerator # type: ignore

# Define your model architecture
input_dim = (640, 285, 3)  # Replace with the actual input dimension (height, width, channels)
num_classes = 3  # Number of classes (Red, Green, Nothing)
num_epochs = 10  # Replace 10 with the actual number of epochs
batch_size = 32  # Replace 32 with the actual batch size

# Create an ImageDataGenerator for training data
train_datagen = ImageDataGenerator(rescale=1./255)  # Rescale pixel values to [0,1]
train_generator = train_datagen.flow_from_directory(
    'D:/Datasets/WRO Beta test',  # Path to the training data
    target_size=input_dim[:2],  # Resizes images to `input_dim`
    batch_size=batch_size,
    class_mode='categorical',  # For multi-class, categorical outputs
    classes=['Red', 'Green', 'Nothing']  # Explicitly specify the order of classes
)

# Create an ImageDataGenerator for test data
test_datagen = ImageDataGenerator(rescale=1./255)  # Rescale pixel values to [0,1]
test_generator = test_datagen.flow_from_directory(
    'D:/Datasets/WRO Beta test/Testing',  # Path to the test data
    target_size=input_dim[:2],  # Resizes images to `input_dim`
    batch_size=batch_size,
    class_mode='categorical',  # For multi-class, categorical outputs
)

model = tf.keras.Sequential([
    tf.keras.layers.Conv2D(32, (3, 3), activation='relu', input_shape=input_dim),
    tf.keras.layers.MaxPooling2D((2, 2)),
    tf.keras.layers.Conv2D(64, (3, 3), activation='relu'),
    tf.keras.layers.MaxPooling2D((2, 2)),
    tf.keras.layers.Flatten(),
    tf.keras.layers.Dense(64, activation='relu'),
    tf.keras.layers.Dense(num_classes, activation='softmax')
])

# Compile the model
model.compile(optimizer='adam',
              loss=tf.keras.losses.CategoricalCrossentropy(from_logits=True),
              metrics=['accuracy'])

# Train the model
model.fit(train_generator, epochs=num_epochs)

# Evaluate the model
test_loss, test_acc = model.evaluate(test_generator)
print('Test accuracy:', test_acc)