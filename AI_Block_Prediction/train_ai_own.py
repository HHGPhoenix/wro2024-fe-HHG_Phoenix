import tkinter as tk
from tkinter import filedialog
import tensorflow as tf
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, MaxPooling2D, Flatten, Dense, Dropout
from tensorflow.keras.callbacks import EarlyStopping
from sklearn.utils import class_weight
from tensorflow.keras.optimizers import Adam
import numpy as np

def select_train_path():
    path = filedialog.askdirectory()
    train_path_var.set(path)

def select_validation_path():
    path = filedialog.askdirectory()
    val_path_var.set(path)

def start_training():
    train_path = train_path_var.get()
    val_path = val_path_var.get()

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
        train_path,
        target_size=(320, 143),  # changed from (128, 128) to (320, 143)
        batch_size=32,
        class_mode='categorical',
        subset='training'
    )

    validation_generator = train_datagen.flow_from_directory(
        val_path,
        target_size=(320, 143),  # changed from (128, 128) to (320, 143)
        batch_size=32,
        class_mode='categorical',
        subset='validation'
    )

    # Add dropout layers to the model
    model = Sequential([
        Conv2D(32, (3, 3), activation='relu', input_shape=(320, 143, 3)),
        MaxPooling2D(pool_size=(2, 2)),
        Dropout(0.25),
        Conv2D(64, (3, 3), activation='relu'),
        MaxPooling2D(pool_size=(2, 2)),
        Dropout(0.25),
        Conv2D(128, (3, 3), activation='relu'),
        MaxPooling2D(pool_size=(2, 2)),
        Dropout(0.25),
        Flatten(),
        Dense(128, activation='relu'),
        Dropout(0.5),
        Dense(train_generator.num_classes, activation='softmax')
    ])

    # Add early stopping
    early_stopping = EarlyStopping(monitor='val_loss', patience=5)

    print(np.unique(train_generator.classes))

    model.compile(optimizer=Adam(),
                  loss='categorical_crossentropy',
                  metrics=['accuracy'])

    # Fit the model
    history = model.fit(
        train_generator,
        steps_per_epoch=train_generator.samples // train_generator.batch_size,
        validation_data=validation_generator,
        validation_steps=validation_generator.samples // validation_generator.batch_size,
        epochs=18,
        callbacks=[early_stopping]  # use early stopping
    )

    loss, accuracy = model.evaluate(validation_generator)
    print(f'Validation accuracy: {accuracy:.2f}')

    model.save('cube_classifier.keras')

# Tkinter GUI setup
root = tk.Tk()
root.title("Train Model")

train_path_var = tk.StringVar()
val_path_var = tk.StringVar()

tk.Label(root, text="Training Dataset Path:").grid(row=0, column=0, padx=10, pady=10)
tk.Entry(root, textvariable=train_path_var, width=50).grid(row=0, column=1, padx=10, pady=10)
tk.Button(root, text="Browse", command=select_train_path).grid(row=0, column=2, padx=10, pady=10)

tk.Label(root, text="Validation Dataset Path:").grid(row=1, column=0, padx=10, pady=10)
tk.Entry(root, textvariable=val_path_var, width=50).grid(row=1, column=1, padx=10, pady=10)
tk.Button(root, text="Browse", command=select_validation_path).grid(row=1, column=2, padx=10, pady=10)

tk.Button(root, text="Start Training", command=start_training).grid(row=2, column=0, columnspan=3, pady=20)

root.mainloop()
