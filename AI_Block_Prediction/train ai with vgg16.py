import tkinter as tk
from tkinter import filedialog, messagebox
import tensorflow as tf
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, MaxPooling2D, Flatten, Dense, Dropout, BatchNormalization
from tensorflow.keras.callbacks import EarlyStopping, ModelCheckpoint, ReduceLROnPlateau, CSVLogger
from tensorflow.keras.applications import VGG16
from tensorflow.keras.optimizers import Adam
from sklearn.utils import class_weight
import numpy as np
import os

def select_train_path():
    train_path = filedialog.askdirectory()
    train_path_var.set(train_path)

def select_validation_path():
    validation_path = filedialog.askdirectory()
    validation_path_var.set(validation_path)

def start_training():
    train_path = train_path_var.get()
    validation_path = validation_path_var.get()
    
    if not train_path or not validation_path:
        messagebox.showerror("Error", "Both paths must be selected!")
        return

    # Data augmentation
    train_datagen = ImageDataGenerator(
        rescale=1./255,
        shear_range=0.3,
        zoom_range=0.3,
        rotation_range=20,  # Increased rotation range
        width_shift_range=0.3,
        height_shift_range=0.3,
        brightness_range=[0.5, 1.5],  # Wider brightness range
        horizontal_flip=True,
        vertical_flip=True,  # Include vertical flip
        fill_mode='nearest',  # Fill mode for image augmentation
        validation_split=0.1
    )

    # Load training and validation data
    train_generator = train_datagen.flow_from_directory(
        train_path,
        target_size=(224, 224),
        batch_size=32,
        class_mode='categorical',
        subset='training'
    )

    validation_generator = train_datagen.flow_from_directory(
        validation_path,
        target_size=(224, 224),
        batch_size=32,
        class_mode='categorical',
        subset='validation'
    )

    # Load the VGG16 model pre-trained on ImageNet, including the top (fully connected) layers
    base_model = VGG16(weights='imagenet', include_top=False, input_shape=(224, 224, 3))

    # Unfreeze some of the deeper layers of VGG16
    for layer in base_model.layers[-8:]:  # Unfreezing more layers for better fine-tuning
        layer.trainable = True

    # Add custom top layers
    model = Sequential([
        base_model,
        Flatten(),
        Dense(512, activation='relu'),
        BatchNormalization(),  # Added batch normalization
        Dropout(0.5),
        Dense(256, activation='relu'),
        BatchNormalization(),  # Added batch normalization
        Dropout(0.3),
        Dense(train_generator.num_classes, activation='softmax')
    ])

    # Compile the model
    model.compile(optimizer=Adam(learning_rate=0.0001),
                loss='categorical_crossentropy',
                metrics=['accuracy'])

    # Callbacks
    early_stopping = EarlyStopping(monitor='val_loss', patience=20, restore_best_weights=True)
    model_checkpoint = ModelCheckpoint('best_model.keras', monitor='val_loss', save_best_only=True)
    reduce_lr = ReduceLROnPlateau(monitor='val_loss', factor=0.2, patience=5, min_lr=1e-6)
    csv_logger = CSVLogger('training_log.csv', append=True)

    # Compute class weights
    class_weights = class_weight.compute_class_weight(
        class_weight='balanced',
        classes=np.unique(train_generator.classes),
        y=train_generator.classes
    )
    class_weights = {i: class_weights[i] for i in range(len(class_weights))}

    # Fit the model
    history = model.fit(
        train_generator,
        steps_per_epoch=train_generator.samples // train_generator.batch_size,
        validation_data=validation_generator,
        validation_steps=validation_generator.samples // validation_generator.batch_size,
        epochs=100,  # Increased number of epochs
        class_weight=class_weights,
        callbacks=[early_stopping, model_checkpoint, reduce_lr, csv_logger]
    )

    # Evaluate the model
    loss, accuracy = model.evaluate(validation_generator, steps=validation_generator.samples // validation_generator.batch_size)
    messagebox.showinfo("Training Complete", f'Validation accuracy: {accuracy:.2f}')

    # Save the model
    model.save('cube_classifier_vgg16.keras')
    messagebox.showinfo("Model Saved", "Model has been saved as 'cube_classifier_vgg16.keras'")

# Create the main window
root = tk.Tk()
root.title("TensorFlow Training")

# Variables to store the paths
train_path_var = tk.StringVar()
validation_path_var = tk.StringVar()

# Create and place the widgets
tk.Label(root, text="Training Data Path:").grid(row=0, column=0, padx=10, pady=10)
tk.Entry(root, textvariable=train_path_var, width=50).grid(row=0, column=1, padx=10, pady=10)
tk.Button(root, text="Browse", command=select_train_path).grid(row=0, column=2, padx=10, pady=10)

tk.Label(root, text="Validation Data Path:").grid(row=1, column=0, padx=10, pady=10)
tk.Entry(root, textvariable=validation_path_var, width=50).grid(row=1, column=1, padx=10, pady=10)
tk.Button(root, text="Browse", command=select_validation_path).grid(row=1, column=2, padx=10, pady=10)

tk.Button(root, text="Start Training", command=start_training).grid(row=2, columnspan=3, pady=20)

# Start the main loop
root.mainloop()
