import tkinter as tk
from tkinter import filedialog
import tensorflow as tf
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, MaxPooling2D, Flatten, Dense, Dropout
from tensorflow.keras.callbacks import EarlyStopping, ModelCheckpoint
from tensorflow.keras.regularizers import l2
from tensorflow.keras.optimizers import Adam
import numpy as np
from kerastuner import HyperModel, RandomSearch
import matplotlib.pyplot as plt
import uuid
import os

#print all gpu devices
print("Num GPUs Available: ", len(tf.config.experimental.list_physical_devices('GPU')))

for gpu in tf.config.experimental.list_physical_devices('GPU'):
    tf.config.experimental.set_memory_growth(gpu, True)

def select_dataset_path():
    path = filedialog.askdirectory()
    dataset_path_var.set(path)

class CNNHyperModel(HyperModel):
    def build(self, hp):
        model = Sequential()
        model.add(Conv2D(
            filters=hp.Int('conv_1_filter', min_value=32, max_value=128, step=16),
            kernel_size=hp.Choice('conv_1_kernel', values=[3, 5]),
            activation='relu',
            kernel_regularizer=l2(0.001),
            input_shape=(320, 143, 3)
        ))
        model.add(MaxPooling2D(pool_size=(2, 2)))
        model.add(Dropout(hp.Float('dropout_1', min_value=0.2, max_value=0.5, step=0.1)))

        model.add(Conv2D(
            filters=hp.Int('conv_2_filter', min_value=32, max_value=128, step=16),
            kernel_size=hp.Choice('conv_2_kernel', values=[3, 5]),
            activation='relu',
            kernel_regularizer=l2(0.001)
        ))
        model.add(MaxPooling2D(pool_size=(2, 2)))
        model.add(Dropout(hp.Float('dropout_2', min_value=0.2, max_value=0.5, step=0.1)))

        model.add(Flatten())

        model.add(Dense(
            units=hp.Int('dense_units', min_value=64, max_value=256, step=32),
            activation='relu',
            kernel_regularizer=l2(0.001)
        ))
        model.add(Dropout(hp.Float('dropout_3', min_value=0.2, max_value=0.5, step=0.1)))
        
        model.add(Dense(train_generator.num_classes, activation='softmax'))
        
        model.compile(optimizer=Adam(hp.Choice('learning_rate', values=[1e-2, 1e-3, 1e-4])),
                      loss='categorical_crossentropy',
                      metrics=['accuracy'])
        return model

def plot_training_history(history, model_id):
    # Plot training & validation accuracy values
    plt.figure(figsize=(12, 4))
    plt.subplot(1, 2, 1)
    plt.plot(history.history['accuracy'])
    plt.plot(history.history['val_accuracy'])
    plt.title('Model accuracy')
    plt.ylabel('Accuracy')
    plt.xlabel('Epoch')
    plt.legend(['Train', 'Validation'], loc='upper left')

    # Plot training & validation loss values
    plt.subplot(1, 2, 2)
    plt.plot(history.history['loss'])
    plt.plot(history.history['val_loss'])
    plt.title('Model loss')
    plt.ylabel('Loss')
    plt.xlabel('Epoch')
    plt.legend(['Train', 'Validation'], loc='upper left')

    plt.savefig(f'training_history_{model_id}.png')
    plt.show()

def start_training():
    try:
        dataset_path = dataset_path_var.get()

        # Data augmentation
        datagen = ImageDataGenerator(
            rescale=1./255,
            shear_range=0.2,
            zoom_range=0.2,
            rotation_range=20,
            width_shift_range=0.2,
            height_shift_range=0.2,
            horizontal_flip=True,
            validation_split=0.2
        )

        # Load training and validation data
        global train_generator
        train_generator = datagen.flow_from_directory(
            dataset_path,
            target_size=(320, 143),
            batch_size=32,
            class_mode='categorical',
            subset='training'
        )

        validation_generator = datagen.flow_from_directory(
            dataset_path,
            target_size=(320, 143),
            batch_size=32,
            class_mode='categorical',
            subset='validation'
        )

        hypermodel = CNNHyperModel()

        tuner = RandomSearch(
            hypermodel,
            objective='val_accuracy',
            max_trials=20,
            executions_per_trial=1,
            directory='hyperparameter_tuning',
            project_name='cnn_tuning'
        )

        tuner.search(train_generator, epochs=20, validation_data=validation_generator)
    finally:
        best_model = tuner.get_best_models(num_models=1)[0]

        best_model.summary()

        # Generate a unique ID for the model
        model_id = str(uuid.uuid4())

        # Early stopping and model checkpoint
        early_stopping = EarlyStopping(monitor='val_loss', patience=10)
        model_checkpoint = ModelCheckpoint(f'best_model_{model_id}_{{val_accuracy:.2f}}.keras', monitor='val_accuracy', save_best_only=True)

        # Fit the best model
        history = best_model.fit(
            train_generator,
            steps_per_epoch=train_generator.samples // train_generator.batch_size,
            validation_data=validation_generator,
            validation_steps=validation_generator.samples // validation_generator.batch_size,
            epochs=50,
            callbacks=[early_stopping, model_checkpoint]
        )

        loss, accuracy = best_model.evaluate(validation_generator)
        print(f'Validation accuracy: {accuracy:.2f}')

        # Save the model with the accuracy in the filename
        model_filename = f'cube_classifier_{model_id}_{accuracy:.2f}.keras'
        best_model.save(model_filename)

        # Plot and save training history
        plot_training_history(history, model_id)

# Tkinter GUI setup
root = tk.Tk()
root.title("Train Model")

dataset_path_var = tk.StringVar()

tk.Label(root, text="Dataset Path:").grid(row=0, column=0, padx=10, pady=10)
tk.Entry(root, textvariable=dataset_path_var, width=50).grid(row=0, column=1, padx=10, pady=10)
tk.Button(root, text="Browse", command=select_dataset_path).grid(row=0, column=2, padx=10, pady=10)

tk.Button(root, text="Start Training", command=start_training).grid(row=1, column=0, columnspan=3, pady=20)

root.mainloop()
