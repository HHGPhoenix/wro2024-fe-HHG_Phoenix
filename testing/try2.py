import keyboard
import RPi.GPIO as GPIO
import time

F = 16
R = 18
Signal = 12

GPIO.setmode(GPIO.BOARD)

GPIO.setup(F, GPIO.OUT)
GPIO.setup(R, GPIO.OUT)
GPIO.setup(Signal, GPIO.OUT)

servo1 = GPIO.PWM(Signal, 50)
servo1.start(6.6)

# Define the key you want to listen for
target_key1 = 'w'
target_key2 = 's'
target_key3 = 'a'
target_key4 = 'd'

# Function to be called when the target key is pressed
def on_key_press(e):
    if e.name == target_key1:
        GPIO.output(F, 1)
    if e.name == target_key2:
        GPIO.output(R, 1)
    if e.name == target_key3:
        servo1.ChangeDutyCycle(5.2)
    if e.name == target_key4:
        servo1.ChangeDutyCycle(8.1)

def on_key_release(e):
    if e.name == target_key1:
        GPIO.output(F, 0)
    if e.name == target_key2:
        GPIO.output(R, 0)
    if e.name == target_key3:
        servo1.ChangeDutyCycle(6.6)
    if e.name == target_key4:
        servo1.ChangeDutyCycle(6.6)


# Register the key press event
keyboard.on_press_key(target_key1, on_key_press)
keyboard.on_release_key(target_key1, on_key_release)
keyboard.on_press_key(target_key2, on_key_press)
keyboard.on_release_key(target_key2, on_key_release)
keyboard.on_press_key(target_key3, on_key_press)
keyboard.on_release_key(target_key3, on_key_release)
keyboard.on_press_key(target_key4, on_key_press)
keyboard.on_release_key(target_key4, on_key_release)
# Keep the script running
keyboard.wait('esc')  # You can change 'esc' to any other key to exit

# Unregister the key press event (optional)
keyboard.unhook_all()
