import os
import json
import shutil
from tkinter import *
from tkinter import filedialog, simpledialog
from PIL import Image, ImageTk
import re
import uuid

# JSON file to save settings
json_file = 'settings.json'

# Set the size for all images
image_size = (500, 500)


def move_file(subdir):
    global current_image
    if current_image:
        # Generate a unique name for the file
        base, ext = os.path.splitext(current_image)
        new_name = base + str(uuid.uuid4()) + ext

        # Rename the file
        os.rename(current_image, new_name)

        # Move the file
        shutil.move(new_name, os.path.join(dir_path, subdir))
        current_image = None
        display_next_image()

def display_next_image():
    global current_image
    if dir_path and os.path.exists(dir_path):  # Check if dir_path is not None and exists
        jpg_files = [f for f in os.listdir(dir_path) if f.endswith('.jpg')]
        # Extract the frame number from the file name and sort the files by it
        jpg_files = sorted(jpg_files, key=lambda f: int(re.search(r'frame(\d+)', f).group(1)))
        if jpg_files:
            current_image = os.path.join(dir_path, jpg_files[0])
            img = Image.open(current_image)
            img = img.resize(image_size, Image.LANCZOS)  # Resize the image
            photo = ImageTk.PhotoImage(img)
            label.config(image=photo)
            label.image = photo
        else:
            label.config(image=None)

def select_directory():
    global dir_path
    dir_path = filedialog.askdirectory()
    if dir_path:  # Only display next image if a directory was selected
        display_next_image()
        save_settings()

# Rest of the code remains the same

def add_subdir():
    subdir = filedialog.askdirectory()
    if subdir:
        button = Button(root, text=subdir, command=lambda subdir=subdir: move_file(subdir), relief=FLAT)  # Make the button flat
        button.pack()
        subdirs.append(subdir)
        save_settings()

def edit_subdir():
    subdir_index = simpledialog.askinteger("Input", "Enter button number to edit", minvalue=1, maxvalue=len(subdirs))
    if subdir_index is not None:
        new_subdir = filedialog.askdirectory()
        if new_subdir:
            subdirs[subdir_index - 1] = new_subdir
            buttons[subdir_index - 1]['text'] = new_subdir
            save_settings()

def remove_subdir():
    subdir_index = simpledialog.askinteger("Input", "Enter button number to remove", minvalue=1, maxvalue=len(subdirs))
    if subdir_index is not None:
        subdirs.pop(subdir_index - 1)
        buttons[subdir_index - 1].destroy()
        buttons.pop(subdir_index - 1)
        save_settings()

def save_settings():
    if dir_path and subdirs:  # Only save if dir_path and subdirs are not None
        with open(json_file, 'w') as f:
            json.dump({'dir_path': dir_path, 'subdirs': subdirs}, f)

def load_settings():
    global dir_path, subdirs
    if os.path.exists(json_file):
        with open(json_file, 'r') as f:
            settings = json.load(f)
            dir_path = settings.get('dir_path')
            subdirs = settings.get('subdirs', [])
            for subdir in subdirs:
                button = Button(root, text=subdir, command=lambda subdir=subdir: move_file(subdir), relief=FLAT)  # Make the button flat
                button.pack()
                buttons.append(button)

root = Tk()

# Set the initial size of the window and enable the maximize button
root.geometry("800x600")
root.resizable(True, True)

# Use native window icons
# root.iconbitmap(default='icon.ico')

current_image = None
dir_path = None
subdirs = []
buttons = []

label = Label(root)
label.pack()

select_dir_button = Button(root, text="Select Directory", command=select_directory, relief=FLAT)  # Make the button flat
select_dir_button.pack()

add_subdir_button = Button(root, text="Add Subdirectory", command=add_subdir, relief=FLAT)  # Make the button flat
add_subdir_button.pack()

edit_subdir_button = Button(root, text="Edit Subdirectory", command=edit_subdir, relief=FLAT)  # Make the button flat
edit_subdir_button.pack()

remove_subdir_button = Button(root, text="Remove Subdirectory", command=remove_subdir, relief=FLAT)  # Make the button flat
remove_subdir_button.pack()

load_settings()

root.mainloop()