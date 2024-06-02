import os
import json
import shutil
import re
import uuid
from tkinter import filedialog, simpledialog
from PIL import Image, ImageTk
import customtkinter as ctk
import tkinter.messagebox as messagebox
import tkinter as tk

# JSON file to save settings
json_file = ''

# Set the size for all images (one side)
image_side_size = 500

def move_file(subdir):
    global current_image
    if current_image:
        base, ext = os.path.splitext(current_image)
        new_name = base + str(uuid.uuid4()) + ext
        os.rename(current_image, new_name)
        shutil.move(new_name, os.path.join(subdir))
        current_image = None
        display_next_image()

def resize_image(img, size):
    width, height = img.size
    if width > height:
        new_width = size
        new_height = int((size / width) * height)
    else:
        new_height = size
        new_width = int((size / height) * width)
    return img.resize((new_width, new_height), Image.LANCZOS)

def display_next_image():
    global current_image, current_image_index
    if dir_path and os.path.exists(dir_path):
        jpg_files = [f for f in os.listdir(dir_path) if f.endswith('.jpg')]
        jpg_files = sorted(jpg_files, key=lambda f: int((re.search(r'frame(\d+)', f) or re.search(r'(\d+)', f)).group(1)))
        if jpg_files:
            current_image_index = (current_image_index + 1) % len(jpg_files)
            current_image = os.path.join(dir_path, jpg_files[current_image_index])
            img = Image.open(current_image)
            img = resize_image(img, image_side_size)
            photo = ImageTk.PhotoImage(img)
            label.configure(image=photo)
            label.image = photo
        else:
            label.configure(image=None)

def display_previous_image():
    global current_image, current_image_index
    if dir_path and os.path.exists(dir_path):
        jpg_files = [f for f in os.listdir(dir_path) if f.endswith('.jpg')]
        jpg_files = sorted(jpg_files, key=lambda f: int((re.search(r'frame(\d+)', f) or re.search(r'(\d+)', f)).group(1)))
        if jpg_files:
            current_image_index = (current_image_index - 1) % len(jpg_files)
            current_image = os.path.join(dir_path, jpg_files[current_image_index])
            img = Image.open(current_image)
            img = resize_image(img, image_side_size)
            photo = ImageTk.PhotoImage(img)
            label.configure(image=photo)
            label.image = photo
        else:
            label.configure(image=None)

def select_directory():
    global dir_path
    dir_path = filedialog.askdirectory()
    if dir_path:
        display_next_image()
        save_settings()

def add_subdir():
    subdir = filedialog.askdirectory()
    if subdir:
        subdir_name = os.path.basename(subdir)
        button = ctk.CTkButton(subdir_frame, text=subdir_name, command=lambda subdir=subdir: move_file(subdir))
        buttons.append(button)
        subdirs.append(subdir)
        refresh_subdir_buttons()
        save_settings()

def edit_subdir():
    subdir_index = simpledialog.askinteger("Input", "Enter button number to edit", minvalue=1, maxvalue=len(subdirs))
    if subdir_index is not None:
        new_subdir = filedialog.askdirectory()
        if new_subdir:
            subdirs[subdir_index - 1] = new_subdir
            buttons[subdir_index - 1].configure(text=os.path.basename(new_subdir))
            save_settings()

def remove_subdir():
    subdir_index = simpledialog.askinteger("Input", "Enter button number to remove", minvalue=1, maxvalue=len(subdirs))
    if subdir_index is not None:
        subdirs.pop(subdir_index - 1)
        if buttons[subdir_index - 1] is not None:
            buttons[subdir_index - 1].destroy()
        buttons.pop(subdir_index - 1)
        refresh_subdir_buttons()
        save_settings()

def refresh_subdir_buttons():
    for widget in subdir_frame.winfo_children():
        if widget not in buttons:
            widget.destroy()
    for button in buttons:
        button.pack(padx=5, pady=5, fill='both')

def save_settings():
    if dir_path or subdirs:
        with open(json_file, 'w') as f:
            json.dump({'dir_path': dir_path, 'subdirs': subdirs}, f)


def load_settings():
    global dir_path, subdirs, json_file

    root = tk.Tk()
    root.withdraw()  # Hide the main window

    if not json_file:
        if messagebox.askyesno("Settings file", "Do you want to create a new settings file?"):
            json_file = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON files", "*.json")], title="Create new settings.json", initialfile="settings.json")
            with open(json_file, 'w') as f:
                json.dump({'dir_path': None, 'subdirs': []}, f)
        else:
            json_file = filedialog.askopenfilename(defaultextension=".json", filetypes=[("JSON files", "*.json")], title="Select existing settings.json")

    if os.path.exists(json_file):
        with open(json_file, 'r') as f:
            settings = json.load(f)
            dir_path = settings.get('dir_path')
            subdirs = settings.get('subdirs', [])
            for subdir in subdirs:
                subdir_name = os.path.basename(subdir)
                button = ctk.CTkButton(subdir_frame, text=subdir_name, command=lambda subdir=subdir: move_file(subdir))
                buttons.append(button)
            refresh_subdir_buttons()

def delete_current_image():
    global current_image
    if current_image:
        os.remove(current_image)
        current_image = None
        display_next_image()



# Initialize customtkinter
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

root = ctk.CTk()
root.geometry("800x600")
root.resizable(True, True)

current_image = None
current_image_index = -1
dir_path = None
subdirs = []
buttons = []

label = ctk.CTkLabel(root, text="")
label.pack(pady=10)

select_dir_button = ctk.CTkButton(root, text="Select Directory", command=select_directory)
select_dir_button.pack(pady=5)

add_subdir_button = ctk.CTkButton(root, text="Add Subdirectory", command=add_subdir)
add_subdir_button.pack(pady=5)

edit_subdir_button = ctk.CTkButton(root, text="Edit Subdirectory", command=edit_subdir)
edit_subdir_button.pack(pady=5)

remove_subdir_button = ctk.CTkButton(root, text="Remove Subdirectory", command=remove_subdir)
remove_subdir_button.pack(pady=5)

prev_image_button = ctk.CTkButton(root, text="Previous Image", command=display_previous_image)
prev_image_button.pack(pady=5)

delete_image_button = ctk.CTkButton(root, text="Delete Image", command=delete_current_image)
delete_image_button.pack(pady=5)

subdir_frame = ctk.CTkFrame(root)
subdir_frame.pack(pady=10, fill="both", expand=True)

load_settings()

root.mainloop()
