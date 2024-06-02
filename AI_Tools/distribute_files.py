import os
import shutil
import random
import tkinter as tk
from tkinter import filedialog, messagebox

def select_main_folder():
    folder = filedialog.askdirectory()
    if folder:
        main_folder_entry.delete(0, tk.END)
        main_folder_entry.insert(0, folder)

def select_target_folder_1():
    folder = filedialog.askdirectory()
    if folder:
        target_folder_1_entry.delete(0, tk.END)
        target_folder_1_entry.insert(0, folder)

def select_target_folder_2():
    folder = filedialog.askdirectory()
    if folder:
        target_folder_2_entry.delete(0, tk.END)
        target_folder_2_entry.insert(0, folder)

def purge_folder(folder):
    for root, dirs, files in os.walk(folder):
        for file in files:
            os.remove(os.path.join(root, file))
        for dir in dirs:
            shutil.rmtree(os.path.join(root, dir))

def start_distribution():
    main_folder = main_folder_entry.get()
    target_folder_1 = target_folder_1_entry.get()
    target_folder_2 = target_folder_2_entry.get()
    try:
        percentage = float(percentage_entry.get())
        if not (0 <= percentage <= 100):
            raise ValueError
    except ValueError:
        messagebox.showerror("Invalid Input", "Please enter a valid percentage between 0 and 100.")
        return

    if not (main_folder and target_folder_1 and target_folder_2):
        messagebox.showerror("Missing Folders", "Please select all folders.")
        return

    # Purge target folders
    purge_folder(target_folder_1)
    purge_folder(target_folder_2) 

    for subdir, _, files in os.walk(main_folder):
        relative_path = os.path.relpath(subdir, main_folder)
        target_subdir_1 = os.path.join(target_folder_1, relative_path)
        target_subdir_2 = os.path.join(target_folder_2, relative_path)
        
        if not os.path.exists(target_subdir_1):
            os.makedirs(target_subdir_1)
        if not os.path.exists(target_subdir_2):
            os.makedirs(target_subdir_2)
        
        random.shuffle(files)
        split_point = int(len(files) * (percentage / 100))
        files_for_folder_2 = files[:split_point]
        files_for_folder_1 = files[split_point:]
        
        for file in files_for_folder_1:
            shutil.move(os.path.join(subdir, file), os.path.join(target_subdir_1, file))
        
        for file in files_for_folder_2:
            shutil.move(os.path.join(subdir, file), os.path.join(target_subdir_2, file))

    messagebox.showinfo("Success", "Files have been distributed successfully.")

# Setup Tkinter window
root = tk.Tk()
root.title("File Distribution")

# Main Folder
tk.Label(root, text="Main Folder:").grid(row=0, column=0, padx=10, pady=5)
main_folder_entry = tk.Entry(root, width=50)
main_folder_entry.grid(row=0, column=1, padx=10, pady=5)
tk.Button(root, text="Browse", command=select_main_folder).grid(row=0, column=2, padx=10, pady=5)

# Target Folder 1
tk.Label(root, text="Target Folder 1:").grid(row=1, column=0, padx=10, pady=5)
target_folder_1_entry = tk.Entry(root, width=50)
target_folder_1_entry.grid(row=1, column=1, padx=10, pady=5)
tk.Button(root, text="Browse", command=select_target_folder_1).grid(row=1, column=2, padx=10, pady=5)

# Target Folder 2
tk.Label(root, text="Target Folder 2:").grid(row=2, column=0, padx=10, pady=5)
target_folder_2_entry = tk.Entry(root, width=50)
target_folder_2_entry.grid(row=2, column=1, padx=10, pady=5)
tk.Button(root, text="Browse", command=select_target_folder_2).grid(row=2, column=2, padx=10, pady=5)

# Percentage Split
tk.Label(root, text="Percentage for Folder 2:").grid(row=3, column=0, padx=10, pady=5)
percentage_entry = tk.Entry(root, width=10)
percentage_entry.grid(row=3, column=1, padx=10, pady=5)

# Start Button
tk.Button(root, text="Start", command=start_distribution).grid(row=4, column=1, pady=20)

root.mainloop()
