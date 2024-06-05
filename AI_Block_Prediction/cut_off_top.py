import os
from PIL import Image

def crop_image(image_path):
    with Image.open(image_path) as img:
        width, height = img.size
        if height > 250:
            area = (0, 250, width, height)
            cropped_img = img.crop(area)
            cropped_img.save(image_path)

def process_directory(directory):
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(('.png', '.jpg', '.jpeg')):
                image_path = os.path.join(root, file)
                crop_image(image_path)

# Replace 'your_directory' with the path to the directory you want to process
process_directory(r"C:\Users\felix\Downloads\Grouped + All - Kopie")