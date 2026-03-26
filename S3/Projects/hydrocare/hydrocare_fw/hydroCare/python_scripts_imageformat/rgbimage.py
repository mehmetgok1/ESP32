import numpy as np
import cv2
import os

# Configuration
width, height = 64, 64
# Path to your SD card color camera folder
input_dir = "/media/deso/BOOT/20260326_13194/color_camera/"
output_dir = "./color_processed"
os.makedirs(output_dir, exist_ok=True)

print(f"Processing RGB565 files from {input_dir}...")

for filename in os.listdir(input_dir):
    if filename.endswith(".rgb565"):
        full_path = os.path.join(input_dir, filename)
        
        # 1. Read the file as raw bytes (just like ffmpeg does)
        # 64x64 pixels * 2 bytes per pixel = 8192 bytes
        raw_data = np.fromfile(full_path, dtype=np.uint8)
        
        if raw_data.size != (width * height * 2):
            print(f"Skipping {filename}: Incorrect file size.")
            continue

        # 2. Reshape to (Height, Width, 2 Channels)
        # We use 2 channels because RGB565 is 2 bytes per pixel
        img_raw = raw_data.reshape((height, width, 2))

        # 3. Convert to BGR (Standard PC Color)
        # 'rgb565le' in ffmpeg means Little Endian. 
        # In OpenCV, COLOR_BGR5652BGR matches that logic.
        img_bgr = cv2.cvtColor(img_raw, cv2.COLOR_BGR5652BGR)

        # 4. Upscale to 256x256 for better visualization
        # INTER_NEAREST keeps the pixels sharp (good for debugging sensor data)
        large_img = cv2.resize(img_bgr, (256, 256), interpolation=cv2.INTER_NEAREST)

        # 5. Save as PNG
        output_filename = filename.replace(".rgb565", ".png")
        cv2.imwrite(os.path.join(output_dir, output_filename), large_img)

print(f"Done! Check the '{output_dir}' folder for your 256x256 images.")
