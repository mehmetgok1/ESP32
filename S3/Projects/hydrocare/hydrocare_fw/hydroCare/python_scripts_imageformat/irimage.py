import numpy as np
import cv2
import os

# Configuration
width, height = 16, 12  # Your sensor resolution
input_dir = "/media/deso/BOOT/20260326_13194/thermal_camera/"
output_dir = "./thermal_colored_16bit"
os.makedirs(output_dir, exist_ok=True)

print(f"Processing 16-bit IR files...")

for filename in os.listdir(input_dir):
    if filename.endswith(".ir16"):
        full_path = os.path.join(input_dir, filename)
        
        # 1. Load raw data as 16-bit (unsigned short)
        # Note: If your sensor data is signed (can be negative), use np.int16
        raw_data = np.fromfile(full_path, dtype=np.uint16)
        
        # 2. Reshape (16x12)
        if raw_data.size != width * height:
            print(f"Skipping {filename}: size mismatch ({raw_data.size} elements)")
            continue
            
        thermal_img = raw_data.reshape((height, width))

        # 3. Normalization (Even more critical for 16-bit!)
        # This converts the 0-65535 range down to 0-255 for the colormap
        norm_img = cv2.normalize(thermal_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        # 4. Upscale (30x)
        # We use INTER_CUBIC to get that smooth "blooming" heat effect
        large_img = cv2.resize(norm_img, (width*30, height*30), interpolation=cv2.INTER_CUBIC)

        # 5. Apply Colormap
        # COLORMAP_MAGMA or COLORMAP_INFERNO often looks better for 16-bit data
        color_img = cv2.applyColorMap(large_img, cv2.COLORMAP_JET)

        # 6. Save
        output_filename = filename.replace(".ir16", ".png")
        cv2.imwrite(os.path.join(output_dir, output_filename), color_img)

print(f"Success! 16-bit thermal images are in: {output_dir}")
