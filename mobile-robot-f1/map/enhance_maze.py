import cv2
import os
import numpy as np

# Lay duong dan + doc anh
script_dir = os.path.dirname(os.path.abspath(__file__))
image_path = os.path.join(script_dir, "m4.png")
image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# nhi phan hoa lam ro mau tuong + nen
_, binary_image = cv2.threshold(image, 128, 255, cv2.THRESH_BINARY)

# tao file dau ra tu dong tang
idx = 1
while os.path.exists(os.path.join(script_dir, f"m{idx}.png")):
    idx += 1
output_path = os.path.join(script_dir, f"m{idx}.png")

# Luu anh da xu ly
cv2.imwrite(output_path, binary_image)
print(f"Image saved as {output_path}")
