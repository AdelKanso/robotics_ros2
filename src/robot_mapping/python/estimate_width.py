#!/usr/bin/env python3
from PIL import Image
import numpy as np

def estimate_width(image_path, road_color=(0,128,255), tolerance=20):
    img = Image.open(image_path).convert("RGB")
    arr = np.array(img)

    r, g, b = road_color

    dist = np.sqrt(
        (arr[:,:,0] - r)**2 +
        (arr[:,:,1] - g)**2 +
        (arr[:,:,2] - b)**2
    )

    mask = dist < tolerance

    if not np.any(mask):
        print("No road pixels found")
        return 0

    # Extract coordinates
    ys, xs = np.where(mask)

    width_px = xs.max() - xs.min()

    print("Detected road width (pixels):", width_px)
    return width_px


if __name__ == "__main__":
    estimate_width("road_only.png", (0,128,255), tolerance=20)
