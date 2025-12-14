#!/usr/bin/env python3
from PIL import Image

def extract_road(input_file, output_file, line_color=(0, 128, 255), threshold=30):
    img = Image.open(input_file).convert("RGB")
    width, height = img.size

    road_img = Image.new("RGB", (width, height), (255, 255, 255))
    pixels = img.load()
    road_pixels = road_img.load()

    r_target, g_target, b_target = line_color

    for x in range(width):
        for y in range(height):
            r, g, b = pixels[x, y]
            if abs(r - r_target) <= threshold and abs(g - g_target) <= threshold and abs(b - b_target) <= threshold:
                road_pixels[x, y] = (r, g, b)
            else:
                road_pixels[x, y] = (0, 0, 0)

    road_img.save(output_file)
    print(f"Saved extracted road image to {output_file}")


if __name__ == "__main__":
    extract_road("map_traj_new.png", "road_only.png", line_color=(0, 128, 255), threshold=30)
