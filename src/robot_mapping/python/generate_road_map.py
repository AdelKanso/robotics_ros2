
from PIL import Image
import numpy as np
import yaml
import os

def generate_centered_pgm_and_yaml(road_png_path, output_prefix, road_color=(0,128,255), resolution=0.0197, margin_pixels=10):
    img = Image.open(road_png_path).convert("RGB")
    arr = np.array(img)

    r_target, g_target, b_target = road_color

    mask = ((arr[:,:,0] == r_target) & (arr[:,:,1] == g_target) & (arr[:,:,2] == b_target))

    if not np.any(mask):
        raise ValueError("No road pixels found with this color!")

    ys, xs = np.where(mask)
    min_x, max_x = xs.min(), xs.max()
    min_y, max_y = ys.min(), ys.max()

    min_x = max(min_x - margin_pixels, 0)
    max_x = min(max_x + margin_pixels, arr.shape[1]-1)
    min_y = max(min_y - margin_pixels, 0)
    max_y = min(max_y + margin_pixels, arr.shape[0]-1)

    cropped_arr = arr[min_y:max_y+1, min_x:max_x+1]

    binary_map = np.where(
        (cropped_arr[:,:,0] == r_target) & (cropped_arr[:,:,1] == g_target) & (cropped_arr[:,:,2] == b_target),
        255, 0
    ).astype(np.uint8)

    pgm_img = Image.fromarray(binary_map, mode='L')

    
    home_dir = os.path.expanduser("~")
    pgm_file = os.path.join(home_dir, output_prefix + ".pgm")
    pgm_img.save(pgm_file)
    print(f"Saved PGM map to: {pgm_file}")

    
    width_px, height_px = binary_map.shape[1], binary_map.shape[0]
    origin_x = - (width_px / 2) * resolution
    origin_y = - (height_px / 2) * resolution

    yaml_file = os.path.join(home_dir, output_prefix + ".yaml")
    map_yaml = {
        'image': os.path.basename(pgm_file),
        'resolution': resolution,
        'origin': [origin_x, origin_y, 0.0],
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
        'negate': 0
    }

    with open(yaml_file, 'w') as f:
        yaml.dump(map_yaml, f, default_flow_style=False)
    print(f"Saved YAML map to: {yaml_file}")


if __name__ == "__main__":
    road_png_path = "road_only.png"
    output_prefix = "road_map"
    resolution = 0.0400  
    generate_centered_pgm_and_yaml(road_png_path, output_prefix, road_color=(0,128,255), resolution=resolution, margin_pixels=10)