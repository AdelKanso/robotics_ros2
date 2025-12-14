
import os
import csv
import yaml
import numpy as np
from PIL import Image, ImageDraw


def load_traj   (csv_path):
    x_coords, y_coords, yaw_angles = [], [], []

    with open(csv_path, "r") as f:
        reader = csv.reader(f)
        rows = list(reader)

    if not rows:
        return x_coords, y_coords, yaw_angles

    
    header = [h.strip().lower() for h in rows[0]]
    has_header = any(h in header for h in ["x", "y", "yaw"])

    if has_header:
        col_idx = {name: i for i, name in enumerate(header)}

        def find_index(*candidates):
            for c in candidates:
                if c in col_idx:
                    return col_idx[c]
            return None

        ix = find_index("x")
        iy = find_index("y")

        for r in rows[1:]:
            if len(r) < 2: continue
            try:
                x = float(r[ix]) if ix is not None else float(r[0])
                y = float(r[iy]) if iy is not None else float(r[1])
                x_coords.append(x)
                y_coords.append(y)
            except ValueError:
                continue
    else:
        for r in rows:
            if len(r) < 2: continue
            try:
                x_coords.append(float(r[0]))
                y_coords.append(float(r[1]))
            except ValueError:
                continue

    return x_coords, y_coords, []


def draw(yaml_file_path, csv_path, output_file="trajectory_map.png", line_color=(255, 0, 255), line_width=3):
    with open(yaml_file_path, "r") as f:
        map_meta_data = yaml.safe_load(f)

    resolution = float(map_meta_data["resolution"])
    origin = map_meta_data["origin"]
    img_file = map_meta_data["image"]

    if not os.path.isabs(img_file):
        img_file = os.path.join(os.path.dirname(yaml_file_path), img_file)

    img = Image.open(img_file)
    height, width = np.array(img).shape[:2]

    xs, ys, _ = load_traj   (csv_path)

    rgb_image = img.convert("RGB")
    draw = ImageDraw.Draw(rgb_image)

    pixel_coords = []
    for x, y in zip(xs, ys):
        origin_x, origin_y, _ = origin
        newX=(x - origin_x)
        px = int(newX / resolution)
        newY=(y - origin_y)
        py = int(height - newY / resolution)
        if 0 <= px < width and 0 <= py < height:
            pixel_coords.append((px, py))

    if len(pixel_coords) >= 2:
        draw.line(pixel_coords, fill=line_color, width=line_width)

    rgb_image.save(output_file)


def run():
    draw("/home/masters/map.yaml", "/home/masters/robot_xy_yaw.csv", output_file="map_traj_new.png", line_color=(0, 128, 255), line_width=3)

if __name__ == "__main__":
    run()
