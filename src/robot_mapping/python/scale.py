from PIL import Image

img = Image.open("road_only.png")
scale_factor = 3
new_size = (int(img.width * scale_factor), (img.height * scale_factor))
img_resized = img.resize(new_size, resample=Image.NEAREST)
img_resized.save("road_only.png")
