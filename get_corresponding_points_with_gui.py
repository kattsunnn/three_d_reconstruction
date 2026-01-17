import sys
import json
from img_utils.img_utils import load_imgs, get_img_points_with_gui


img_path = sys.argv[1]
window_scale = float(sys.argv[2])
output_file_path = sys.argv[3]

imgs = load_imgs(img_path)

save_data = []
for img in imgs:
    corr_points, _ = get_img_points_with_gui(img, window_scale)
    save_data.append(corr_points.tolist())

with open(output_file_path, 'w', encoding='utf-8') as f:
    json.dump(save_data, f, indent=4)

