import cv2
import cv2.aruco as aruco
import os

# 建立儲存資料夾
output_dir = "Aruco"
os.makedirs(output_dir, exist_ok=True)

# 使用新版 getPredefinedDictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# 產生 ID 0~3 的標記
for id in range(4):
    img = aruco.generateImageMarker(aruco_dict, id, 700)
    filename = os.path.join(output_dir, f"aruco_id_{id}.png")
    cv2.imwrite(filename, img)

print("Aruco 標記已成功產生！")
