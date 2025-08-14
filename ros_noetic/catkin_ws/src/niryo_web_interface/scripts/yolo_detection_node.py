#!/usr/bin/python3
import rospkg
import os
import rospy
import cv2
import numpy as np
import math
import json
from ultralytics import YOLO
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# --- ROS 初始化 ---
rospack = rospkg.RosPack()
package_path = rospack.get_path('niryo_web_interface')
model_path = os.path.join(package_path, 'model', 'best.pt')

rospy.init_node('yolo_detection_node', anonymous=True)

# --- 載入 YOLO 模型 ---
try:
    model = YOLO(model_path)
except Exception as e:
    rospy.logerr(f"Failed to load YOLO model: {e}")

# --- ArUco 標記設定 ---
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

# 桌面尺寸 (cm)
TABLE_WIDTH_CM = 23
TABLE_HEIGHT_CM = 30

# --- 座標轉換 ---
def camera_to_robot_coords(wx, wy):
    origin_x = 0.2
    origin_y = -0.1
    real_x_cm = wx * TABLE_WIDTH_CM
    real_y_cm = wy * TABLE_HEIGHT_CM
    return origin_x + real_x_cm / 100.0, origin_y + real_y_cm / 100.0

# --- ROS Publisher ---
bridge = CvBridge()
pub_target = rospy.Publisher('/yolo_detected_targets', String, queue_size=10)  # 改成 targets，傳回多個物件

# --- 影像處理 ---
def image_callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 偵測 ArUco
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    if ids is None or len(ids) < 4:
        return

    # 取得中心點
    id_list = ids.flatten()
    aruco_points = {}
    for i, corner in enumerate(corners):
        aruco_points[id_list[i]] = corner[0].mean(axis=0)

    # 建立透視轉換矩陣
    if all(k in aruco_points for k in [0, 1, 2, 3]):
        src_pts = np.array([
            aruco_points[3],  # 左上
            aruco_points[2],  # 右上
            aruco_points[1],  # 右下
            aruco_points[0],  # 左下
        ], dtype="float32")

        dst_pts = np.array([
            [0, 0],
            [1, 0],
            [1, 1],
            [0, 1],
        ], dtype="float32")

        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    else:
        return

    # YOLO 偵測
    results = model(frame, verbose=False)[0]
    detections_list = []

    if results and results.obb is not None and len(results.obb) > 0:
        for box in results.obb:
            cls_id = int(box.cls[0])
            label_name = model.names[cls_id].lower()

            if label_name not in ["blue_block", "red_block"]:
                continue

            pts = box.xyxyxyxy.cpu().numpy().astype(int).reshape((-1, 2))
            cx = int(np.mean(pts[:, 0]))
            cy = int(np.mean(pts[:, 1]))

            dx = pts[1][0] - pts[0][0]
            dy = pts[1][1] - pts[0][1]
            roll_rad = math.atan2(dy, dx)

            world_pos = cv2.perspectiveTransform(np.array([[[cx, cy]]], dtype='float32'), M)
            wx, wy = world_pos[0][0]

            if 0 <= wx <= 1 and 0 <= wy <= 1:
                robot_x, robot_y = camera_to_robot_coords(wx, wy)

                detections_list.append({
                    "cx": cx,
                    "cy": cy,
                    "pts": pts.tolist(),
                    "roll": roll_rad,
                    "label": label_name,
                    "x": robot_x,
                    "y": robot_y
                })

    # 發送 JSON
    if detections_list:
        pub_target.publish(json.dumps(detections_list))

# --- 訂閱攝影機 ---
rospy.Subscriber('/web_camera/image_raw', Image, image_callback)

# --- ROS 主循環 ---
rospy.spin()
