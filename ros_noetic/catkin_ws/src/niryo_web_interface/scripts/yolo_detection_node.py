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
package_path = rospack.get_path('niryo_web_interface')   # 取得 ROS 專案目錄
model_path = os.path.join(package_path, 'model', 'best.pt')  # YOLO 模型路徑

rospy.init_node('yolo_detection_node', anonymous=True)

# --- 載入 YOLO 模型 ---
try:
    model = YOLO(model_path) 
except Exception as e:
    rospy.logerr(f"Failed to load YOLO model: {e}")

# --- ArUco 相關設定（用於標記桌面四角）---
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()


# 桌面尺寸（單位：公分）
TABLE_WIDTH_CM = 23
TABLE_HEIGHT_CM = 30

# --- 座標轉換：將桌面單位座標轉成 Niryo 機械臂世界座標（公尺） ---
def camera_to_robot_coords(wx, wy):
    origin_x = 0.2     # Niryo 座標系中，桌面左上角 x 偏移（公尺）
    origin_y = -0.1    # Niryo 座標系中，桌面左上角 y 偏移（公尺）
    real_x_cm = wx * TABLE_WIDTH_CM
    real_y_cm = wy * TABLE_HEIGHT_CM
    return origin_x + real_x_cm / 100.0, origin_y + real_y_cm / 100.0

bridge = CvBridge()
pub_target = rospy.Publisher('/yolo_detected_target', String, queue_size=10)
pub_image = rospy.Publisher('/yolo_annotated_image', Image, queue_size=1)

def image_callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    if ids is None or len(ids) < 4:
        return

    id_list = ids.flatten()
    aruco_points = {}
    for i, corner in enumerate(corners):
        aruco_points[id_list[i]] = corner[0].mean(axis=0)

    if not all(k in aruco_points for k in [0, 1, 2, 3]):
        return

    src_pts = np.array([
        aruco_points[3], aruco_points[2], aruco_points[1], aruco_points[0]
    ], dtype="float32")
    dst_pts = np.array([[0,0],[1,0],[1,1],[0,1]], dtype="float32")
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)

    results = model(frame, verbose=False)[0]
    detected_labels = []  # 收集當前偵測的標籤

    if results and results.obb is not None and len(results.obb) > 0:
        for box in results.obb:
            cls_id = int(box.cls[0])
            label_name = model.names[cls_id].lower()

            # 只保留藍色與紅色方塊
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

                target_data = {
                    "x": robot_x,
                    "y": robot_y,
                    "roll": roll_rad,
                    "label": label_name
                }
                pub_target.publish(json.dumps(target_data))

                # 畫出 OBB 框與中心點
                color = (255, 0, 0) if label_name == "blue_block" else (0, 0, 255)
                cv2.polylines(frame, [pts.reshape(-1, 1, 2)], isClosed=True, color=color, thickness=2)
                cv2.circle(frame, (cx, cy), 5, color, -1)
                cv2.putText(frame, label_name, (cx, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                detected_labels.append(label_name)

    # 在控制台印出當前偵測到的藍色與紅色方塊
    if detected_labels:
        rospy.loginfo(f"Detected blocks: {', '.join(detected_labels)}")

    # 發佈帶標註的影像
    pub_image.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

rospy.Subscriber('/camera/image_raw', Image, image_callback)
rospy.spin()
