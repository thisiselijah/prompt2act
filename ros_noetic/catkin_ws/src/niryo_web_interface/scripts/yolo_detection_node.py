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

# --- ROS 初始化 (ROS initialization) ---
rospack = rospkg.RosPack()
package_path = rospack.get_path('niryo_web_interface')   # 取得 ROS 套件目錄 (Get ROS package path)
model_path = os.path.join(package_path, 'model', 'best.pt')  # YOLO 模型檔案路徑 (YOLO model path)

rospy.init_node('yolo_detection_node', anonymous=True)  # 初始化 ROS 節點 (Initialize ROS node)

# --- 載入 YOLO 模型 (Load YOLO model) ---
try:
    model = YOLO(model_path)  # 使用 Ultralytics YOLO 載入訓練好的權重 (Load trained YOLO model)
except Exception as e:
    rospy.logerr(f"Failed to load YOLO model: {e}")

# --- ArUco 標記相關設定 (ArUco marker setup, for table corners) ---
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)  # 預設 ArUco 字典
aruco_params = cv2.aruco.DetectorParameters()  # 偵測參數

# 桌面實際尺寸（單位：公分）(Real table dimensions in cm)
TABLE_WIDTH_CM = 23
TABLE_HEIGHT_CM = 30

# --- 座標轉換函式 (Coordinate transformation: camera → robot) ---
def camera_to_robot_coords(wx, wy):
    """
    將歸一化桌面座標 (0~1) 轉換成 Niryo 機械臂的世界座標（單位：公尺）
    Convert normalized table coordinates (0~1) to Niryo robot world coordinates (meters)
    """
    origin_x = 0.2     # Niryo 桌面左上角 X 偏移 (m)
    origin_y = -0.1    # Niryo 桌面左上角 Y 偏移 (m)
    real_x_cm = wx * TABLE_WIDTH_CM
    real_y_cm = wy * TABLE_HEIGHT_CM
    return origin_x + real_x_cm / 100.0, origin_y + real_y_cm / 100.0

# --- ROS Publisher 初始化 (Initialize ROS publishers) ---
bridge = CvBridge()  # 影像格式轉換 (ROS <-> OpenCV)
pub_target = rospy.Publisher('/yolo_detected_target', String, queue_size=10)  # 發送 YOLO 偵測結果 (Publish YOLO detection results)
pub_image = rospy.Publisher('/yolo_annotated_image', Image, queue_size=1)    # 發送帶標註的影像 (Publish annotated image)

# --- 處理影像的 Callback 函式 (Callback to process incoming camera frames) ---
def image_callback(msg):
    # 1. 轉換 ROS 影像訊息到 OpenCV 格式
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 2. 偵測 ArUco 標記 (Detect ArUco markers)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    # 沒找到足夠標記就跳過 (Need at least 4 markers)
    if ids is None or len(ids) < 4:
        return

    # 3. 建立 ArUco 標記座標字典
    id_list = ids.flatten()
    aruco_points = {}
    for i, corner in enumerate(corners):
        aruco_points[id_list[i]] = corner[0].mean(axis=0)  # 取每個標記的中心點

    # 4. 根據 ArUco ID 位置順序重排並計算透視轉換矩陣
    if all(k in aruco_points for k in [0, 1, 2, 3]):
        src_pts = np.array([
            aruco_points[3],  # 左上
            aruco_points[2],  # 右上
            aruco_points[1],  # 右下
            aruco_points[0],  # 左下
        ], dtype="float32")

        dst_pts = np.array([
            [0, 0],  # 左上
            [1, 0],  # 右上
            [1, 1],  # 右下
            [0, 1],  # 左下
        ], dtype="float32")

        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    else:
        return

    # 5. 執行 YOLO 偵測 (Run YOLO detection)
    results = model(frame, verbose=False)[0]
    detected_labels = []  # 收集當前偵測到的標籤 (Collect detected labels)

    # 6. 處理偵測到的旋轉框 (Process oriented bounding boxes)
    if results and results.obb is not None and len(results.obb) > 0:
        for box in results.obb:
            cls_id = int(box.cls[0])
            label_name = model.names[cls_id].lower()

            # 只保留藍色與紅色方塊 (Filter only blue_block / red_block)
            if label_name not in ["blue_block", "red_block"]:
                continue

            # 擷取 OBB 四個角座標 (Get OBB vertices)
            pts = box.xyxyxyxy.cpu().numpy().astype(int).reshape((-1, 2))
            cx = int(np.mean(pts[:, 0]))  # 中心點 X
            cy = int(np.mean(pts[:, 1]))  # 中心點 Y

            # 計算旋轉角度 roll（由 X 軸指向的邊決定）
            dx = pts[1][0] - pts[0][0]
            dy = pts[1][1] - pts[0][1]
            roll_rad = math.atan2(dy, dx)

            # 將畫面座標轉換為歸一化桌面座標 (Map image coords to normalized table coords)
            world_pos = cv2.perspectiveTransform(np.array([[[cx, cy]]], dtype='float32'), M)
            wx, wy = world_pos[0][0]

            # 檢查是否在桌面範圍內 (Check if within table bounds)
            if 0 <= wx <= 1 and 0 <= wy <= 1:
                # 轉成機械臂世界座標
                robot_x, robot_y = camera_to_robot_coords(wx, wy)

                # 準備並發送 JSON 格式的偵測結果
                target_data = {
                    "x": robot_x,
                    "y": robot_y,
                    "roll": roll_rad,
                    "label": label_name
                }
                pub_target.publish(json.dumps(target_data))

                # 畫出 OBB 與中心點 (Draw OBB and center point)
                color = (255, 0, 0) if label_name == "blue_block" else (0, 0, 255)
                cv2.polylines(frame, [pts.reshape(-1, 1, 2)], isClosed=True, color=color, thickness=2)
                cv2.circle(frame, (cx, cy), 5, color, -1)
                cv2.putText(frame, label_name, (cx, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                detected_labels.append(label_name)

    # 7. 在控制台列出目前偵測到的方塊 (Log detected blocks)
    if detected_labels:
        rospy.loginfo(f"Detected blocks: {', '.join(detected_labels)}")

    # 8. 發佈帶標註的影像 (Publish annotated frame)
    pub_image.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

# --- ROS 訂閱攝影機影像 (Subscribe to camera image topic) ---
rospy.Subscriber('/web_camera/image_raw', Image, image_callback)

# --- 進入 ROS 事件循環 (Spin to keep node alive) ---
rospy.spin()
