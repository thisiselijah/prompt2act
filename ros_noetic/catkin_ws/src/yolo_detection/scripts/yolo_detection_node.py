#!/usr/bin/python3
"""
Enhanced YOLO Detection Node for Robot Vision

This node provides advanced object detection capabilities with standardized output format
for seamless integration with behavior tree systems and robotic control.

Publishers:
    /yolo_detected_targets (std_msgs/String): Enhanced JSON detection data

Enhanced JSON Format:
{
    "detections": [
        {
            "cx": int,              # Center X pixel coordinate
            "cy": int,              # Center Y pixel coordinate  
            "pts": [[x,y], ...],    # Bounding box corner points
            "roll": float,          # Object orientation in radians
            "label": str,           # Original YOLO label (e.g., "blue_block")
            "class": str,           # Standardized object class (e.g., "cube", "sphere")
            "color": str,           # Standardized color name (e.g., "blue", "red")
            "x": float,             # Robot coordinate X (meters)
            "y": float,             # Robot coordinate Y (meters)
            "z": float,             # Robot coordinate Z (meters, default table height)
            "confidence": float     # YOLO detection confidence score (0.0-1.0)
        }
    ],
    "white_region": {            # Designated work area (if detected)
        "x": float,              # Work area center X coordinate
        "y": float               # Work area center Y coordinate
    },
    "timestamp": float,          # ROS timestamp
    "frame_count": int           # Frame sequence number
}

Supported Objects:
- blue_block, red_block, blue_cube, red_cube -> class: "cube" (blue/red colors only)
- white_region -> Special work area marker

Optimized Detection:
- Focuses on 2 color cubes (blue, red) + work area detection
- Improved performance by filtering unnecessary object types
- Streamlined object mapping for faster processing

Key Features:
- ArUco marker-based coordinate transformation
- Perspective correction for accurate positioning
- Standardized object classification for behavior trees
- Continuous publishing for real-time robot control
- Enhanced debugging and monitoring capabilities
"""
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
package_path = rospack.get_path('yolo_detection')
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
TABLE_WIDTH_CM = 30
TABLE_HEIGHT_CM = 29

# --- 座標轉換 ---
def camera_to_robot_coords(wx, wy):
    origin_x = 0.2
    origin_y = -0.1
    real_x_cm = wx * TABLE_WIDTH_CM
    real_y_cm = wy * TABLE_HEIGHT_CM
    return origin_x + real_x_cm / 100.0, origin_y + real_y_cm / 100.0

# --- ROS Publisher ---
bridge = CvBridge()
pub_target = rospy.Publisher('/yolo_detected_targets', String, queue_size=10)

# --- 狀態旗標 ---
white_region_detected = False
white_region_coords = None

# --- 影像處理 ---
def image_callback(msg):
    global white_region_detected, white_region_coords

    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 偵測 ArUco
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    if ids is None or len(ids) < 4:
        return

    id_list = ids.flatten()
    aruco_points = {}
    for i, corner in enumerate(corners):
        aruco_points[id_list[i]] = corner[0].mean(axis=0)

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

            # --- 新增 white_region 一次性偵測 ---
            if label_name == "white_region" and not white_region_detected:
                pts = box.xyxyxyxy.cpu().numpy().astype(int).reshape((-1, 2))
                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))

                world_pos = cv2.perspectiveTransform(np.array([[[cx, cy]]], dtype='float32'), M)
                wx, wy = world_pos[0][0]

                if 0 <= wx <= 1 and 0 <= wy <= 1:
                    robot_x, robot_y = camera_to_robot_coords(wx, wy)
                    white_region_coords = {"x": robot_x, "y": robot_y}
                    white_region_detected = True
                    rospy.loginfo(f"White region detected at {white_region_coords}")

                # 不要把 white_region 加進 detections_list，因為它只是工作區塊
                continue

            # --- 偵測指定物體類型：兩色立方體和白色區域 ---
            # Optimized mapping for only required objects: two color cubes + white region
            object_mapping = {
                "blue_block": {"class": "cube", "color": "blue"},
                "red_block": {"class": "cube", "color": "red"},
                "blue_cube": {"class": "cube", "color": "blue"},
                "red_cube": {"class": "cube", "color": "red"}
            }
            
            if label_name not in object_mapping:
                # Skip detection of unsupported objects
                continue
                
            obj_info = object_mapping[label_name]

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
                    "class": obj_info["class"],  # Standardized class name
                    "color": obj_info["color"],  # Standardized color name
                    "x": robot_x,
                    "y": robot_y,
                    "z": 0.2,  # Default table height
                    "confidence": float(box.conf[0]) if hasattr(box, 'conf') else 1.0
                })

    # 發送 JSON (always publish, even if empty)
    output = {
        "detections": detections_list,
        "white_region": white_region_coords if white_region_coords else None,
        "timestamp": rospy.Time.now().to_sec(),
        "frame_count": getattr(image_callback, 'frame_count', 0)
    }
    
    # Increment frame counter
    if not hasattr(image_callback, 'frame_count'):
        image_callback.frame_count = 0
    image_callback.frame_count += 1
    
    # Publish detection data
    pub_target.publish(json.dumps(output))
    
    # Log detection summary (focused on required objects only)
    if detections_list:
        detection_summary = [f"{d['color']} {d['class']}" for d in detections_list]
        rospy.loginfo(f"📦 Detected {len(detections_list)} cubes: {detection_summary}")
    
    # Log white region status
    if white_region_coords:
        rospy.loginfo_once(f"📍 Work area (white region) located at ({white_region_coords['x']:.3f}, {white_region_coords['y']:.3f})")

# --- 訂閱攝影機 ---
rospy.Subscriber('/web_camera/image_raw', Image, image_callback)

# --- ROS 主循環 ---
rospy.spin()
