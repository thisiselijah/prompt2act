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
import json
import onnxruntime as ort
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# --- ROS 初始化 ---
rospack = rospkg.RosPack()
package_path = rospack.get_path('yolo_detection')
model_path = os.path.join(package_path, 'model', 'best.onnx')

rospy.init_node('yolo_detection_node', anonymous=True)

# --- ArUco 標記設定 ---
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

# 桌面尺寸 (cm)
TABLE_WIDTH_CM = 30
TABLE_HEIGHT_CM = 30
# --- 校準參數 ---
# 這些值應該通過校準過程來設定
calibrated_origin_x = 0.06  # 米
calibrated_origin_y = 0.05 # 米
origin_calibrated = False  # 標記是否已校準

# --- ONNX 推論設定 ---
ONNX_INPUT_SIZE = 640
ONNX_CONF_THRESHOLD = 0.1  # 降低信心閾值以增加偵測
ONNX_IOU_THRESHOLD = 0.45
DEFAULT_CLASS_NAMES = ["blue_block", "red_block", "blue_cube", "red_cube", "white_region"]


def load_class_names(model_file_path):
    """嘗試從 .names 檔載入類別名稱，失敗時使用預設類別。"""
    names_path = os.path.splitext(model_file_path)[0] + ".names"
    if os.path.exists(names_path):
        try:
            with open(names_path, "r", encoding="utf-8") as names_file:
                names = [line.strip() for line in names_file if line.strip()]
                if names:
                    return [name.lower() for name in names]
        except Exception as error:
            rospy.logwarn(f"⚠️ 無法載入類別名稱檔案 {names_path}: {error}")
    return [name.lower() for name in DEFAULT_CLASS_NAMES]


CLASS_NAMES = load_class_names(model_path)
rospy.loginfo(f"📋 載入類別名稱: {CLASS_NAMES}")

OBJECT_MAPPING = {
    "blue_block": {"class": "cube", "color": "blue"},
    "red_block": {"class": "cube", "color": "red"},
    "blue_cube": {"class": "cube", "color": "blue"},
    "red_cube": {"class": "cube", "color": "red"},
    "white_region": {"class": "white_region", "color": "white"},
}


def letterbox(image, new_shape=(ONNX_INPUT_SIZE, ONNX_INPUT_SIZE), color=(114, 114, 114)):
    """以保持比例方式縮放圖片，並補齊邊框。"""
    shape = image.shape[:2]  # (height, width)
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    ratio = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    new_unpad = (int(round(shape[1] * ratio)), int(round(shape[0] * ratio)))
    dw = (new_shape[1] - new_unpad[0]) / 2
    dh = (new_shape[0] - new_unpad[1]) / 2

    if shape[::-1] != new_unpad:
        image = cv2.resize(image, new_unpad, interpolation=cv2.INTER_LINEAR)

    top = int(round(dh - 0.1))
    bottom = int(round(dh + 0.1))
    left = int(round(dw - 0.1))
    right = int(round(dw + 0.1))

    image = cv2.copyMakeBorder(image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return image, ratio, (dw, dh)


def preprocess_image(image):
    """將輸入影像轉換成 ONNX 模型需要的格式。"""
    processed, ratio, dwdh = letterbox(image, new_shape=(ONNX_INPUT_SIZE, ONNX_INPUT_SIZE))
    processed = cv2.cvtColor(processed, cv2.COLOR_BGR2RGB)
    processed = processed.astype(np.float32) / 255.0
    processed = np.transpose(processed, (2, 0, 1))  # CHW
    processed = np.expand_dims(processed, axis=0)
    return processed, ratio, dwdh


def bbox_iou(box, boxes):
    """計算單一方框與多個方框的 IoU。"""
    if boxes.size == 0:
        return np.array([])

    inter_x1 = np.maximum(box[0], boxes[:, 0])
    inter_y1 = np.maximum(box[1], boxes[:, 1])
    inter_x2 = np.minimum(box[2], boxes[:, 2])
    inter_y2 = np.minimum(box[3], boxes[:, 3])

    inter_w = np.maximum(0.0, inter_x2 - inter_x1)
    inter_h = np.maximum(0.0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h

    box_area = (box[2] - box[0]) * (box[3] - box[1])
    boxes_area = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])

    union_area = box_area + boxes_area - inter_area
    union_area = np.maximum(union_area, 1e-6)
    return inter_area / union_area


def non_max_suppression(boxes, scores, iou_threshold):
    """執行簡單的 NMS。"""
    if boxes.shape[0] == 0:
        return []

    order = scores.argsort()[::-1]
    keep = []

    while order.size > 0:
        i = order[0]
        keep.append(int(i))
        if order.size == 1:
            break

        remaining = order[1:]
        ious = bbox_iou(boxes[i], boxes[remaining])
        remaining = remaining[ious <= iou_threshold]
        order = remaining

    return keep


def postprocess_detections(output, original_shape, ratio, dwdh, conf_threshold, iou_threshold):
    """將 ONNX 模型輸出轉換為可讀的偵測結果。支援 OBB 輸出格式。"""
    if output is None:
        return []

    predictions = np.squeeze(output)
    if predictions.ndim == 1:
        predictions = np.expand_dims(predictions, axis=0)

    if predictions.shape[0] <= predictions.shape[1]:
        predictions = predictions.transpose(1, 0)

    # 假設 OBB 格式: [cx, cy, w, h, angle, conf1, conf2, ...]
    boxes = predictions[:, :4]  # cx, cy, w, h
    angles = predictions[:, 4] if predictions.shape[1] > 4 else np.zeros(predictions.shape[0])  # angle
    scores = predictions[:, 5:] if predictions.shape[1] > 5 else predictions[:, 4:]  # 類別分數

    if scores.size == 0:
        return []

    class_ids = np.argmax(scores, axis=1)
    confidences = scores[np.arange(scores.shape[0]), class_ids]

    mask = confidences > conf_threshold
    boxes = boxes[mask]
    angles = angles[mask]
    confidences = confidences[mask]
    class_ids = class_ids[mask]

    if boxes.size == 0:
        return []

    # 轉換為 xyxy 格式
    boxes_xyxy = np.zeros_like(boxes)
    boxes_xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2  # x1
    boxes_xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2  # y1
    boxes_xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2  # x2
    boxes_xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2  # y2

    # 調整邊框座標
    dw, dh = dwdh
    boxes_xyxy[:, [0, 2]] -= dw
    boxes_xyxy[:, [1, 3]] -= dh
    boxes_xyxy[:, [0, 2]] /= ratio
    boxes_xyxy[:, [1, 3]] /= ratio

    height, width = original_shape
    boxes_xyxy[:, [0, 2]] = np.clip(boxes_xyxy[:, [0, 2]], 0, width - 1)
    boxes_xyxy[:, [1, 3]] = np.clip(boxes_xyxy[:, [1, 3]], 0, height - 1)

    keep_indices = non_max_suppression(boxes_xyxy, confidences, iou_threshold)

    detections = []
    for idx in keep_indices:
        detections.append({
            "bbox": boxes_xyxy[idx],
            "score": float(confidences[idx]),
            "class_id": int(class_ids[idx]),
            "angle": float(angles[idx]) if idx < len(angles) else 0.0,
        })

    return detections


onnx_session = None
onnx_input_name = None

try:
    providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
    try:
        onnx_session = ort.InferenceSession(model_path, providers=providers)
    except Exception:
        onnx_session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])

    onnx_input_name = onnx_session.get_inputs()[0].name if onnx_session else None
    if onnx_session:
        rospy.loginfo(f"✅ ONNX 模型載入成功：{model_path}")
    else:
        rospy.logerr(f"❌ 無法建立 ONNX 推論會話：{model_path}")
except FileNotFoundError:
    rospy.logerr(f"❌ 找不到 ONNX 模型檔案：{model_path}")
except Exception as error:
    rospy.logerr(f"❌ 無法載入 ONNX 模型：{error}")


def run_onnx_inference(frame):
    """對影像執行 ONNX 推論並回傳偵測結果。"""
    if onnx_session is None or onnx_input_name is None:
        rospy.logerr_throttle(5.0, "❌ ONNX 推論會話尚未建立，無法進行偵測")
        return []

    input_tensor, ratio, dwdh = preprocess_image(frame)
    try:
        outputs = onnx_session.run(None, {onnx_input_name: input_tensor})
        output_array = outputs[0] if isinstance(outputs, list) else outputs
        # 添加調試日誌
        if not hasattr(run_onnx_inference, 'logged_shape'):
            run_onnx_inference.logged_shape = True
            rospy.loginfo(f"🔍 ONNX 輸出形狀: {output_array.shape}")
    except Exception as error:
        rospy.logerr_throttle(5.0, f"❌ ONNX 推論失敗：{error}")
        return []

    return postprocess_detections(
        output_array,
        frame.shape[:2],
        ratio,
        dwdh,
        ONNX_CONF_THRESHOLD,
        ONNX_IOU_THRESHOLD,
    )

# --- 座標轉換 ---
def camera_to_robot_coords(wx, wy):
    """
    將相機座標 (0-1 歸一化) 轉換為機器人座標 (米)
    
    參數:
    wx, wy: 歸一化的世界座標 (0-1)
    
    返回:
    robot_x, robot_y: 機器人座標 (米)
    """
    # 將歸一化座標轉換為實際尺寸 (厘米)
    real_x_cm = wx * TABLE_WIDTH_CM
    real_y_cm = wy * TABLE_HEIGHT_CM
    
    # 轉換為米並應用校準後的原點偏移
    return calibrated_origin_x + real_x_cm / 100.0, calibrated_origin_y + real_y_cm / 100.0

# --- ROS Publisher ---
bridge = CvBridge()
pub_target = rospy.Publisher('/yolo_detected_targets', String, queue_size=10)

# --- 狀態旗標 ---
white_region_detected = False
white_region_coords = None

# --- 校準函數 ---
def calibrate_aruco_origin(corners, ids, frame, M):
    """
    校準 ArUco 標記原點位置
    找到 ID=3 的標記並計算其在機器人座標系中的實際位置
    
    參數:
    corners: ArUco 標記角點
    ids: 標記 ID 列表
    frame: 當前影像幀
    M: 透視變換矩陣
    
    返回:
    bool: 是否成功校準
    """
    global calibrated_origin_x, calibrated_origin_y, origin_calibrated
    
    if ids is not None and not origin_calibrated:
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == 3:  # 使用 ID=3 作為原點標記
                # 計算標記中心點 (像素座標)
                marker_corners = corners[i][0]
                center_x = int((marker_corners[0][0] + marker_corners[2][0]) / 2)
                center_y = int((marker_corners[0][1] + marker_corners[2][1]) / 2)
                
                # 將像素座標轉換為歸一化座標
                norm_x = center_x / frame.shape[1]
                norm_y = center_y / frame.shape[0]
                
                # 應用透視變換得到世界座標
                world_pos = cv2.perspectiveTransform(
                    np.array([[[norm_x, norm_y]]], dtype='float32'), 
                    np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype='float32')
                )
                wx, wy = world_pos[0][0]
                
                # 計算未校準的機器人座標 (這是標記的當前位置)
                current_robot_x, current_robot_y = camera_to_robot_coords(wx, wy)
                
                # 假設標記 ID=3 應該在機器人座標系的 (0, 0) 位置
                # 因此我們需要將原點調整為負的當前位置
                calibrated_origin_x = +current_robot_x
                calibrated_origin_y = -current_robot_y
                origin_calibrated = True
                
                # 在影像上標記校準點
                cv2.circle(frame, (center_x, center_y), 10, (0, 0, 255), -1)
                cv2.putText(frame, "CALIBRATED ORIGIN (ID=3)", 
                           (center_x + 15, center_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
                rospy.loginfo(f"🎯 ArUco Origin calibrated: Marker at pixel({center_x},{center_y}) -> Robot({current_robot_x:.3f},{current_robot_y:.3f})m")
                rospy.loginfo(f"📐 Origin offset set to: ({calibrated_origin_x:.3f}, {calibrated_origin_y:.3f})m")
                
                return True
    
    return False

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
        # 優化：只在首次成功計算時記錄
        if not hasattr(image_callback, 'transform_logged'):
            image_callback.transform_logged = True
            rospy.loginfo("🔧 Perspective transform matrix computed")
            
            # 校準 ArUco 原點
            calibrate_aruco_origin(corners, ids, frame, M)
    else:
        return
    
    # 準備 ArUco 標記資訊供其他節點使用
    # ✅ 優化：直接傳送計算好的透視變換矩陣 M，避免其他節點重複計算
    aruco_info = {
        "detected": True,
        "transform_matrix": M.tolist() if M is not None else None,  # 3x3 矩陣
        "marker_count": len(aruco_points)
    }

    # 校準原點位置
    calibrate_aruco_origin(corners, ids, frame, M)

    detections_list = []
    onnx_detections = run_onnx_inference(frame)

    # 添加調試日誌
    if onnx_detections:
        rospy.loginfo_throttle(5.0, f"🔍 ONNX 偵測到 {len(onnx_detections)} 個物件")

    for detection in onnx_detections:
        class_id = detection.get("class_id", -1)
        if class_id < 0 or class_id >= len(CLASS_NAMES):
            continue

        label_name = CLASS_NAMES[class_id]
        label_name = label_name.lower()
        confidence = detection.get("score", 0.0)
        bbox = detection.get("bbox")
        if bbox is None or len(bbox) != 4:
            continue

        x1, y1, x2, y2 = bbox.astype(float)
        x1 = max(0, min(frame.shape[1] - 1, x1))
        y1 = max(0, min(frame.shape[0] - 1, y1))
        x2 = max(0, min(frame.shape[1] - 1, x2))
        y2 = max(0, min(frame.shape[0] - 1, y2))

        cx = int(round((x1 + x2) / 2.0))
        cy = int(round((y1 + y2) / 2.0))

        pts = [
            [int(round(x1)), int(round(y1))],
            [int(round(x2)), int(round(y1))],
            [int(round(x2)), int(round(y2))],
            [int(round(x1)), int(round(y2))],
        ]

        if label_name == "white_region" and not white_region_detected:
            world_pos = cv2.perspectiveTransform(
                np.array([[[float(cx), float(cy)]]], dtype="float32"),
                M,
            )
            wx, wy = world_pos[0][0]

            if 0 <= wx <= 1 and 0 <= wy <= 1:
                robot_x, robot_y = camera_to_robot_coords(wx, wy)
                white_region_coords = {"x": robot_x, "y": robot_y}
                white_region_detected = True
                if not hasattr(image_callback, 'white_region_logged'):
                    image_callback.white_region_logged = True
                    rospy.loginfo(f"📍 Work area detected at ({robot_x:.3f}, {robot_y:.3f}m)")
            continue

        obj_info = OBJECT_MAPPING.get(label_name)
        if not obj_info:
            continue

        world_pos = cv2.perspectiveTransform(
            np.array([[[float(cx), float(cy)]]], dtype="float32"),
            M,
        )
        wx, wy = world_pos[0][0]

        if not (0 <= wx <= 1 and 0 <= wy <= 1):
            continue

        robot_x, robot_y = camera_to_robot_coords(wx, wy)

        if hasattr(image_callback, 'debug_count'):
            image_callback.debug_count += 1
        else:
            image_callback.debug_count = 1

        if image_callback.debug_count % 100 == 0:
            if origin_calibrated:
                rospy.loginfo(
                    f"🔍 Calibrated: Camera({cx},{cy}) -> World({wx:.3f},{wy:.3f}) -> Robot({robot_x:.3f},{robot_y:.3f})m"
                )
            else:
                rospy.loginfo(
                    f"🔍 Uncalibrated: Camera({cx},{cy}) -> World({wx:.3f},{wy:.3f}) -> Robot({robot_x:.3f},{robot_y:.3f})m"
                )

        detections_list.append({
            "cx": cx,
            "cy": cy,
            "pts": pts,
            "roll": detection.get("angle", 0.0),
            "label": label_name,
            "class": obj_info["class"],
            "color": obj_info["color"],
            "x": robot_x,
            "y": robot_y,
            "z": 0.2,
            "confidence": float(confidence),
        })

    # 發送 JSON (always publish, even if empty)
    output = {
        "detections": detections_list,
        "white_region": white_region_coords if white_region_coords else None,
        "aruco_markers": aruco_info,  # 新增：ArUco 標記資訊
        "timestamp": rospy.Time.now().to_sec(),
        "frame_count": getattr(image_callback, 'frame_count', 0)
    }
    
    # Increment frame counter
    if not hasattr(image_callback, 'frame_count'):
        image_callback.frame_count = 0
    image_callback.frame_count += 1
    
    # Publish detection data
    pub_target.publish(json.dumps(output))
    
    # # Log detection summary (focused on required objects only)
    # if detections_list:
    #     detection_summary = [f"{d['color']} {d['class']}" for d in detections_list]
    #     rospy.loginfo(f"📦 Detected {len(detections_list)} cubes: {detection_summary}")
    
    # Log white region status
    if white_region_coords:
        rospy.loginfo_once(f"📍 Work area (white region) located at ({white_region_coords['x']:.3f}, {white_region_coords['y']:.3f})")

# --- 訂閱攝影機 ---
rospy.Subscriber('/web_camera/image_raw', Image, image_callback)

# --- ROS 主循環 ---
rospy.spin()
