#!/usr/bin/python3
import rospkg
import os
import rospy
from flask import Flask, render_template, Response, request, jsonify
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import threading
import json
import signal
import sys

from niryo_web_interface.srv import PickObject, PickObjectResponse  # 修改為你實際的 package 名稱
from speech_recognition_package.srv import ProcessAudioCommand


# ========== ROS 初始化 ==========
rospack = rospkg.RosPack()
package_path = rospack.get_path('niryo_web_interface')

rospy.init_node('web_camera_node', anonymous=True)
bridge = CvBridge()

# ========== ROS Publisher ==========
arm_command_pub = rospy.Publisher('/niryo_arm_command', String, queue_size=10)
image_pub = rospy.Publisher('/web_camera/image_raw', Image, queue_size=1)

# ========== Flask 初始化 ==========
app = Flask(
    __name__,
    template_folder=os.path.join(package_path, 'templates'),
    static_folder=os.path.join(package_path, 'static')
)

# ========== 載入 No Signal 圖片 ==========
# no-signal image lives in the package's static folder
no_signal_image_path = os.path.join(package_path, 'static', 'no-signal.png')
no_signal_frame = None
try:
    no_signal_frame = cv2.imread(no_signal_image_path)
    if no_signal_frame is None:
        rospy.logwarn(f"無法載入 no-signal 圖片: {no_signal_image_path}")
        # 創建一個黑色的替代圖片
        no_signal_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(no_signal_frame, "NO CAMERA SIGNAL", (150, 240), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
except Exception as e:
    rospy.logerr(f"載入 no-signal 圖片時發生錯誤: {e}")
    # 創建一個黑色的替代圖片
    no_signal_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(no_signal_frame, "NO CAMERA SIGNAL", (150, 240), 
               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

# ========== 開啟攝影機 ==========
camera_available = False
camera = None
camera_error_logged = False  # 新增：避免重複列印錯誤

# 控制是否在啟動或斷線時自動嘗試替代裝置連線 (設為 False -> 不自動重連)
AUTO_FALLBACK_DEVICE = False

# 全域執行旗標，用來優雅關閉背景執行緒與產生器
RUNNING = True

# 嘗試開啟攝影機
try:
    camera = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
    if not camera.isOpened() and AUTO_FALLBACK_DEVICE:
        # 僅在顯式允許自動 fallback 時才嘗試 index 0
        rospy.logwarn("無法透過 /dev/video0 開啟攝影機，改用 index 0")
        camera = cv2.VideoCapture(0, cv2.CAP_V4L2)

    if camera is not None and camera.isOpened():
        camera_available = True
        rospy.loginfo("✅ 攝影機開啟成功")
    else:
        camera_available = False
        rospy.logwarn("❌ 攝影機開啟失敗")
except Exception as e:
    camera_available = False
    rospy.logerr(f"❌ 攝影機初始化時發生錯誤: {e}")

# ========== ArUco 設定 ==========
# ArUco 標記檢測已移至 yolo_detection_node 進行，避免重複計算
# aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
# aruco_params = cv2.aruco.DetectorParameters()
TABLE_WIDTH_CM = 23
TABLE_HEIGHT_CM = 30

# ========== YOLO 偵測結果 ==========
detected_objects = []  # 當前偵測到的物件
white_region_coords = None  # 全域變數，只存一次
aruco_markers_info = None  # 全域變數，儲存 ArUco 標記資訊

# ========== Debug Mode ==========
debug_mode_enabled = False  # Global flag for debug mode
debug_info_alpha = 0.7  # Debug info background transparency (0.0-1.0)

def yolo_result_callback(msg):
    global detected_objects, white_region_coords, aruco_markers_info
    try:
        data = json.loads(msg.data)
        detected_objects = data.get("detections", [])

        # 只在尚未存過 white_region 時才儲存
        if white_region_coords is None:
            white_region_coords = data.get("white_region", None)
            if white_region_coords is not None:
                rospy.loginfo(f"白色區塊座標已儲存: {white_region_coords}")
        
        # 接收 ArUco 標記資訊
        aruco_markers_info = data.get("aruco_markers", None)

    except Exception as e:
        rospy.logerr(f"解析 YOLO 偵測結果失敗: {e}")


rospy.Subscriber('/yolo_detected_targets', String, yolo_result_callback)

# ========== 行為樹狀態接收 ==========
behavior_tree_data = {"structure": None, "status": None, "timestamp": 0}

def behavior_tree_callback(msg):
    global behavior_tree_data
    try:
        data = json.loads(msg.data)
        # 更新行為樹數據
        if 'structure' in data:
            behavior_tree_data['structure'] = data['structure']
        if 'status' in data:
            behavior_tree_data['status'] = data['status']
        behavior_tree_data['timestamp'] = data.get('timestamp', 0)
        rospy.logdebug("收到行為樹狀態更新")
    except Exception as e:
        rospy.logerr(f"解析行為樹狀態失敗: {e}")

rospy.Subscriber('/behavior_tree_json', String, behavior_tree_callback)

# ========== Debug Mode Drawing Functions ==========
def draw_coordinate_axes(frame, M, scale=0.1, alpha=0.7):
    """
    Draw coordinate axes on the frame based on perspective transform
    Args:
        frame: OpenCV frame
        M: Perspective transform matrix
        scale: Length of axes in normalized coordinates (0-1)
        alpha: Background transparency (0.0 = fully transparent, 1.0 = fully opaque)
    """
    try:
        # 確保 M 不是 None
        if M is None:
            rospy.logwarn_throttle(5.0, "⚠️ Cannot draw coordinate axes: M is None")
            return
        
        # Compute inverse transform
        M_inv = np.linalg.inv(M)
        
        # Origin point (0, 0) in world coordinates
        origin_world = np.array([[[0.0, 0.0]]], dtype='float32')
        origin_pixel = cv2.perspectiveTransform(origin_world, M_inv)
        ox, oy = int(origin_pixel[0][0][0]), int(origin_pixel[0][0][1])
        
        # X-axis endpoint
        x_axis_world = np.array([[[scale, 0.0]]], dtype='float32')
        x_axis_pixel = cv2.perspectiveTransform(x_axis_world, M_inv)
        xx, xy = int(x_axis_pixel[0][0][0]), int(x_axis_pixel[0][0][1])
        
        # Y-axis endpoint
        y_axis_world = np.array([[[0.0, scale]]], dtype='float32')
        y_axis_pixel = cv2.perspectiveTransform(y_axis_world, M_inv)
        yx, yy = int(y_axis_pixel[0][0][0]), int(y_axis_pixel[0][0][1])
        
        # Draw X-axis (Red) with thicker line
        cv2.arrowedLine(frame, (ox, oy), (xx, xy), (0, 0, 255), 4, tipLength=0.3)
        cv2.putText(frame, "X", (xx + 15, xy), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        
        # Draw Y-axis (Green) with thicker line
        cv2.arrowedLine(frame, (ox, oy), (yx, yy), (0, 255, 0), 4, tipLength=0.3)
        cv2.putText(frame, "Y", (yx, yy + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        
        # Draw origin point with enhanced visibility
        cv2.circle(frame, (ox, oy), 8, (255, 255, 255), -1)
        cv2.circle(frame, (ox, oy), 10, (0, 0, 0), 2)
        
        # Draw origin label with background
        label = "Origin (0,0)"
        (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        
        # Semi-transparent background
        overlay = frame.copy()
        cv2.rectangle(overlay, (ox + 8, oy - label_h - 18), 
                     (ox + label_w + 18, oy - 3), (0, 0, 0), -1)
        cv2.addWeighted(overlay, alpha, frame, 1-alpha, 0, frame)
        
        cv2.putText(frame, label, (ox + 12, oy - 8), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        rospy.logdebug("✅ Coordinate axes drawn successfully")
        
    except Exception as e:
        rospy.logwarn_throttle(5.0, f"Failed to draw coordinate axes: {e}")

def draw_debug_info(frame, detected_objects, white_region_coords, alpha=0.7):
    """
    Draw debug information on the frame
    Args:
        frame: OpenCV frame
        detected_objects: List of detected objects with coordinates
        white_region_coords: White region coordinates
        alpha: Background transparency (0.0 = fully transparent, 1.0 = fully opaque)
    """
    try:
        import math
        
        # Draw title with semi-transparent background
        overlay = frame.copy()
        cv2.rectangle(overlay, (5, 5), (350, 40), (0, 0, 0), -1)
        cv2.addWeighted(overlay, alpha, frame, 1-alpha, 0, frame)
        cv2.putText(frame, "DEBUG MODE", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        
        # Draw detected objects count
        obj_count_text = f"Objects: {len(detected_objects)}"
        cv2.putText(frame, obj_count_text, (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Draw detected objects info
        y_offset = 90
        for i, obj in enumerate(detected_objects):
            label = obj.get('label', 'unknown')
            color_name = obj.get('color', 'unknown')
            class_name = obj.get('class', 'unknown')
            x = obj.get('x', 0.0)
            y = obj.get('y', 0.0)
            cx = obj.get('cx', 0)
            cy = obj.get('cy', 0)
            roll = obj.get('roll', 0.0)
            confidence = obj.get('confidence', 0.0)
            
            # Draw semi-transparent background for each object info
            overlay = frame.copy()
            cv2.rectangle(overlay, (5, y_offset - 18), (450, y_offset + 58), (0, 0, 0), -1)
            cv2.addWeighted(overlay, alpha, frame, 1-alpha, 0, frame)
            
            # Draw info text with confidence
            info_text = f"{i+1}. {color_name} {class_name} (conf: {confidence:.2f})"
            cv2.putText(frame, info_text, (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            y_offset += 20
            
            coord_text = f"   Robot: ({x:.3f}, {y:.3f}m)"
            cv2.putText(frame, coord_text, (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            y_offset += 17
            
            pixel_text = f"   Pixel: ({cx}, {cy})"
            cv2.putText(frame, pixel_text, (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
            y_offset += 15
            
            # Convert roll to degrees for better readability
            roll_deg = math.degrees(roll)
            roll_text = f"   Roll: {roll:.3f} rad ({roll_deg:.1f}°)"
            cv2.putText(frame, roll_text, (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
            y_offset += 30
            
            # Draw crosshair at object center with label
            cv2.drawMarker(frame, (cx, cy), (0, 255, 255), 
                          cv2.MARKER_CROSS, 20, 2)
            cv2.circle(frame, (cx, cy), 3, (255, 255, 255), -1)
        
        # Draw white region info
        if white_region_coords:
            overlay = frame.copy()
            cv2.rectangle(overlay, (5, y_offset - 18), (400, y_offset + 30), (0, 0, 0), -1)
            cv2.addWeighted(overlay, alpha, frame, 1-alpha, 0, frame)
            
            cv2.putText(frame, f"White Region:", (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            y_offset += 20
            white_text = f"  ({white_region_coords['x']:.3f}, {white_region_coords['y']:.3f}m)"
            cv2.putText(frame, white_text, (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        else:
            cv2.putText(frame, "White Region: Not detected", (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
        
    except Exception as e:
        rospy.logwarn_throttle(5.0, f"Failed to draw debug info: {e}")

# ========== 背景影像處理 ==========
output_frame = None

def process_frames():
    global output_frame, camera_available, camera, camera_error_logged, debug_mode_enabled, aruco_markers_info
    while not rospy.is_shutdown() and RUNNING:
        frame = None
        M = None  # 移到外層，確保在所有情況下都可用
        
        # 檢查攝影機是否可用
        if camera_available and camera is not None:
            try:
                success, frame = camera.read()
                if not success:
                    # 攝影機讀取失敗，可能已斷線
                    if not camera_error_logged:
                        rospy.logwarn("攝影機讀取失敗，切換至 no-signal 模式")
                        camera_error_logged = True
                    camera_available = False
                    frame = no_signal_frame.copy()
                else:
                    # 攝影機讀取成功，重置錯誤標記
                    camera_error_logged = False
                    
                    # ✅ 優化：直接使用 YOLO 節點計算好的透視變換矩陣，避免重複計算
                    if aruco_markers_info and aruco_markers_info.get("detected"):
                        transform_matrix = aruco_markers_info.get("transform_matrix")
                        
                        if transform_matrix is not None:
                            try:
                                # 直接使用接收到的透視變換矩陣
                                M = np.array(transform_matrix, dtype='float32')
                                marker_count = aruco_markers_info.get("marker_count", 0)
                                # 優化：減少頻繁的成功訊息，只在狀態改變時記錄
                                if not hasattr(process_frames, 'last_marker_count') or process_frames.last_marker_count != marker_count:
                                    process_frames.last_marker_count = marker_count
                                    rospy.loginfo(f"📍 ArUco markers detected: {marker_count} markers")
                            except (ValueError, TypeError) as e:
                                # 優化：錯誤訊息更簡潔，只記錄一次
                                if not hasattr(process_frames, 'transform_error_logged'):
                                    process_frames.transform_error_logged = True
                                    rospy.logerr(f"Transform matrix error: {str(e)[:50]}...")
                                M = None
                        else:
                            # 優化：減少警告訊息頻率，只在狀態改變時記錄
                            if not hasattr(process_frames, 'no_transform_logged') or process_frames.no_transform_logged:
                                process_frames.no_transform_logged = False
                                rospy.logwarn("Waiting for ArUco markers...")
                            M = None

                    # 畫出 YOLO 偵測框（使用最新 detected_objects）
                    for obj in detected_objects:
                        pts = np.array(obj["pts"], dtype=int).reshape((-1, 1, 2))
                        label = obj["label"]
                        cx, cy = int(obj["cx"]), int(obj["cy"])

                        color = (255, 0, 0) if label == "blue_block" else (0, 0, 255)
                        cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=2)
                        cv2.circle(frame, (cx, cy), 5, color, -1)
                        cv2.putText(frame, label, (cx, cy - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    
            except Exception as e:
                if not camera_error_logged:
                    rospy.logerr(f"攝影機處理時發生錯誤: {e}")
                    camera_error_logged = True
                camera_available = False
                frame = no_signal_frame.copy()
        else:
            # 攝影機不可用，使用 no-signal 圖片
            frame = no_signal_frame.copy()

        # Debug mode 繪製（移到外層，適用於所有情況）
        if frame is not None and debug_mode_enabled:
            if M is not None:
                draw_coordinate_axes(frame, M, scale=0.2, alpha=debug_info_alpha)
            else:
                # 即使沒有 M，也顯示提示訊息
                cv2.putText(frame, "DEBUG MODE - Waiting for ArUco markers", 
                           (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.6, (0, 255, 255), 2)
            
            draw_debug_info(frame, detected_objects, white_region_coords, alpha=debug_info_alpha)

        # 更新全域影像
        if frame is not None:
            output_frame = frame.copy()

            # 發佈 ROS Image
            try:
                ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                image_pub.publish(ros_image)
            except Exception as e:
                rospy.logerr(f"影像發佈失敗: {e}")
        
        # 短暫延遲以避免過度消耗 CPU
        rospy.sleep(0.033)  # 約 30 FPS

# ========== 視訊串流產生器 ==========
def generate_frames():
    global output_frame, no_signal_frame
    # 產生器在 RUNNING 為 False 時結束，避免在程序關閉後持續嘗試編碼
    while RUNNING:
        # 如果沒有影像，使用 no-signal 圖片
        if output_frame is None:
            # 不在 fallback 圖片上繪製任何額外文字
            frame_to_encode = no_signal_frame.copy()
        else:
            frame_to_encode = output_frame
            
        ret, buffer = cv2.imencode('.jpg', frame_to_encode)
        frame = buffer.tobytes()
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


def _shutdown(signum, frame):
    """Signal handler for clean shutdown on SIGINT / SIGTERM."""
    global RUNNING, camera
    RUNNING = False
    rospy.loginfo("Shutdown signal received (signal %s)", signum)
    # release camera if opened
    try:
        if camera is not None:
            camera.release()
    except Exception:
        pass
    # print Bye to stdout as requested
    print("Bye")
    # give ROS a moment to shutdown
    try:
        rospy.signal_shutdown('shutdown via signal')
    except Exception:
        pass
    # exit process
    sys.exit(0)

# ========== ROS Service Callback ==========
def pick_object_service_callback(req):
    """
    ROS Service 用來接收 x, y, roll, label 指令並送出抓取命令
    """
    command = f"pick_at:{req.x:.3f},{req.y:.3f},{req.roll:.3f}"
    rospy.loginfo(f"收到 Service 請求：label={req.label}, x={req.x}, y={req.y}, roll={req.roll}")
    arm_command_pub.publish(command)
    return PickObjectResponse(message=f"✅ 已送出抓取指令：{req.label} → ({req.x:.2f}, {req.y:.2f}), roll = {req.roll:.3f}")

# ========== 建立 Service ==========
pick_object_service = rospy.Service('/pick_object_service', PickObject, pick_object_service_callback)

# ========== Flask 路由 ==========
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video')
def video():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/open_and_close', methods=['POST'])
def open_and_close():
    data = request.get_json()
    direction = data.get("direction", "")
    arm_command_pub.publish(direction)
    
    return jsonify({"message": f"已送出 {direction} 指令"})

@app.route('/learning_mode', methods=['POST'])
def learning_mode():
    arm_command_pub.publish("learning_mode")

    return jsonify({"message": "已切換至 Learning Mode"})

@app.route('/move_to_home', methods=['POST'])
def move_to_home():
    arm_command_pub.publish("move_to_home")

    return jsonify({"message": "已切換至 Home Pose"})

@app.route('/camera_status', methods=['GET'])
def camera_status():
    global camera_available

    return jsonify({
        "camera_available": camera_available,
        "status": "connected" if camera_available else "disconnected",
        "message": "攝影機已連接" if camera_available else "攝影機未連接"
    })

@app.route('/reconnect_camera', methods=['POST'])
def reconnect_camera():
    global camera_available, camera, camera_error_logged

    try:
        rospy.loginfo("手動嘗試重新連接攝影機...")
        
        # 釋放現有攝影機資源
        if camera is not None:
            camera.release()
        
        # 僅嘗試明確的裝置，不自動嘗試其他 index
        camera = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        if camera is not None and camera.isOpened():
            camera_available = True
            camera_error_logged = False  # 重置錯誤標記
            rospy.loginfo("✅ 攝影機重新連接成功")
            return jsonify({"success": True, "message": "✅ 攝影機重新連接成功"})
        else:
            camera_available = False
            return jsonify({"success": False, "message": "❌ 攝影機重新連接失敗"}), 400
            
    except Exception as e:
        rospy.logerr(f"手動重新連接攝影機時發生錯誤: {e}")
        camera_available = False
        return jsonify({"success": False, "message": f"❌ 重新連接失敗: {str(e)}"}), 500

@app.route('/pick_object', methods=['POST'])
def pick_object():
    global camera_available

    data = request.get_json()
    target_label = data.get("label", "").lower()

    # 檢查攝影機狀態
    if not camera_available:
        return jsonify({"message": "❌ 攝影機未連接，無法進行物件偵測"}), 400

    x = None
    y = None
    roll = None

    for obj in detected_objects:
        if obj["label"] == target_label:
            x = obj.get("x")
            y = obj.get("y")
            roll = obj.get("roll")
            break

    if x is None or y is None or roll is None:
        return jsonify({"message": f"❌ 尚未偵測到 {target_label} 目標物"}), 400

    command = f"pick_at:{x:.3f},{y:.3f},{roll:.3f}"
    arm_command_pub.publish(command)
    return jsonify({"message": f"✅ 已送出抓取指令：{target_label} → ({x:.2f}, {y:.2f}), roll = {roll:.3f}"})


@app.route('/place_to_white_region', methods=['POST'])
def place_to_white_region():
    global white_region_coords

    if white_region_coords is None:
        return jsonify({"success": False, "message": "❌ 尚未偵測到白色區塊，無法放置"}), 400

    x = white_region_coords["x"]
    y = white_region_coords["y"]
    z = 0.163  # 放置高度（公尺）
    roll = 0.0  # 可以依需求調整
    pitch = 1.438
    yaw = -0.35

    # 發送到 ROS /niryo_arm_command
    command = f"place_at:{x:.3f},{y:.3f},{z:.3f},{roll:.3f},{pitch:.3f},{yaw:.3f}"
    arm_command_pub.publish(command)
    rospy.loginfo(f"放置至白色區塊: ({x:.3f}, {y:.3f})")

    return jsonify({"success": True, "message": f"✅ 已送出放置指令至白色區塊: ({x:.3f}, {y:.3f})"})


@app.route('/behavior_tree_status', methods=['GET'])
def behavior_tree_status():
    global behavior_tree_data

    return jsonify({
        "structure": behavior_tree_data.get('structure'),
        "status": behavior_tree_data.get('status'),
        "timestamp": behavior_tree_data.get('timestamp', 0),
        "has_data": behavior_tree_data.get('structure') is not None,
        "node_count": count_tree_nodes(behavior_tree_data.get('structure')) if behavior_tree_data.get('structure') else 0
    })

def count_tree_nodes(node):
    """Recursively count nodes in the tree structure"""
    if not node:
        return 0
    
    count = 1  # Count current node
    if node.get('children'):
        for child in node['children']:
            count += count_tree_nodes(child)
    
    return count


@app.route("/voice_command", methods=["POST"])
def voice_command():
    if "file" not in request.files:
        return jsonify({"error": "No audio file"}), 400

    audio_file = request.files["file"]
    save_path = "/tmp/recording.webm"
    audio_file.save(save_path)

    # 呼叫 ROS Service
    rospy.wait_for_service("/process_audio_command")
    try:
        process_audio = rospy.ServiceProxy("/process_audio_command", ProcessAudioCommand)
        resp = process_audio(audio_file_path=save_path, language="en-US")

        return jsonify({
            "success": resp.success,
            "transcribed_text": resp.transcribed_text,
            "behavior_tree_json": resp.behavior_tree_json,
            "error_message": resp.error_message
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/toggle_debug_mode', methods=['POST'])
def toggle_debug_mode():
    """Toggle debug mode on/off"""
    global debug_mode_enabled
    
    data = request.get_json()
    if data and 'enabled' in data:
        debug_mode_enabled = data['enabled']
    else:
        debug_mode_enabled = not debug_mode_enabled
    
    rospy.loginfo(f"Debug mode {'enabled' if debug_mode_enabled else 'disabled'}")
    
    return jsonify({
        "success": True,
        "debug_mode": debug_mode_enabled,
        "message": f"Debug mode {'enabled' if debug_mode_enabled else 'disabled'}"
    })

@app.route('/debug_mode_status', methods=['GET'])
def debug_mode_status():
    """Get current debug mode status"""
    global debug_mode_enabled
    
    return jsonify({
        "debug_mode": debug_mode_enabled
    })

@app.route('/set_debug_transparency', methods=['POST'])
def set_debug_transparency():
    """Set debug info transparency"""
    global debug_info_alpha
    
    data = request.get_json()
    if data and 'alpha' in data:
        alpha = float(data['alpha'])
        if 0.0 <= alpha <= 1.0:
            debug_info_alpha = alpha
            return jsonify({
                "success": True,
                "alpha": debug_info_alpha,
                "message": f"Debug transparency set to {debug_info_alpha:.2f}"
            })
        else:
            return jsonify({
                "success": False,
                "message": "Alpha must be between 0.0 and 1.0"
            }), 400
    
    return jsonify({
        "success": False,
        "message": "Missing alpha parameter"
    }), 400

@app.route('/get_debug_transparency', methods=['GET'])
def get_debug_transparency():
    """Get current debug transparency setting"""
    global debug_info_alpha
    
    return jsonify({
        "alpha": debug_info_alpha
    })

# ========== 主程式 ==========
if __name__ == '__main__':
    # 註冊信號處理器，確保 Ctrl+C (SIGINT) 或 SIGTERM 會優雅關閉
    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    # 啟動背景處理緒
    t = threading.Thread(target=process_frames, daemon=True)
    t.start()

    try:
        app.run(host="0.0.0.0", port=5000, debug=True, use_reloader=False)
    except (KeyboardInterrupt, SystemExit):
        # 如果 Flask 捕捉到 KeyboardInterrupt，呼叫關閉
        _shutdown(signal.SIGINT, None)
