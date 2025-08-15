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
    else:
        rospy.loginfo(f"✅ 成功載入 no-signal 圖片: {no_signal_image_path}")
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
        rospy.logwarn("❌ 攝影機開啟失敗，將使用 no-signal 圖片")
except Exception as e:
    camera_available = False
    rospy.logerr(f"❌ 攝影機初始化時發生錯誤: {e}，將使用 no-signal 圖片")

# ========== ArUco 設定 ==========
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
TABLE_WIDTH_CM = 23
TABLE_HEIGHT_CM = 30

# ========== YOLO 偵測結果 ==========
detected_objects = []  # 當前偵測到的物件

def yolo_result_callback(msg):
    global detected_objects
    try:
        data = json.loads(msg.data)
        # 保證 detected_objects 永遠是最新的
        detected_objects = data if data else []
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

# ========== 背景影像處理 ==========
output_frame = None

def process_frames():
    global output_frame, camera_available, camera, camera_error_logged
    while not rospy.is_shutdown() and RUNNING:
        frame = None
        
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
                    # 攝影機讀取成功，進行正常處理
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

                    # 畫出 YOLO 偵測框（使用最新 detected_objects）
                    for obj in detected_objects:
                        pts = np.array(obj["pts"], dtype=int).reshape((-1, 1, 2))
                        label = obj["label"]
                        cx, cy = int(obj["cx"]), int(obj["cy"])

                        # 印出當前物件的中心座標
                        #print(f"偵測到物件: {label}, cx: {cx}, cy: {cy}")

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
            # 直接使用原始的 no-signal 圖片，不添加任何文字
            frame = no_signal_frame.copy()

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

@app.route('/behavior_tree_status', methods=['GET'])
def behavior_tree_status():
    global behavior_tree_data
    return jsonify({
        "structure": behavior_tree_data.get('structure'),
        "status": behavior_tree_data.get('status'),
        "timestamp": behavior_tree_data.get('timestamp', 0),
        "has_data": behavior_tree_data.get('structure') is not None
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
