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

# ========== 開啟攝影機 ==========
camera = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
if not camera.isOpened():
    rospy.logwarn("無法透過 /dev/video0 開啟攝影機，改用 index 0")
    camera = cv2.VideoCapture(0, cv2.CAP_V4L2)

if not camera.isOpened():
    rospy.logerr("❌ 攝影機開啟失敗")
    exit()
rospy.loginfo("✅ 攝影機開啟成功")

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

# ========== 背景影像處理 ==========
output_frame = None

def process_frames():
    global output_frame
    while not rospy.is_shutdown():
        success, frame = camera.read()
        if not success:
            continue

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

        # 更新全域影像
        output_frame = frame.copy()

        # 發佈 ROS Image
        try:
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            image_pub.publish(ros_image)
        except Exception as e:
            rospy.logerr(f"影像發佈失敗: {e}")

# ========== 視訊串流產生器 ==========
def generate_frames():
    global output_frame
    while True:
        if output_frame is None:
            continue
        ret, buffer = cv2.imencode('.jpg', output_frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

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

@app.route('/pick_object', methods=['POST'])
def pick_object():
    data = request.get_json()
    target_label = data.get("label", "").lower()

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

# ========== 主程式 ==========
if __name__ == '__main__':
    t = threading.Thread(target=process_frames, daemon=True)
    t.start()
    app.run(host="0.0.0.0", port=5000, debug=True, use_reloader=False)
