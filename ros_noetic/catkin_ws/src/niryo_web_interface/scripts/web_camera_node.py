#!/usr/bin/python3
import rospkg
import os
import rospy
from flask import Flask, render_template, Response, request, jsonify
import cv2
import numpy as np
from std_msgs.msg import String
import threading
import time
import json

# --- ROS 初始化 ---
rospack = rospkg.RosPack()
package_path = rospack.get_path('niryo_web_interface')

rospy.init_node('web_camera_node', anonymous=True)

# --- Flask 初始化 ---
app = Flask(__name__,
            template_folder=os.path.join(package_path, 'templates'),
            static_folder=os.path.join(package_path, 'static'))

# --- 開啟攝影機 ---
camera = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
if not camera.isOpened():
    rospy.logwarn("無法透過 /dev/video0 開啟攝影機，改用 index 0")
    camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
if not camera.isOpened():
    rospy.logerr("❌ 攝影機開啟失敗")
    exit()
rospy.loginfo("✅ 攝影機開啟成功")

# --- ArUco ---
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

TABLE_WIDTH_CM = 23
TABLE_HEIGHT_CM = 30

# --- ROS Publisher ---
arm_command_pub = rospy.Publisher('/niryo_arm_command', String, queue_size=10)

# --- YOLO 偵測結果 (由另一個節點發佈) ---
detected_target = {"x": None, "y": None, "roll": None, "label": None}

def yolo_result_callback(msg):
    global detected_target
    try:
        detected_target = json.loads(msg.data)
    except Exception as e:
        rospy.logerr(f"解析 YOLO 偵測結果失敗: {e}")

rospy.Subscriber('/yolo_detected_target', String, yolo_result_callback)

output_frame = None
lock = threading.Lock()

# --- 背景影像處理 ---
def process_frames():
    global output_frame
    while not rospy.is_shutdown():
        success, frame = camera.read()
        if not success:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        if ids is not None:
            #cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            pass
        with lock:
            output_frame = frame.copy()

        time.sleep(0.03)

# --- 視訊串流 ---
def generate_frames():
    global output_frame
    while True:
        with lock:
            if output_frame is None:
                continue
            ret, buffer = cv2.imencode('.jpg', output_frame)
            frame = buffer.tobytes()
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# --- Flask 路由 ---
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

    x = detected_target.get("x")
    y = detected_target.get("y")
    roll = detected_target.get("roll")
    label = detected_target.get("label")

    if x is None or y is None or roll is None or label != target_label:
        return jsonify({"message": f"❌ 尚未偵測到 {target_label} 目標物"}), 400

    command = f"pick_at:{x:.3f},{y:.3f},{roll:.3f}"
    arm_command_pub.publish(command)
    return jsonify({"message": f"✅ 已送出抓取指令：{target_label} → ({x:.2f}, {y:.2f}), roll = {roll:.3f}"})

# --- 主程式 ---
if __name__ == '__main__':
    t = threading.Thread(target=process_frames, daemon=True)
    t.start()
    app.run(host="0.0.0.0", port=5000, debug=True, use_reloader=False)
