#!/usr/bin/env python3

import rospkg # <--- 1. 導入 rospkg 函式庫
import os      # <--- 2. 導入 os 函式庫來組合路徑

import rospy
from flask import Flask, render_template, Response, request, jsonify
import cv2
from ultralytics import YOLO
import numpy as np
from std_msgs.msg import String

import time  # 引入 time 模組 (測試用)

rospack = rospkg.RosPack()
package_path = rospack.get_path('niryo_web_interface') # 請確認 'niryo_web_interface' 是你的 package 名稱

model_path = os.path.join(package_path, 'model', 'best.pt')

rospy.loginfo(f"Loading model from: {model_path}") # (可選) 印出路徑方便除錯

rospy.init_node('web_interface_node', anonymous=True)  #anonymous=True 讓節點支援衝突


app = Flask(__name__, template_folder=os.path.join(package_path, 'templates'))


try:
    model = YOLO(model_path)
except Exception as e:
    rospy.logerr(f"Failed to load YOLO model: {e}")
    # exit()
    
camera = None

rospy.loginfo("嘗試開啟攝影機...")
# First, try opening by device path, then by index.
# Using CAP_V4L2 backend is often more reliable on Linux.
camera = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
if not camera.isOpened():
    rospy.logwarn("無法透過 /dev/video0 開啟攝影機，嘗試使用索引 0...")
    camera = cv2.VideoCapture(0, cv2.CAP_V4L2)

if not camera.isOpened():
    rospy.logerr("❌ 無法開啟攝影機。請確認：")
    rospy.logerr("  1. 攝影機已正確連接。")
    rospy.logerr("  2. 若在 Docker 容器中，確認已使用 --device=/dev/video0 掛載。")
    rospy.logerr("  3. 在(容器)終端機執行 `ls -l /dev/video0` 確認設備存在且權限正確 (e.g. crw-rw-rw-).")
    exit()

rospy.loginfo("✅ 攝影機開啟成功!")

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

TABLE_WIDTH_CM = 60
TABLE_HEIGHT_CM = 40

arm_command_pub = rospy.Publisher('/niryo_arm_command', String, queue_size=10)

def generate_frames():
    rospy.logwarn("鏡頭開始")
    while not rospy.is_shutdown():
        # rospy.logwarn("嘗試讀取 frame...")  # 測試點 1

        success, frame = camera.read()
        # if not success:
        #     rospy.logwarn("讀不到影像")
        #     time.sleep(1)  # 加上一秒延遲
        #     continue  # 改成繼續嘗試
        #     # break
        
        # rospy.logwarn("成功讀取影像")  # 測試點 2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        if ids is not None and len(ids) >= 4:
            id_list = ids.flatten()
            aruco_points = {}

            for i, corner in enumerate(corners):
                id = id_list[i]
                aruco_points[id] = corner[0].mean(axis=0)

            if all(k in aruco_points for k in [0, 1, 2, 3]):
                src_pts = np.array([aruco_points[0], aruco_points[1],
                                    aruco_points[2], aruco_points[3]], dtype="float32")

                dst_pts = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype="float32")
                M = cv2.getPerspectiveTransform(src_pts, dst_pts)

                results = model(frame, verbose=False)[0]
                if results and results.obb is not None and len(results.obb) > 0:
                    for box in results.obb:
                        cls_id = int(box.cls[0])
                        label_name = model.names[cls_id]

                        if label_name.lower() == "other":
                            continue

                        conf = float(box.conf[0])
                        label = f"{label_name} {conf:.2f}"

                        if hasattr(box, "xyxyxyxy") and box.xyxyxyxy is not None:
                            pts = box.xyxyxyxy.cpu().numpy().astype(int).reshape((-1, 2))
                            cx = int(np.mean(pts[:, 0]))
                            cy = int(np.mean(pts[:, 1]))
                            world_pos = cv2.perspectiveTransform(np.array([[[cx, cy]]], dtype='float32'), M)
                            wx, wy = world_pos[0][0]

                            if 0 <= wx <= 1 and 0 <= wy <= 1:
                                real_x = wx * TABLE_WIDTH_CM
                                real_y = wy * TABLE_HEIGHT_CM
                                text = f"{label} ({real_x:.1f}cm, {real_y:.1f}cm)"
                                cv2.putText(frame, text, (cx, cy - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                            cv2.polylines(frame, [pts.reshape(-1, 1, 2)], isClosed=True, color=(0, 255, 0), thickness=2)
                            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()

        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video')
def video():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/move_arm', methods=['POST'])
def move_arm():
    data = request.get_json()
    direction = data.get("direction", "")
    arm_command_pub.publish(direction)
    return jsonify({"message": f"已送出 {direction} 指令"})


@app.route('/learning_mode', methods=['POST'])
def learning_mode():
    arm_command_pub.publish("learning_mode")
    return jsonify({"message": "已切換至 Learning Mode"})


if __name__ == '__main__':
    app.run(host="0.0.0.0", port=5000, debug=True, use_reloader=False)