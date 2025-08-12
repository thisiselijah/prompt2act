#!/root/.pyenv/versions/3.9.19/bin/python3

# --- 套件匯入區 ---
import rospkg                     # 取得 ROS 套件目錄
import os
import rospy                      # ROS Python API
from flask import Flask, render_template, Response, request, jsonify  # 建立 Web 應用
import cv2                        # OpenCV 影像處理
from ultralytics import YOLO      # YOLOv8 模型
import numpy as np
from std_msgs.msg import String   # ROS 訊息型態
import threading
import time
import math  

# --- ROS 初始化 ---
rospack = rospkg.RosPack()
package_path = rospack.get_path('niryo_web_interface')   # 取得 ROS 專案目錄
model_path = os.path.join(package_path, 'model', 'best.pt')  # YOLO 模型路徑

rospy.loginfo(f"Loading model from: {model_path}")
rospy.init_node('web_interface_node', anonymous=True)  # 初始化 ROS Node

# --- Flask 初始化 ---
app = Flask(__name__,
            template_folder=os.path.join(package_path, 'templates'),   # HTML 模板
            static_folder=os.path.join(package_path, 'static'))       # 靜態資源（CSS/JS）

# --- 載入 YOLO 模型 ---
try:
    model = YOLO(model_path) 
except Exception as e:
    rospy.logerr(f"Failed to load YOLO model: {e}")

# --- 開啟攝影機 ---
camera = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)  # 嘗試開啟 USB 攝影機（Linux）
if not camera.isOpened():
    rospy.logwarn("無法透過 /dev/video0 開啟攝影機，改用 index 0")
    camera = cv2.VideoCapture(0, cv2.CAP_V4L2)  # 改用 index

if not camera.isOpened():
    rospy.logerr("❌ 攝影機開啟失敗，請檢查連接或權限")
    exit()
rospy.loginfo("✅ 攝影機開啟成功")

# --- ArUco 相關設定（用於標記桌面四角）---
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

# 桌面尺寸（單位：公分）
TABLE_WIDTH_CM = 23
TABLE_HEIGHT_CM = 30

# --- ROS Publisher ---
arm_command_pub = rospy.Publisher('/niryo_arm_command', String, queue_size=10)

# --- 全域變數 ---
detected_target = {"x": None, "y": None, "roll": None}  # 儲存 YOLO 偵測結果（世界座標）
output_frame = None                      # 最新畫面
lock = threading.Lock()                  # 保護畫面更新的鎖

# --- 座標轉換：將桌面單位座標轉成 Niryo 機械臂世界座標（公尺） ---
def camera_to_robot_coords(wx, wy):
    origin_x = 0.2     # Niryo 座標系中，桌面左上角 x 偏移（公尺）
    origin_y = -0.1    # Niryo 座標系中，桌面左上角 y 偏移（公尺）
    real_x_cm = wx * TABLE_WIDTH_CM
    real_y_cm = wy * TABLE_HEIGHT_CM
    return origin_x + real_x_cm / 100.0, origin_y + real_y_cm / 100.0


# --- 背景執行緒：處理即時畫面（YOLO 偵測 + ArUco 框） ---
def process_frames():
    global output_frame, detected_target

    prev_aruco_points = None  # 初始化
    

    while not rospy.is_shutdown():
        success, frame = camera.read()
        if not success:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        if ids is not None and len(ids) >= 4:
            id_list = ids.flatten()
            aruco_points = {}
            for i, corner in enumerate(corners):
                aruco_points[id_list[i]] = corner[0].mean(axis=0)  # 取得每個 ArUco 中心點

            # 根據 ArUco ID 位置順序重排
            if all(k in aruco_points for k in [0, 1, 2, 3]):
                src_pts = np.array([
                    aruco_points[3],  # 左上
                    aruco_points[2],  # 右上
                    aruco_points[1],  # 右下
                    aruco_points[0]   # 左下
                ], dtype="float32")

                dst_pts = np.array([
                    [0, 0],  # 轉換後的左上角
                    [1, 0],  # 右上
                    [1, 1],  # 右下
                    [0, 1],  # 左下
                ], dtype="float32")

                if not np.array_equal(prev_aruco_points, aruco_points):
                    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
                    prev_aruco_points = aruco_points.copy()

                # --- YOLOv8 OBB 偵測 ---
                results = model(frame, verbose=False)[0]

                if results and results.obb is not None and len(results.obb) > 0:
                    for box in results.obb:
                        cls_id = int(box.cls[0])
                        label_name = model.names[cls_id]
                        if label_name.lower() == "other":
                            continue

                        conf = float(box.conf[0])
                        label = f"{label_name} {conf:.2f}"

                        

                        # 確保有 OBB 資訊
                        if hasattr(box, "xyxyxyxy") and box.xyxyxyxy is not None:
                            pts = box.xyxyxyxy.cpu().numpy().astype(int).reshape((-1, 2))
                            cx = int(np.mean(pts[:, 0]))
                            cy = int(np.mean(pts[:, 1]))

                            # 計算 roll 角度（單位：弧度）
                            dx = pts[1][0] - pts[0][0]
                            dy = pts[1][1] - pts[0][1]
                            

                            # --- 座標轉換至桌面標準化平面 ---
                            world_pos = cv2.perspectiveTransform(
                                np.array([[[cx, cy]]], dtype='float32'), M)
                            wx, wy = world_pos[0][0]

                            # --- 檢查是否在有效區域內，並轉換為機械臂座標 ---
                            if 0 <= wx <= 1 and 0 <= wy <= 1:
                                real_x = wx * TABLE_WIDTH_CM
                                real_y = wy * TABLE_HEIGHT_CM
                                robot_x, robot_y = camera_to_robot_coords(wx, wy)

                                roll_rad = math.atan2(dy, dx)
                                roll_deg = math.degrees(roll_rad)

                                if roll_deg < -90:
                                    roll_deg += 180
                                elif roll_deg > 90:
                                    roll_deg -= 180
                                roll_rad = math.radians(roll_deg)


                                detected_target["x"] = robot_x
                                detected_target["y"] = robot_y
                                detected_target["roll"] = roll_rad
                                detected_target["label"] = label_name.lower()

                                # 顯示標籤
                                text = f"{label} ({real_x:.1f}cm, {real_y:.1f}cm)"
                                cv2.putText(frame, text, (cx, cy - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                            # 畫出 OBB 框與中心點
                            cv2.polylines(frame, [pts.reshape(-1, 1, 2)], isClosed=True, color=(0, 255, 0), thickness=2)
                            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)


        # 更新畫面（需加鎖保護）
        with lock:
            output_frame = frame.copy()

        time.sleep(0.05)  # 降低 CPU 使用率


# --- 串流畫面傳送函式（Flask 調用）---
def generate_frames():
    global output_frame
    while True:
        with lock:
            if output_frame is None:
                continue
            ret, buffer = cv2.imencode('.jpg', output_frame)
            frame = buffer.tobytes()
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


# --- Flask 路由區塊 ---

@app.route('/')
def index():
    return render_template('index.html')  # 主頁面（含視訊與控制 UI）

@app.route('/video')
def video():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')  # 視訊串流

#控制開合
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
    arm_command_pub.publish("move_to_home")  # 發送指令
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
    return jsonify({
        "message": f"✅ 已送出抓取指令：{target_label} → ({x:.2f}, {y:.2f}), roll = {roll:.3f}"
    })





# --- 主程式入口點 ---
if __name__ == '__main__':
    t = threading.Thread(target=process_frames, daemon=True)  # 啟動背景影像處理
    t.start()

    app.run(host="0.0.0.0", port=5000, debug=True, use_reloader=False)  # 啟動 Flask Web Server
