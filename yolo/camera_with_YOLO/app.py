from flask import Flask, render_template, Response
import cv2
from ultralytics import YOLO
import numpy as np
import threading
import time

# 載入 YOLO 模型（請確認模型路徑正確）
model = YOLO("model/best.pt")

# 建立 Flask 應用程式
app = Flask(__name__)

# 開啟攝影機（0 表內建攝影機）
camera = cv2.VideoCapture(0)

# 初始化 ArUco 標記的字典與參數
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

# 桌面實際寬高（單位：公分），用於座標轉換
TABLE_WIDTH_CM = 60
TABLE_HEIGHT_CM = 40

# 共用變數與鎖（防止多執行緒衝突）
lock = threading.Lock()
output_frame = None  # 儲存最新處理後的畫面


def process_frames():
    """
    背景執行緒：處理攝影機畫面，進行 ArUco 偵測與 YOLO 物件辨識，
    並將結果畫到影像上。
    """
    global output_frame
    while True:
        success, frame = camera.read()  # 讀取影像
        if not success:
            continue  # 如果讀不到影像，跳過此次迴圈

        # 轉成灰階以利 ArUco 偵測
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        # 若偵測到至少 4 個 ArUco 標記
        if ids is not None and len(ids) >= 4:
            id_list = ids.flatten()
            aruco_points = {}

            # 取得每個 ArUco 的中心點
            for i, corner in enumerate(corners):
                id = id_list[i]
                aruco_points[id] = corner[0].mean(axis=0)

            # 確保四個特定的 ID 存在（0,1,2,3）
            if all(k in aruco_points for k in [0, 1, 2, 3]):
                # 組合對應影像座標的四個角落
                src_pts = np.array([
                    aruco_points[0],  # 左下
                    aruco_points[1],  # 右下
                    aruco_points[2],  # 右上
                    aruco_points[3]   # 左上
                ], dtype="float32")

                # 對應到桌面標準平面 (0~1) 區域
                dst_pts = np.array([
                    [0, 0],
                    [1, 0],
                    [1, 1],
                    [0, 1]
                ], dtype="float32")

                # 計算透視轉換矩陣 M
                M = cv2.getPerspectiveTransform(src_pts, dst_pts)

                # 使用 YOLO 模型進行物件辨識
                results = model(frame, verbose=False)[0]

                # 檢查是否有偵測到物件（且為旋轉框）
                if results and results.obb is not None and len(results.obb) > 0:
                    for box in results.obb:
                        cls_id = int(box.cls[0])  # 取得類別 ID
                        label_name = model.names[cls_id]  # 取得標籤名稱

                        if label_name.lower() == "other":
                            continue  # 忽略「其他」類型

                        conf = float(box.conf[0])  # 信心度
                        label = f"{label_name} {conf:.2f}"

                        # 取得旋轉框的四個點
                        if hasattr(box, "xyxyxyxy") and box.xyxyxyxy is not None:
                            pts = box.xyxyxyxy.cpu().numpy().astype(int).reshape((-1, 2))

                            # 計算旋轉框的中心點
                            cx = int(np.mean(pts[:, 0]))
                            cy = int(np.mean(pts[:, 1]))

                            # 將中心點從影像座標轉換到桌面座標系
                            world_pos = cv2.perspectiveTransform(np.array([[[cx, cy]]], dtype='float32'), M)
                            wx, wy = world_pos[0][0]

                            # 檢查是否落在桌面範圍內（0~1）
                            if 0 <= wx <= 1 and 0 <= wy <= 1:
                                real_x = wx * TABLE_WIDTH_CM
                                real_y = wy * TABLE_HEIGHT_CM

                                # 顯示座標與標籤
                                text = f"{label} ({real_x:.1f}cm, {real_y:.1f}cm)"
                                cv2.putText(frame, text, (cx, cy - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                            # 畫出旋轉框與中心點
                            cv2.polylines(frame, [pts.reshape(-1, 1, 2)], isClosed=True, color=(0, 255, 0), thickness=2)
                            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        # 更新最新畫面（加鎖保護）
        with lock:
            output_frame = frame.copy()

        time.sleep(0.01)  # 暫停一下，避免佔用過多 CPU 資源


def generate_frames():
    """
    給 Flask /video route 使用，持續回傳最新處理後的影像 frame。
    """
    global output_frame
    while True:
        with lock:
            if output_frame is None:
                continue  # 若尚無影像資料，跳過
            ret, buffer = cv2.imencode('.jpg', output_frame)  # 將畫面轉成 JPEG 格式
            frame = buffer.tobytes()

        # 回傳 multipart 資料流給前端
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/')
def index():
    """
    首頁：顯示 index.html 畫面（請放在 templates 資料夾中）。
    """
    return render_template('index.html')


@app.route('/video')
def video():
    """
    影片串流路由：回傳即時影像流。
    """
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    # 啟動影像處理背景執行緒
    t = threading.Thread(target=process_frames, daemon=True)
    t.start()

    # 啟動 Flask 網頁伺服器
    app.run(debug=True, use_reloader=False)
