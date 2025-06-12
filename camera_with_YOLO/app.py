#加上Aruco的版本  要有四個Aruco同時出現在畫面上  才能辨識位置

from flask import Flask, render_template, Response
import cv2
from ultralytics import YOLO
import numpy as np

# 載入訓練好的 YOLOv8 模型
model = YOLO("model/best.pt")  # <-- 改成你的 best.pt 路徑

app = Flask(__name__)
camera = cv2.VideoCapture(0)  # 0 表示使用內建攝影機

# ArUco 偵測相關設定
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

# 定義桌面實際尺寸（公分）
TABLE_WIDTH_CM = 60
TABLE_HEIGHT_CM = 40

def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            break

        # ----------- [1] 偵測 ArUco 四角 -------------
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        if ids is not None and len(ids) >= 4:
            # 假設我們需要的是 ID 0~3，對應桌面四角 (0左下、1右下、2右上、3左上)
            id_list = ids.flatten()
            aruco_points = {}

            for i, corner in enumerate(corners):
                id = id_list[i]
                aruco_points[id] = corner[0].mean(axis=0)  # 取中心

            # 檢查是否四個標記都有
            if all(k in aruco_points for k in [0, 1, 2, 3]):
                # 建立轉換矩陣 (影像座標 → 桌面平面)
                src_pts = np.array([
                    aruco_points[0],
                    aruco_points[1],
                    aruco_points[2],
                    aruco_points[3]
                ], dtype="float32")

                dst_pts = np.array([
                    [0, 0],
                    [1, 0],
                    [1, 1],
                    [0, 1]
                ], dtype="float32")

                M = cv2.getPerspectiveTransform(src_pts, dst_pts)

                # ----------- [2] YOLO 偵測紅/藍方塊 -------------
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

                            # 計算中心點
                            cx = int(np.mean(pts[:, 0]))
                            cy = int(np.mean(pts[:, 1]))

                            # 投影中心點到桌面座標系
                            world_pos = cv2.perspectiveTransform(np.array([[[cx, cy]]], dtype='float32'), M)
                            wx, wy = world_pos[0][0]

                            # 只在桌面範圍內顯示
                            if 0 <= wx <= 1 and 0 <= wy <= 1:
                                # 轉換為公分座標
                                real_x = wx * TABLE_WIDTH_CM
                                real_y = wy * TABLE_HEIGHT_CM

                                # 顯示資訊
                                text = f"{label} ({real_x:.1f}cm, {real_y:.1f}cm)"
                                cv2.putText(frame, text, (cx, cy - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)


                            # 畫出旋轉框與中心點
                            cv2.polylines(frame, [pts.reshape(-1, 1, 2)], isClosed=True, color=(0, 255, 0), thickness=2)
                            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        # 編碼與回傳畫面
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/')
def index():
    return render_template('index.html')  # HTML 頁面請自行建立 templates/index.html


@app.route('/video')
def video():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(debug=True)



