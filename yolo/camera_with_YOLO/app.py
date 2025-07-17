from flask import Flask, render_template, Response
import cv2
from ultralytics import YOLO
import numpy as np
import threading
import time
import supervision as sv 

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
    global output_frame
    oriented_box_annotator = sv.OrientedBoxAnnotator(thickness=3)

    while True:
        success, frame = camera.read()
        if not success:
            continue

        # ArUco 偵測
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        if ids is not None and len(ids) >= 4:
            id_list = ids.flatten()
            aruco_points = {}
            for i, corner in enumerate(corners):
                id = id_list[i]
                aruco_points[id] = corner[0].mean(axis=0)

            if all(k in aruco_points for k in [0, 1, 2, 3]):
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

                # YOLO 偵測
                results = model(frame, verbose=False)[0]

                if results and results.obb is not None and len(results.obb) > 0:
                    # 轉換成 supervision 偵測物件
                    detections = sv.Detections.from_ultralytics(results)

                    # 用 supervision 畫旋轉框（這裡會畫出所有旋轉框和標籤）
                    annotated_frame = oriented_box_annotator.annotate(scene=frame, detections=detections)

                    # 在 annotated_frame 上疊加座標轉換的文字標籤
                    for box in results.obb:
                        try:
                            x, y, w_box, h_box, angle = box.xywhr[0].cpu().numpy()
                            conf = float(box.conf[0].cpu().numpy())
                            cls_id = int(box.cls[0].cpu().numpy())
                            label_name = model.names[cls_id]

                            if label_name.lower() == "other":
                                continue

                            h_img, w_img = frame.shape[:2]
                            cx = x * w_img
                            cy = y * h_img

                            # 座標轉世界座標
                            world_pos = cv2.perspectiveTransform(np.array([[[cx, cy]]], dtype='float32'), M)
                            wx, wy = world_pos[0][0]

                            if 0 <= wx <= 1 and 0 <= wy <= 1:
                                real_x = wx * TABLE_WIDTH_CM
                                real_y = wy * TABLE_HEIGHT_CM
                                text = f"({real_x:.1f}cm, {real_y:.1f}cm)"
                                # 文字位置稍微往上方一點
                                cv2.putText(annotated_frame, text, (int(cx), int(cy) - 35),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                        except Exception as e:
                            print("⚠️ 處理座標轉換錯誤:", e)

                    # 更新畫面
                    with lock:
                        output_frame = annotated_frame.copy()

                else:
                    # 如果沒物件，直接更新原始影像
                    with lock:
                        output_frame = frame.copy()
        else:
            # 若 ArUco 不足 4 個，直接更新原始影像
            with lock:
                output_frame = frame.copy()

        time.sleep(0.01)


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
