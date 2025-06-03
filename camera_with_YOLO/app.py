#加上YOLO辨識的版本

from flask import Flask, render_template, Response
import cv2
from ultralytics import YOLO

# 載入訓練好的 YOLOv8 模型
model = YOLO("model/best.pt")  # <-- 改成你的 best.pt 路徑

app = Flask(__name__)
camera = cv2.VideoCapture(0)  # 0 表示使用內建攝影機

import numpy as np

def generate_frames():
    while True:
        # 從攝影機讀取一幀畫面
        success, frame = camera.read()
        if not success:
            # 若讀取失敗，跳出迴圈結束產生影像
            break
        else:
            # 使用YOLO模型對畫面進行物件偵測，取得第一筆結果
            results = model(frame, verbose=False)[0]

            # 確認結果有OBB資料且不為空
            if results and results.obb is not None and len(results.obb) > 0:
                # 針對每個旋轉包圍盒進行處理
                for box in results.obb:
                    # 取得該box的類別ID並找出類別名稱
                    cls_id = int(box.cls[0])
                    label_name = model.names[cls_id]

                    # 過濾掉類別名稱為 "other" 的目標
                    if label_name.lower() == "other":
                        continue

                    # 取得該box的信心度並製作標籤文字
                    conf = float(box.conf[0])
                    label = f"{label_name} {conf:.2f}"

                    # 若box有旋轉框的8點座標資訊，則將其繪製到影像上
                    if hasattr(box, "xyxyxyxy") and box.xyxyxyxy is not None:
                        # 將8點座標轉為numpy陣列並調整形狀方便cv2使用
                        pts = box.xyxyxyxy.cpu().numpy().astype(int).reshape((-1, 1, 2))

                        # 用綠色線條畫出多邊形(旋轉包圍盒)
                        cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

                        # 在包圍盒左上角附近放置標籤文字
                        x, y = pts[0][0]
                        cv2.putText(frame, label, (x, y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 將處理好的畫面編碼為JPEG格式的位元組
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()

            # 產生串流格式的HTTP回應，每幀影像中間用boundary分隔
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')




#當使用者進入網站根目錄 /（例如 http://localhost:5000/） 這個函數會回傳 index.html 這個 HTML 頁面，讓使用者看到畫面
@app.route('/')
def index():
    return render_template('index.html')  # HTML 頁面請自行建立 templates/index.html

#而這個函數會使用 generate_frames() 不斷產生圖片（每一幀畫面），並用 MJPEG 格式串流給瀏覽器，
@app.route('/video')
def video():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(debug=True)


