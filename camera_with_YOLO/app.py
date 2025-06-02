#加上YOLO辨識的版本

from flask import Flask, render_template, Response
import cv2
from ultralytics import YOLO

# 載入訓練好的 YOLOv8 模型
model = YOLO("model/best.pt")  # <-- 改成你的 best.pt 路徑

app = Flask(__name__)
camera = cv2.VideoCapture(0)  # 0 表示使用內建攝影機

def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            # YOLO 模型使用 BGR 格式影像進行推論
            results = model(frame, verbose=False)[0]

            # 逐個標出偵測到的物件
            for box in results.boxes:
                cls_id = int(box.cls[0])  # 類別 ID
                label_name = model.names[cls_id]

                if label_name.lower() == "other":
                    continue  # 不顯示 other 類別

                # 顯示非-other 類別
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                label = f"{label_name} {conf:.2f}"

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 2)
            # 將畫面轉為 JPEG 並串流
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()

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


