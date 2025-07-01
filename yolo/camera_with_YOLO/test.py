import cv2

cap = cv2.VideoCapture('/dev/video0')
if not cap.isOpened():
    print("無法開啟攝影機")
else:
    ret, frame = cap.read()
    if ret:
        print("成功讀取影像")
    else:
        print("讀不到影像")
    cap.release()
