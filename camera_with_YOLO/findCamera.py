import cv2

'''
用來找電腦接到哪個鏡頭
'''



for i in range(5):  # 假設最多試5個攝影機
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"攝影機 {i} 可用")
        cap.release()
    else:
        print(f"攝影機 {i} 不可用")
