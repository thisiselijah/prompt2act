from ultralytics import YOLO

# 載入預訓練的 YOLOv8n 模型（nano版本）
model = YOLO("yolov8n-obb.pt")

# Windows 系統使用 multiprocessing 時，必須將訓練代碼包在這個判斷內
if __name__ == '__main__':
    # 開始訓練模型
    model.train(
        data="dataSet/data.yaml",  # 資料集配置文件路徑，定義訓練/驗證集及類別
        epochs=100,                # 訓練迭代次數
        imgsz=640,                 # 輸入圖片尺寸（640x640）
        batch=16,                  # 每個批次的圖片數量
        device=0,                  # 使用的設備編號，0代表第一張GPU，若無GPU設成 'cpu'

        #指定輸出資料夾：
        project="output",          # 預設會在這個資料夾下建立子資料夾
        name="red_blue_model"      # 子資料夾名稱，例如：output_dir/red_blue_model
    )
