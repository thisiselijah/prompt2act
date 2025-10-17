# Debug Mode Feature - Web GUI

## 📋 功能概述

在 Web GUI 中添加了 **Debug Mode（調試模式）**功能，讓使用者可以通過點擊按鈕切換模式，在攝影機畫面上顯示座標軸和詳細的物體座標資訊。

---

## ✨ 主要功能

### **1. 座標軸顯示**
- **X 軸（紅色）**：指向攝影機視角的右側
- **Y 軸（綠色）**：指向攝影機視角的下側
- **原點標記（白色）**：顯示座標系統的原點 (0, 0)
- **動態調整**：座標軸方向會根據 ArUco 標記的透視變換自動調整

### **2. 物體座標資訊**
顯示每個偵測到的物體的詳細資訊：
- **物體編號與名稱**：例如 "1. blue cube"
- **機器人座標**：Robot: (x, y) 以公尺為單位
- **像素座標**：Pixel: (cx, cy) 影像中的中心點
- **旋轉角度**：Roll: 角度（弧度）
- **中心十字準星**：在物體中心位置繪製黃色十字標記

### **3. 白色區域資訊**
如果偵測到白色工作區域，顯示其座標：
- **White Region**: (x, y) 機器人座標系統

### **4. 視覺效果**
- **DEBUG MODE 標題**：畫面左上角顯示黃色標題
- **半透明背景**：確保文字清晰可讀
- **顏色編碼**：不同資訊使用不同顏色區分

---

## 🎮 使用方式

### **啟用 Debug Mode**
1. 打開 Web GUI（`http://<your-ip>:5000`）
2. 在攝影機視窗的右上角，找到 **🐛 Debug Mode** 按鈕
3. 點擊按鈕啟用 Debug Mode
4. 按鈕變為綠色並顯示 **🐛 Debug: ON**

### **關閉 Debug Mode**
1. 再次點擊 **🐛 Debug: ON** 按鈕
2. 按鈕恢復為灰色並顯示 **🐛 Debug Mode**

### **查看資訊**
啟用後，攝影機畫面會即時顯示：
- 座標軸（紅色 X 軸、綠色 Y 軸）
- 所有偵測到的物體資訊
- 白色區域座標（如果有）

---

## 🔧 技術實作

### **後端 (Python)**

#### **1. Debug Mode 狀態管理**
```python
debug_mode_enabled = False  # Global flag for debug mode
```

#### **2. 座標軸繪製函數**
```python
def draw_coordinate_axes(frame, M, scale=0.1):
    """Draw coordinate axes based on perspective transform"""
    - 計算原點在影像中的位置
    - 繪製 X 軸（紅色箭頭）
    - 繪製 Y 軸（綠色箭頭）
    - 標記原點
```

#### **3. Debug 資訊繪製函數**
```python
def draw_debug_info(frame, detected_objects, white_region_coords):
    """Draw debug information overlay"""
    - 繪製 DEBUG MODE 標題
    - 列出所有偵測物體的詳細資訊
    - 顯示白色區域座標
    - 在物體中心繪製十字準星
```

#### **4. Flask API 端點**
```python
@app.route('/toggle_debug_mode', methods=['POST'])
def toggle_debug_mode():
    """Toggle debug mode on/off"""
    
@app.route('/debug_mode_status', methods=['GET'])
def debug_mode_status():
    """Get current debug mode status"""
```

---

### **前端 (JavaScript)**

#### **1. 切換 Debug Mode**
```javascript
function toggleDebugMode() {
    - 切換 debugModeEnabled 狀態
    - 調用 /toggle_debug_mode API
    - 更新按鈕樣式和文字
    - 顯示提示訊息
}
```

#### **2. 檢查 Debug Mode 狀態**
```javascript
function checkDebugModeStatus() {
    - 頁面載入時檢查 debug mode 狀態
    - 從 /debug_mode_status API 獲取狀態
    - 更新按鈕顯示
}
```

---

### **前端 (CSS)**

#### **按鈕樣式**
```css
#debug-mode-btn {
    background: #6b7280;  /* 灰色 */
}

#debug-mode-btn.active {
    background: #10b981;  /* 綠色 */
    box-shadow: 0 0 0 3px rgba(16, 185, 129, 0.2);
}
```

---

## 📊 顯示資訊格式

### **畫面左上角文字疊加**
```
DEBUG MODE

1. blue cube
   Robot: (0.250, -0.050m)
   Pixel: (320, 240)
   Roll: 0.123 rad

2. red cube
   Robot: (0.180, 0.080m)
   Pixel: (450, 180)
   Roll: -0.456 rad

White Region:
  (0.200, -0.100m)
```

### **座標軸顯示**
```
        Y (綠色)
        ↑
        |
Origin (0,0) -----→ X (紅色)
```

---

## 🎨 視覺效果

### **顏色編碼**
- **DEBUG MODE 標題**：黃色 (0, 255, 255)
- **物體編號/名稱**：黃色 (255, 255, 0)
- **機器人座標**：白色 (255, 255, 255)
- **像素座標**：淺灰色 (200, 200, 200)
- **旋轉角度**：淺灰色 (200, 200, 200)
- **X 軸**：紅色 (0, 0, 255)
- **Y 軸**：綠色 (0, 255, 0)
- **原點**：白色 (255, 255, 255)
- **十字準星**：青色 (0, 255, 255)

### **圖形元素**
- **座標軸**：粗箭頭（3px）
- **原點**：實心圓（5px 半徑）
- **十字準星**：15px 大小的交叉標記
- **文字**：多種字體大小（0.4-1.0）

---

## 🔍 使用場景

### **1. 開發調試**
- 驗證物體偵測座標是否正確
- 檢查座標變換是否準確
- 確認 ArUco 標記透視變換

### **2. 系統校準**
- 檢查座標軸方向是否符合預期
- 驗證白色區域偵測位置
- 調整攝影機角度和位置

### **3. 演示展示**
- 向他人展示系統如何工作
- 顯示詳細的技術資訊
- 提供教學用途的視覺化

### **4. 問題診斷**
- 當機器人抓取位置不準確時，查看座標資訊
- 檢查旋轉角度是否正確計算
- 確認偵測框與實際物體的對應關係

---

## 🚀 效能考量

### **最佳化設計**
- Debug Mode 不影響正常模式的效能
- 僅在啟用時進行額外的繪製操作
- 使用高效的 OpenCV 繪圖函數
- 文字疊加不會顯著增加處理時間

### **建議**
- 在生產環境中建議關閉 Debug Mode
- 長時間運行時可能需要更多 CPU 資源
- 如果影像幀率下降，可以關閉 Debug Mode

---

## 📝 注意事項

### **前提條件**
- 需要正確偵測到 ArUco 標記（至少 4 個）
- 攝影機必須正常工作
- YOLO 偵測節點需要運行

### **限制**
- 如果 ArUco 標記未被偵測到，座標軸不會顯示
- 文字疊加可能會遮擋部分影像內容
- 在小螢幕上文字可能過於密集

### **故障排除**
- **座標軸不顯示**：檢查 ArUco 標記是否清晰可見
- **座標資訊缺失**：確認 YOLO 偵測節點正在發布資料
- **按鈕無反應**：檢查 Flask 伺服器日誌

---

## 🎯 總結

Debug Mode 功能提供了強大的視覺化工具，讓開發者和使用者能夠：
- ✅ 即時查看物體的精確座標
- ✅ 理解座標系統的方向和原點
- ✅ 驗證偵測和定位的準確性
- ✅ 快速診斷系統問題
- ✅ 提供教學和演示用途的詳細資訊

這個功能對於系統開發、調試和維護都非常有價值！🎉
