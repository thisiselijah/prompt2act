# LLM Node 使用指南

本指南說明如何在 ROS 中使用 LLM（Large Language Model，大型語言模型）節點進行文字生成和行為樹配置。

## 目錄
- [先決條件](#先決條件)
- [啟動 LLM Node](#啟動-llm-node)
- [ROS 主題](#ros-主題)
- [ROS 服務](#ros-服務)
- [使用範例](#使用範例)
- [行為樹整合](#行為樹整合)
- [故障排除](#故障排除)

## 先決條件

### 1. 環境設定
在使用 LLM 節點之前，請確保您的 ROS 環境已正確配置：

```bash
# 在 Docker 容器內
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
```

### 2. API 金鑰配置
為 LLM 提供者設定 API 金鑰：

```bash
# 適用於 Google Gemini（推薦）
export GEMINI_API_KEY="your_gemini_api_key_here"

# 替代方案：Google API 金鑰
export GOOGLE_API_KEY="your_google_api_key_here"

# 適用於 OpenAI（可選）
export OPENAI_API_KEY="your_openai_api_key_here"
```

### 3. 建置工作區
確保工作區已建置：

```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
```

## 啟動 LLM Node

### 選項 1：直接執行
```bash
# 使用預設 Gemini 提供者啟動
rosrun llm llm_node.py

# 使用特定提供者啟動
rosrun llm llm_node.py _provider:=gemini
rosrun llm llm_node.py _provider:=openai
```

### 選項 2：使用啟動檔案
```bash
# 啟動包含 LLM 節點的所有節點
roslaunch launcher all_nodes.launch llm_provider:=gemini

# 或使用 OpenAI
roslaunch launcher all_nodes.launch llm_provider:=openai
```

### 選項 3：個別節點啟動
```bash
# 建立簡單的啟動檔案或直接執行
roscore &
rosrun llm llm_node.py _provider:=gemini
```

## ROS 主題

### 發佈者
LLM 節點在這些主題上發佈回應：

| 主題 | 訊息類型 | 描述 |
|------|----------|------|
| `/llm_response` | `std_msgs/String` | 來自 LLM 的文字回應 |
| `/llm_json_response` | `std_msgs/String` | 來自 LLM 的 JSON 回應 |

### 訂閱者
LLM 節點監聽這些主題：

| 主題 | 訊息類型 | 描述 |
|------|----------|------|
| `/llm_prompt` | `std_msgs/String` | 發送給 LLM 的文字提示 |
| `/llm_json_prompt` | `std_msgs/String` | JSON 特定的提示 |

### 主題使用範例

#### 文字生成
```bash
# 發送文字提示
rostopic pub /llm_prompt std_msgs/String "data: '解釋機器人如何運作'"

# 監聽回應
rostopic echo /llm_response
```

#### JSON 生成
```bash
# 發送 JSON 提示
rostopic pub /llm_json_prompt std_msgs/String "data: '為機器人移動生成 JSON 配置'"

# 監聽 JSON 回應
rostopic echo /llm_json_response
```

## ROS 服務

LLM 節點提供自訂服務介面以進行結構化通訊：

### 服務訊息格式

#### LLMQuery 服務
```
# 請求
string prompt

# 回應
bool success
string response
string error_message
```

#### LLMJsonQuery 服務
```
# 請求
string prompt
string schema  # 可選的 JSON 結構描述以進行驗證

# 回應
bool success
string json_response
string error_message
```

#### GenerateBehaviorTree 服務
```
# 請求
string task_description

# 回應
bool success
string behavior_tree_json
string error_message
```

#### LLMStatus 服務
```
# 請求
# (空)

# 回應
bool success
string provider_name
bool is_available
string status_message
```

### 可用服務

| 服務 | 服務類型 | 描述 |
|------|----------|------|
| `/llm_query` | `llm/LLMQuery` | 帶提示的直接文字查詢 |
| `/llm_json_query` | `llm/LLMJsonQuery` | 帶可選結構描述的 JSON 查詢 |
| `/generate_behavior_tree` | `llm/GenerateBehaviorTree` | 生成行為樹 JSON |
| `/llm_status` | `llm/LLMStatus` | 檢查 LLM 提供者狀態 |

### 服務使用範例

#### 文字查詢服務
```bash
# 使用自訂提示進行直接文字查詢
rosservice call /llm_query "prompt: '什麼是人工智慧？'"
```

#### JSON 查詢服務
```bash
# 使用提示和可選結構描述進行直接 JSON 查詢
rosservice call /llm_json_query "prompt: '建立包含機器人導航命令的 JSON 物件' schema: ''"

# 使用 JSON 結構描述驗證（適用於 Gemini）
rosservice call /llm_json_query "prompt: '生成機器人配置' schema: '{\"type\": \"object\", \"properties\": {\"joints\": {\"type\": \"array\"}}}'"
```

#### 行為樹生成
```bash
# 方法 1：使用適當的 YAML 多行格式（推薦）
rosservice call /generate_behavior_tree "
task_description: '拿起紅色立方體並將其放置在藍色桌子上'
"

# 方法 2：使用內聯 YAML 並保持適當結構
rosservice call /generate_behavior_tree "{task_description: '按顏色排序物件'}"

# 方法 3：使用傳統 ROS 服務呼叫格式
rosservice call /generate_behavior_tree -- "task_description: '拿起物件'"

# 方法 4：使用預設任務生成
rosservice call /generate_behavior_tree "task_description: ''"
```

**注意**：行為樹組裝現在是自動的 - 不需要指定 auto_assemble 參數！

#### 狀態檢查
```bash
# 檢查 LLM 提供者狀態和可用性
rosservice call /llm_status
```

## 使用範例

### 範例 1：基本文字生成工作流程

```bash
# 終端機 1：啟動 LLM 節點
rosrun llm llm_node.py _provider:=gemini

# 終端機 2：發送提示並監控回應
rostopic pub /llm_prompt std_msgs/String "data: '描述機器人三定律'"
rostopic echo /llm_response
```

### 範例 2：JSON 配置生成

```bash
# 終端機 1：啟動 LLM 節點
rosrun llm llm_node.py

# 終端機 2：生成 JSON 配置
rostopic pub /llm_json_prompt std_msgs/String "data: '為 6-DOF 機器人手臂建立包含關節限制的 JSON 配置'"

# 終端機 3：監控 JSON 回應
rostopic echo /llm_json_response
```

### 範例 3：行為樹生成和自動組裝

```bash
# 終端機 1：啟動行為樹節點
rosrun behavior_tree behavior_tree_node.py

# 終端機 2：啟動 LLM 節點
rosrun llm llm_node.py _provider:=gemini

# 終端機 3：生成綜合機器人任務（自動組裝是自動的）
rosservice call /generate_behavior_tree "
task_description: '機器人應該掃描紅色和藍色積木，將它們排序到單獨區域，然後返回原位'
"

# 預期回應格式：
# success: True
# behavior_tree_json: "{ ... 包含 detect_objects、pick_up、place_down、move_to_home 的 JSON 配置 ... }"
# error_message: ""

# 終端機 4：監控即時行為樹執行
rostopic echo /behavior_tree_status

# 終端機 5：生成夾爪測試序列
rosservice call /generate_behavior_tree "
task_description: '通過多次開啟和關閉來測試夾爪功能'
"
```

### 範例 4：增強行為樹工作流程與完整機器人整合

```bash
# 終端機 1：首先啟動行為樹節點
rosrun behavior_tree behavior_tree_node.py

# 終端機 2：啟動 YOLO 檢測節點以進行視覺處理
rosrun yolo_detection yolo_detection_node.py

# 終端機 3：啟動機器人控制節點
rosrun robot_control robot_control_node.py

# 終端機 4：啟動 LLM 節點
rosrun llm llm_node.py _provider:=gemini

# 終端機 5：生成並執行完整機器人工作流程（自動組裝是自動的）
rosservice call /generate_behavior_tree "
task_description: '按顏色排序物件：拿起紅色積木並將它們放置在紅色區域坐標 (0.12, -0.18, 0.18)，拿起藍色積木並將它們放置在藍色區域 (0.18, -0.12, 0.18)，然後返回原位'
"

# 終端機 6：監控即時執行狀態
rostopic echo /behavior_tree_status

# 終端機 7：監控 YOLO 檢測
rostopic echo /yolo_detected_targets

# 任務變體範例：
# 簡單拿起和放置：
rosservice call /generate_behavior_tree "
task_description: '拿起任何物件並將其放置在坐標 (0.15, -0.15, 0.18)'
"

# 夾爪控制序列：
rosservice call /generate_behavior_tree "
task_description: '開啟夾爪，關閉夾爪，然後移動到原位'
"

# 複雜多步驟任務：
rosservice call /generate_behavior_tree "
task_description: '檢測所有物件，一一拿起它們，將它們堆疊在位置 (0.20, -0.10, 0.18)，並確保完成時夾爪是開啟的'
"
```

## 行為樹整合

LLM 節點可以生成與行為樹節點相容的行為樹，並在 JSON 生成後自動組裝它們。

### 自動組裝功能
- **始終啟用**：LLM 節點在 JSON 生成後自動呼叫 `/assemble_behavior_tree` 服務
- **無縫工作流程**：單一服務呼叫從任務描述轉換為活躍的行為樹
- **強健的錯誤處理**：如果組裝失敗，JSON 生成仍會成功並顯示警告訊息
- **簡化的介面**：不需要指定組裝參數 - 它是自動的

### 整合流程
1. **JSON 生成**：LLM 根據任務描述生成行為樹 JSON
2. **自動組裝**：生成的 JSON 自動發送到 `/assemble_behavior_tree`
3. **即時執行**：行為樹立即開始執行並提供即時狀態更新
4. **JSON 狀態發佈**：執行期間通過 `/behavior_tree_status` 主題發佈樹狀結構狀態
5. **自動終止**：當根節點達到 SUCCESS 或 FAILURE 狀態時，樹狀結構終止
6. **準備下一個任務**：系統等待下一個行為樹生成請求

### 機器人整合功能
- **YOLO 視覺整合**：`detect_objects` 訂閱 `/yolo_detected_targets` 以進行即時物件檢測
- **機器人控制整合**：所有機器人行為使用 `/arm_command` 服務進行 Niryo 機器人控制
- **黑板資料共享**：在行為之間共享檢測到的物件和操作資料
- **坐標系統**：機器人坐標以公尺為單位，具有可配置的放置位置
- **錯誤處理**：優雅的失敗恢復並提供詳細的錯誤報告

生成的 JSON 遵循以下結構：

```json
{
  "type": "sequence",
  "name": "MainTask",
  "children": [
    {
      "type": "detect_objects",
      "name": "DetectObjects"
    },
    {
      "type": "open_gripper",
      "name": "PrepareGripper"
    },
    {
      "type": "pick_up",
      "name": "PickUpObject"
    },
    {
      "type": "place_down",
      "name": "PlaceObject",
      "place_x": 0.15,
      "place_y": -0.15,
      "place_z": 0.18
    },
    {
      "type": "move_to_home",
      "name": "ReturnHome"
    }
  ]
}
```

### 可用行為類型
- `detect_objects`：物件檢測行為
- `pick_up`：物件抓取行為
- `place_down`：物件放置行為
- `sequence`：按順序執行子項（全部必須成功）
- `selector`：嘗試子項直到一個成功（第一個成功完成選擇器）

## 故障排除

### 常見問題

#### 1. 找不到 API 金鑰
```bash
# 錯誤："Failed to initialize provider"
# 解決方案：設定 API 金鑰
export GEMINI_API_KEY="your_api_key"
```

#### 2. 節點無法啟動
```bash
# 錯誤："No module named 'setup'"
# 解決方案：確保您在正確的目錄中且工作區已建置
cd /root/catkin_ws
source devel/setup.bash
```

#### 3. LLM 沒有回應
```bash
# 檢查提供者是否可用
rosservice call /llm_status

# 檢查 ROS 主題
rostopic list | grep llm
```

#### 4. 服務呼叫錯誤
```bash
# 錯誤：expected <block end>, but found '<scalar>' - YAML 解析錯誤
# 問題：不正確的服務呼叫語法
# 
# 解決方案（選擇一個）：
# 方法 1 - 多行 YAML（推薦）：
rosservice call /generate_behavior_tree "
task_description: '拿起紅色立方體'
"

# 方法 2 - JSON 風格內聯：
rosservice call /generate_behavior_tree "{task_description: '拿起紅色立方體'}"

# 方法 3 - 傳統 ROS 格式：
rosservice call /generate_behavior_tree -- "task_description: '拿起立方體'"

# 呼叫 /generate_behavior_tree 時錯誤："No field name [data]"
# 解決方案：使用正確的欄位名稱 'task_description'
# 錯誤：rosservice call /generate_behavior_tree "data: ''"
# 正確：rosservice call /generate_behavior_tree "task_description: ''"

# 錯誤："Incompatible arguments to call service"
# 解決方案：使用以下命令檢查服務訊息格式：
rosservice info /generate_behavior_tree

# 錯誤："JSON generated but assembly failed: Behavior tree assembly service not available"
# 解決方案：在 LLM 節點之前啟動行為樹節點：
rosrun behavior_tree behavior_tree_node.py
# 然後重新啟動 LLM 節點：
rosrun llm llm_node.py _provider:=gemini

# 注意：行為樹組裝服務使用自訂服務介面：
# /assemble_behavior_tree (behavior_tree/AssembleBehaviorTree)
# 請求：string behavior_tree_json
# 回應：bool success, string message
```

#### 5. JSON 結構描述驗證錯誤
```bash
# 錯誤："GenerateContentRequest.generation_config.response_schema.properties..."
# 這表示 Gemini API 的結構描述驗證問題
# 節點會自動回退到不使用結構描述驗證的生成

# 錯誤："Invalid JSON generated: Expecting value: line 1 column 1 (char 0)"
# 這表示 LLM 返回了空或格式錯誤的 JSON
# 解決方案：
# 1. 檢查 API 金鑰是否有效
rosservice call /llm_status

# 2. 使用更簡單的任務描述嘗試
rosservice call /generate_behavior_tree "task_description: '拿起物件'"

# 3. 改用 OpenAI 提供者
rosrun llm llm_node.py _provider:=openai
```

#### 6. 空回應問題
```bash
# 錯誤："Invalid JSON generated"
# 這可能發生在使用 OpenAI 提供者時。嘗試使用 Gemini 以獲得更好的 JSON 相容性
rosrun llm llm_node.py _provider:=gemini
```

### 除錯命令

```bash
# 檢查節點是否正在運行
rosnode list | grep llm

# 監控節點輸出
rosrun llm llm_node.py _provider:=gemini

# 檢查主題
rostopic list | grep llm

# 檢查服務及其訊息格式
rosservice list | grep llm
rosservice info /llm_query
rosservice info /generate_behavior_tree

# 測試基本功能
rosservice call /llm_status

# 使用正確語法測試每個服務（使用適當的 YAML 格式）
rosservice call /llm_query "prompt: 'Hello'"
rosservice call /generate_behavior_tree "
task_description: '測試任務'
auto_assemble: true
"
rosservice call /generate_behavior_tree "{task_description: '測試任務', auto_assemble: false}"
```

### 日誌等級
要獲得更詳細的日誌記錄：

```bash
# 將 ROS 日誌等級設定為 DEBUG
export ROSCONSOLE_CONFIG_FILE=/path/to/debug/config
# 或修改節點以包含更多日誌記錄
```

## 進階使用

### 增強任務描述功能

LLM 節點現在支援生成複雜機器人行為，具有以下增強功能：

#### 支援的任務類型
- **物件檢測和排序**："將紅色和藍色物件排序到單獨區域"
- **精確操作**："拿起物件並將其放置在坐標 (0.15, -0.15, 0.18)"
- **夾爪控制**："開啟夾爪，拿起物件，關閉夾爪，移動到原位"
- **多步驟工作流程**："檢測物件，一一拿起它們，將它們堆疊在位置 X"
- **條件行為**："如果檢測到紅色物件，將其放置在紅色區域，否則放置在藍色區域"

#### 任務描述最佳實務
1. **具體說明**：需要精確度時包含確切坐標
2. **提及顏色**："紅色積木"、"藍色物件" 用於基於顏色的排序
3. **包含結束狀態**："返回原位"、"確保夾爪是開啟的"
4. **指定區域**："紅色區域"、"藍色區域"、"堆疊區域"
5. **使用動作動詞**："檢測"、"拿起"、"放置"、"排序"、"堆疊"

#### 生成的行為功能
- **視覺整合**：自動 YOLO 檢測整合以進行物件辨識
- **坐標控制**：使用可配置的 place_x、place_y、place_z 進行精確定位
- **夾爪管理**：自動夾爪控制與開啟/關閉行為
- **錯誤恢復**：強健的錯誤處理與優雅的失敗模式
- **即時狀態**：通過 JSON 狀態發佈進行即時執行監控

### 自訂提示範本
您可以通過編輯 `llm_node.py` 中的全域常數來修改行為樹提示範本：

```python
BEHAVIOR_TREE_PROMPT_TEMPLATE = """
您的自訂提示範本在此...
{task_description}
"""
```

### 與其他節點整合
LLM 節點設計為與以下節點配合使用：
- **行為樹節點**（用於自動化任務規劃和執行）
- **機器人控制節點**（用於物理執行）
- **語音辨識節點**（用於語音命令）

### 增強工作流程
1. **任務輸入**：通過服務呼叫接收自然語言任務描述
2. **LLM 處理**：Gemini/OpenAI 生成行為樹 JSON 結構
3. **自動組裝**：JSON 自動組裝成可執行的行為樹
4. **視覺化**：樹狀結構以 PNG 格式儲存在 `/frames` 目錄中
5. **執行**：行為樹立即運行並提供即時狀態更新

### 服務依賴項
- **必要**：來自行為樹節點的 `/assemble_behavior_tree` 服務 (behavior_tree/AssembleBehaviorTree) 用於自動組裝
- **可選**：用於增強除錯的視覺化服務

**更新的服務介面**：
```
# /assemble_behavior_tree 服務
string behavior_tree_json    # 要組裝的 JSON 配置
---
bool success                 # 組裝成功狀態
string message              # 狀態訊息或錯誤詳細資訊
```

### 效能考量
- Gemini 提供者通常提供更好的 JSON 相容性
- 回應時間根據提示複雜度而變化
- 考慮對重複查詢進行快取
- 自動組裝增加最小的額外負荷（~100ms）以進行立即執行

## 新功能

### 自動組裝整合（最新更新）

LLM 節點現在具有與行為樹組裝服務的無縫整合：

#### 主要優點
- **一步工作流程**：單一服務呼叫從任務描述轉換為運行的行為樹
- **自動錯誤恢復**：如果組裝服務不可用，優雅地回退
- **靈活控制**：可選的 `auto_assemble` 參數用於手動控制
- **增強回饋**：JSON 生成和組裝的詳細狀態訊息

#### 使用模式

**完整自動化（推薦）**：
```bash
rosservice call /generate_behavior_tree "
task_description: '您的任務在此'
auto_assemble: true
"
```

**僅 JSON 生成**：
```bash
rosservice call /generate_behavior_tree "{task_description: '您的任務在此', auto_assemble: false}"
```

**向後相容性**：
```bash
# 如果省略，auto_assemble 預設為 true
rosservice call /generate_behavior_tree "task_description: '您的任務在此'"
```

#### 錯誤處理
- **組裝不可用**：返回帶警告訊息的 JSON，允許稍後手動組裝
- **JSON 無效**：清除錯誤訊息，包含原始和清理的回應
- **服務重新連線**：組裝服務連線的自動重試邏輯

---

## 測試

### 自動化測試
使用包含的綜合測試腳本：

```bash
# 在 Docker 容器中
cd /root/catkin_ws
source devel/setup.bash

# 首先啟動行為樹節點（自動組裝測試所需）
rosrun behavior_tree behavior_tree_node.py &

# 啟動 LLM 節點
rosrun llm llm_node.py _provider:=gemini &

# 運行綜合測試
rosrun llm test_behavior_tree_generation.py
```

測試腳本包括：
- **啟用自動組裝的測試**：驗證端到端工作流程
- **僅 JSON 測試**：驗證不進行組裝的生成
- **錯誤處理測試**：確認優雅的失敗模式
- **多種任務類型**：測試各種複雜度等級

### 手動測試
```bash
# 測試基本功能
rosservice call /llm_status

# 測試僅 JSON 生成
rosservice call /generate_behavior_tree "{task_description: '拿起物件', auto_assemble: false}"

# 測試完整工作流程
rosservice call /generate_behavior_tree "
task_description: '排序紅色和藍色積木'
auto_assemble: true
"

# 檢查行為樹狀態
rostopic echo /behavior_tree_status
```

有關更多資訊或問題，請參考專案文件或在儲存庫中建立問題。

## 系統架構

### 元件整合
LLM 節點是具有以下元件的綜合機器人系統的一部分：

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   LLM Node      │────│ Behavior Tree    │────│  Robot Control  │
│ (Task Planning) │    │   (Execution)    │    │  (Hardware)     │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         │              ┌────────────────┐              │
         └──────────────│  YOLO Detection │──────────────┘
                        │   (Vision)     │
                        └────────────────┘
```

### 資料流程
1. **任務輸入**：通過 `/llm_service` 接收自然語言描述
2. **行為生成**：LLM 建立行為樹 JSON 結構
3. **樹狀組裝**：行為樹節點組裝並執行行為
4. **視覺整合**：YOLO 檢測提供物件坐標
5. **機器人執行**：物理機器人執行操作任務
6. **狀態監控**：通過 JSON 發佈進行即時回饋

### 部署配置

#### 完整系統啟動
```bash
# 啟動所有系統元件
roslaunch niryo_web_interface all_nodes.launch
```

#### 元件狀態監控
```bash
# 監控行為樹執行
rostopic echo /behavior_tree_status

# 監控檢測到的物件
rostopic echo /detected_objects_json

# 檢查 LLM 服務可用性
rosservice info /llm_service
```

#### 系統健康檢查
```bash
# 驗證所有節點是否正在運行
rosnode list

# 檢查服務可用性
rosservice list | grep -E "(llm|behavior|robot)"

# 監控主題活動
rostopic hz /behavior_tree_status
```

## 依賴項

### 必要的 ROS 套件
- `rospy`：ROS Python 繫結
- `std_msgs`：標準 ROS 訊息類型
- `behavior_tree`：自訂行為樹套件（用於自動組裝）

### Python 依賴項
- `google-generativeai`：Google Gemini API
- `openai`：OpenAI API
- `json`：JSON 解析
- `re`：正規表示式

### 系統需求
- ROS Noetic
- Python 3.8+
- 網際網路連線用於 LLM API
- 所選提供者的有效 API 金鑰

````
