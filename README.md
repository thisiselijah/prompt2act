# prompt2act 

## Project Introduction / 專案介紹

prompt2act 是一個綜合性的機器人框架，將大型語言模型 (LLM) 與 ROS（機器人作業系統）整合，實現智慧型機器人控制與自動化。本專案讓機器人能夠理解自然語言指令，並透過精密的行為樹架構將其轉換為可執行的行為。本系統專門為 **Niryo One 6 軸機械手臂**設計，這是一款教育和研究用的協作機器人。

**主要特色：**
- **多重 LLM 支援**：整合 OpenAI GPT、Google Gemini 和 Ollama（本機模型）。
- **行為樹框架**：使用 py_trees 進行強健的決策制定和任務執行
- **電腦視覺**：基於 YOLO 的物體偵測和識別功能
- **語音識別**：語音指令處理，實現自然的人機互動
- **網頁介面**：使用者友善的網頁儀表板，用於機器人監控和控制
- **Docker 支援**：容器化部署，便於設定和移植

本系統專為研究和教育目的設計，提供一個靈活的平台來實驗 AI 驅動的機器人應用。

---

prompt2act is a comprehensive robotics framework that integrates Large Language Models (LLMs) with ROS (Robot Operating System) for intelligent robot control and automation. This project enables robots to understand natural language commands and convert them into actionable behaviors through a sophisticated behavior tree architecture. The system is specifically designed for the **Niryo One 6-axis robotic arm**, an educational and research-focused collaborative robot.

**Key Features:**
- **Multi-LLM Support**: Integrates OpenAI GPT, Google Gemini, and Ollama (local).
- **Behavior Tree Framework**: Uses py_trees for robust decision-making and task execution
- **Computer Vision**: YOLO-based object detection and recognition capabilities
- **Speech Recognition**: Voice command processing for natural human-robot interaction
- **Web Interface**: User-friendly web dashboard for robot monitoring and control
- **Docker Support**: Containerized deployment for easy setup and portability

The system is designed for research and educational purposes, providing a flexible platform for experimenting with AI-driven robotics applications.



## Architecture / 系統架構

The project follows a modular ROS-based architecture with the following main components:
本專案採用模組化的 ROS 架構，包含以下主要組件：

- **LLM Node**: Natural language processing and response generation / 自然語言處理和回應生成
- **Behavior Tree**: Task planning and execution logic / 任務規劃和執行邏輯
- **Robot Control**: Hardware interface and motion control / 硬體介面和運動控制
- **Vision System**: YOLO-based object detection / 基於 YOLO 的物體偵測
- **Speech Recognition**: Voice command processing / 語音指令處理
- **Web Interface**: Real-time monitoring and control / 即時監控和控制

### References
---
- *https://hackmd.io/@NCTU-auv/B1_ErlCn3*
- *https://github.com/splintered-reality/py_trees/tree/release/0.7.x*
- *https://github.com/splintered-reality/py_trees_ros/tree/release/0.6.x*
- *https://github.com/splintered-reality/rqt_py_trees/tree/release/0.4.x*
- *https://github.com/splintered-reality/py_trees_msgs/tree/release/0.3.x*
- *https://gitcode.com/gh_mirrors/ni/niryo_one_ros/?utm_source=artical_gitcode&index=bottom&type=card&webUrl*
