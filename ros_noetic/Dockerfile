# STEP 1: 基底映像
# 使用 OSRF 官方的 ROS Noetic 映像，名稱為 "osrf/ros:noetic-desktop-full"
FROM osrf/ros:noetic-desktop-full

# STEP 2: 環境設定與系統依賴
# 設定 SHELL，讓後續指令在 bash 環境下執行，並避免互動視窗。
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# 更新 apt 套件列表並安裝常用開發工具與 Python3 依賴。
# 這裡加入了您專案可能需要的 python3-opencv。
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-opencv \
    git \
    nano \
    vim \
    terminator \
    iputils-ping \
    net-tools \
    # --- 您可以在此加入其他需要的 apt 套件 ---
    && rm -rf /var/lib/apt/lists/*

# STEP 3: ROS 環境設定
# 基礎映像已經初始化過 rosdep，我們只需要更新即可。
RUN rosdep update

# STEP 4: Python 依賴設定 (已修正)
# 升級 pip。
RUN python3 -m pip install --no-cache-dir --upgrade pip
# 安裝 Python 套件時，使用 --ignore-installed 忽略系統已安裝的舊版本，避免衝突。
RUN python3 -m pip install --no-cache-dir --ignore-installed \
    ultralytics \
    flask \
    numpy \
    opencv-contrib-python\
    # --- 您可以在此加入其他需要的 pip 套件 ---
    && rm -rf /root/.cache/pip

# STEP 5: Catkin 工作區設定
# 建立 Catkin 工作區，路徑與您專案的習慣 (/root/catkin_ws) 保持一致。
RUN mkdir -p /root/catkin_ws/src

# 設定工作目錄
WORKDIR /root/catkin_ws

# 為了讓 entrypoint 能找到 setup.bash，先編譯一次空的工作區
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# STEP 6: Entrypoint 設定
# 複製 entrypoint 腳本到容器中並給予執行權限
COPY ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

# 設定容器啟動時的進入點
ENTRYPOINT ["/ros_entrypoint.sh"]

# 容器啟動後預設執行的指令 (進入 bash)
CMD ["bash"]