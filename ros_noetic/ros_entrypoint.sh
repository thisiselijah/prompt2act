#!/bin/bash
set -e

# 來源 (Source) ROS Noetic 的主要設定檔
source "/opt/ros/noetic/setup.bash"

# 檢查並來源 Catkin 工作區的設定檔 (路徑與 Dockerfile 中設定的一致)
if [ -f "/root/catkin_ws/devel/setup.bash" ]; then
  echo "Sourcing /root/catkin_ws/devel/setup.bash"
  source "/root/catkin_ws/devel/setup.bash"
fi

# 執行傳遞給容器的指令 (例如 'bash')
exec "$@"