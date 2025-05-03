### Docker
---

- *BUILD*: `docker build -t ros-kinetic-ubuntu1604 .`
- *RUN*: `docker run -it --rm -v ./ros_kinetic_ws/src:/root/catkin_ws/src ros-kinetic-ubuntu1604`

### Compile
---
`catkin_make`

`source devel/setup.bash`

然後執行你的 rosrun / roslaunch 等指令