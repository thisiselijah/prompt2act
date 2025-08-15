---
title: "Quickstart"
weight: 1
# bookFlatSection: false
# bookToc: true
# bookHidden: false
# bookCollapseSection: false
# bookComments: false
# bookSearchExclude: false
# bookHref: ''
---

# Quickstart

1. Clone the project and go to the directory: `prompt2act`.
```bash
$ git clone https://github.com/thisiselijah/prompt2act.git && cd prompt2act/ros_noetic/
```
3. Build the docker image.
```bash
$ docker build -t ros-noetic-dev .
```
4. Run the container.
```bash
$ docker run -it --rm --name <container_name> --net=host --device=/dev/video0:/dev/video0 -v ./catkin_ws/src:/root/catkin_ws/src ros-noetic-

# for test
$ docker run -it --rm -v ./catkin_ws/src:/root/catkin_ws/src ros-noetic-dev
```
5. Use  `zellij` as terminal multiplexer.
```bash
$ zellij
```

6. Build the ROS environments.
```bash
$ source devel/setup.bash && catkin_make
```

7. Set llm API key
```bash
$ export GEMIINI_API_KEY = "<YOUR API KEY>"
```

8. Run the system
```bash
roslaunch launcher all_nodes.launch 
```



