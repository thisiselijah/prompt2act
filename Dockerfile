FROM ubuntu:16.04

ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-get update
RUN apt install -y curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update
RUN apt-get install -y ros-kinetic-desktop-full
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
RUN apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
RUN rosdep init
RUN rosdep update

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
CMD ["/bin/bash"]