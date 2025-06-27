FROM ubuntu:16.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    && sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen \
    && locale-gen \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 LANGUAGE=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US.UTF-8 
ENV LC_ALL=en_US.UTF-8
ENV PYTHONIOENCODING=UTF-8

RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-get update
RUN apt install -y curl git python-pip
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update
RUN apt-get install -y ros-kinetic-desktop-full
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
RUN apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
RUN rosdep init
RUN rosdep update

RUN apt-get update && apt-get install -y ros-kinetic-robot-state-publisher ros-kinetic-moveit ros-kinetic-rosbridge-suite ros-kinetic-joy ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-tf2-web-republisher
RUN curl -sSL https://bootstrap.pypa.io/pip/2.7/get-pip.py -o get-pip.py && \
    python get-pip.py --no-cache-dir && \
    rm get-pip.py
RUN pip install --no-cache-dir jsonpickle

RUN mkdir -p /root/catkin_ws/src/niryo_one_ros
RUN cd /root/catkin_ws/src/niryo_one_ros && git clone https://github.com/NiryoRobotics/niryo_one_ros.git .
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && cd /root/catkin_ws && catkin_make"
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /root/catkin_ws
CMD ["/bin/bash"]