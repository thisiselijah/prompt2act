# STEP 1: Base Image
FROM osrf/ros:noetic-desktop-full

# STEP 2: Environment Setup
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# STEP 3: System Dependencies & ROS
# Install system packages and update rosdep in a single layer for better caching and speed.
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-opencv \
    git \
    tmux \
    terminator \
    iputils-ping \
    net-tools \
    # --- You can add other required apt packages here ---
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# STEP 4: Python Dependencies
# Install all Python packages in a single layer.
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir --ignore-installed \
    ultralytics \
    flask \
    numpy \
    opencv-contrib-python \
    pyniryo==1.1.2 \
    py_trees==0.7.* \
    # --- You can add other required pip packages here ---
    && rm -rf /root/.cache/pip

# STEP 5: Catkin Workspace & Source Dependencies
# Create the workspace and clone source dependencies in one layer.
# Kept separate from package installation to improve caching.
RUN mkdir -p /root/catkin_ws/src && \
    git clone --branch release/0.6.x https://github.com/splintered-reality/py_trees_ros.git /root/catkin_ws/src/py_trees_ros

# STEP 6: Build Workspace
# Set the working directory and build the workspace.
# catkin_make is a slow step, so it gets its own layer. Caching this is important.
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# STEP 7: Entrypoint
# Copy the entrypoint script into the container and make it executable.
COPY ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# The default command to execute after the container starts (enter bash)
CMD ["bash"]
