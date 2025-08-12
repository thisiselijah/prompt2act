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
    neovim \ 
    terminator \
    iputils-ping \
    net-tools \
    curl \
    build-essential \
    libssl-dev \
    zlib1g-dev \
    libbz2-dev \
    libreadline-dev \
    libsqlite3-dev \
    wget \
    llvm \
    libncurses5-dev \
    libncursesw5-dev \
    xz-utils \
    tk-dev \
    libffi-dev \
    liblzma-dev \
    python3-openssl \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# STEP 3.5: Install pyenv for Python version management
RUN curl https://pyenv.run | bash
ENV PATH="/root/.pyenv/bin:$PATH"
RUN echo 'export PATH="/root/.pyenv/bin:$PATH"' >> /root/.bashrc && \
    echo 'eval "$(pyenv init -)"' >> /root/.bashrc && \
    echo 'eval "$(pyenv virtualenv-init -)"' >> /root/.bashrc

# Install Python 3.9 using pyenv
RUN /bin/bash -c "source /root/.bashrc && pyenv install 3.9.19 && pyenv global 3.9.19"

RUN echo 'set -g mouse on' >> /root/.tmux.conf

# STEP 4: Python Dependencies
# Install all Python packages in a single layer.
# Use pyenv's Python 3.9 for packages that require it, system Python 3.8 for ROS packages
RUN /bin/bash -c "source /root/.bashrc && eval \"\$(pyenv init -)\" && \
    python -m pip install --no-cache-dir --upgrade pip && \
    python -m pip install --no-cache-dir --ignore-installed \
    opencv-contrib-python \
    openai>=1.0.0 \
    requests>=2.25.0 \
    typing_extensions>=4.0.0 \
    google-genai \
    && rm -rf /root/.cache/pip"

# Also install basic packages for system Python (for ROS compatibility)
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir \
    ultralytics \
    flask \
    numpy \
    requests \
    py_trees==0.7.* \
    && rm -rf /root/.cache/pip
# Copy the pyniryo package into the appropriate directory.
COPY pyniryo /usr/local/lib/python3.8/dist-packages/pyniryo

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

# STEP 6.5: Setup Python environment aliases and scripts
RUN echo 'alias python39="/root/.pyenv/versions/3.9.19/bin/python"' >> /root/.bashrc && \
    echo 'alias pip39="/root/.pyenv/versions/3.9.19/bin/pip"' >> /root/.bashrc && \
    echo 'export PYTHON39_PATH="/root/.pyenv/versions/3.9.19/bin/python"' >> /root/.bashrc

# Create a script to easily switch between Python versions
RUN echo '#!/bin/bash' > /usr/local/bin/use-python39 && \
    echo 'export PATH="/root/.pyenv/versions/3.9.19/bin:$PATH"' >> /usr/local/bin/use-python39 && \
    echo 'exec "$@"' >> /usr/local/bin/use-python39 && \
    chmod +x /usr/local/bin/use-python39

# STEP 7: Entrypoint
# Copy the entrypoint script and helper scripts into the container and make them executable.
COPY ros_entrypoint.sh /
COPY python_version_helper.py /usr/local/bin/
RUN chmod +x /ros_entrypoint.sh && \
    chmod +x /usr/local/bin/python_version_helper.py
ENTRYPOINT ["/ros_entrypoint.sh"]

# The default command to execute after the container starts (enter bash)

CMD ["bash"]

