# =============================================================================
# ROS Noetic Development Environment with LLM and Speech Recognition
# =============================================================================

FROM osrf/ros:noetic-desktop-full

# =============================================================================
# Environment Setup
# =============================================================================
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# =============================================================================
# System Dependencies
# =============================================================================
# Install essential system packages, development tools, and audio processing libraries
RUN apt-get update && apt-get install -y \
    # Core development tools
    python3-pip \
    python3-rosdep \
    python3-opencv \
    git \
    neovim \
    terminator \
    curl \
    wget \
    # Network utilities
    iputils-ping \
    net-tools \
    # Python build dependencies
    build-essential \
    libssl-dev \
    zlib1g-dev \
    libbz2-dev \
    libreadline-dev \
    libsqlite3-dev \
    llvm \
    libncurses5-dev \
    libncursesw5-dev \
    xz-utils \
    tk-dev \
    libffi-dev \
    liblzma-dev \
    python3-openssl \
    # Audio processing dependencies (for speech recognition)
    ffmpeg \
    libavcodec-extra \
    portaudio19-dev \
    python3-pyaudio \
    # Visualization tools
    graphviz \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# Terminal Multiplexer (Zellij)
# =============================================================================
# Install latest Zellij for enhanced terminal experience
RUN wget https://github.com/zellij-org/zellij/releases/latest/download/zellij-x86_64-unknown-linux-musl.tar.gz \
 && tar -xvf zellij-x86_64-unknown-linux-musl.tar.gz \
 && rm zellij-x86_64-unknown-linux-musl.tar.gz \
 && mv zellij /usr/local/bin/

# =============================================================================
# Python Version Management (PyEnv)
# =============================================================================
# Install pyenv for managing multiple Python versions
RUN curl https://pyenv.run | bash
ENV PATH="/root/.pyenv/bin:$PATH"
RUN echo 'export PATH="/root/.pyenv/bin:$PATH"' >> /root/.bashrc && \
    echo 'eval "$(pyenv init -)"' >> /root/.bashrc && \
    echo 'eval "$(pyenv virtualenv-init -)"' >> /root/.bashrc

# Install Python 3.9.19 for LLM and speech recognition compatibility
RUN /bin/bash -c "source /root/.bashrc && pyenv install 3.9.19 && pyenv global 3.9.19"

# =============================================================================
# Python Dependencies (PyEnv Python 3.9)
# =============================================================================
# Install Python packages for LLM and speech recognition using PyEnv Python 3.9
RUN /bin/bash -c "source /root/.bashrc && eval \"\$(pyenv init -)\" && \
    python -m pip install --no-cache-dir --upgrade pip && \
    python -m pip install --no-cache-dir --ignore-installed \
    # ROS compatibility packages
    catkin_pkg \
    empy \
    rospkg \
    # LLM service dependencies
    openai>=1.0.0 \
    requests>=2.25.0 \
    typing_extensions>=4.0.0 \
    google-genai \
    # Speech recognition dependencies
    SpeechRecognition \
    pydub \
    pyaudio \
    && rm -rf /root/.cache/pip"

# =============================================================================
# Python Dependencies (System Python 3.8)
# =============================================================================
# Install packages for ROS compatibility and computer vision using system Python
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir --ignore-installed \
    # Computer vision and detection
    ultralytics \
    opencv-contrib-python \
    numpy \
    # Web interface
    flask \
    requests \
    # Behavior tree framework
    py_trees==0.7.* \
    pydot \
    && rm -rf /root/.cache/pip
# =============================================================================
# Robot Control Library (PyNiryo)
# =============================================================================
# Copy the PyNiryo package for Niryo robot control
COPY pyniryo /usr/local/lib/python3.8/dist-packages/pyniryo

# =============================================================================
# ROS Workspace Setup
# =============================================================================
# Create catkin workspace and clone behavior tree dependencies
RUN mkdir -p /root/catkin_ws/src && \
    git clone --branch release/0.6.x https://github.com/splintered-reality/py_trees_ros.git /root/catkin_ws/src/py_trees_ros

# =============================================================================
# Build ROS Workspace
# =============================================================================
# Build the catkin workspace (this is a slow step, so it gets its own layer for caching)
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# =============================================================================
# Python Environment Aliases and Scripts
# =============================================================================
# Setup convenient aliases for Python version management
RUN echo 'alias python39="/root/.pyenv/versions/3.9.19/bin/python"' >> /root/.bashrc && \
    echo 'alias pip39="/root/.pyenv/versions/3.9.19/bin/pip"' >> /root/.bashrc && \
    echo 'export PYTHON39_PATH="/root/.pyenv/versions/3.9.19/bin/python"' >> /root/.bashrc

# Create utility script for switching to Python 3.9 environment
RUN echo '#!/bin/bash' > /usr/local/bin/use-python39 && \
    echo 'export PATH="/root/.pyenv/versions/3.9.19/bin:$PATH"' >> /usr/local/bin/use-python39 && \
    echo 'exec "$@"' >> /usr/local/bin/use-python39 && \
    chmod +x /usr/local/bin/use-python39

# =============================================================================
# Container Entry Point and Configuration
# =============================================================================
# Copy and setup entry point script and helper utilities
COPY ros_entrypoint.sh /
COPY python_version_helper.py /usr/local/bin/
RUN chmod +x /ros_entrypoint.sh && \
    chmod +x /usr/local/bin/python_version_helper.py
ENTRYPOINT ["/ros_entrypoint.sh"]

# =============================================================================
# Terminal Configuration
# =============================================================================
# Configure Zellij terminal multiplexer
RUN mkdir -p /root/.config/zellij
RUN echo 'default_shell "/bin/bash"' > /root/.config/zellij/config.kdl

# =============================================================================
# Default Command
# =============================================================================
# Start bash shell when container runs
CMD ["bash"]

