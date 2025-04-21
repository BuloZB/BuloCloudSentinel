FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-dev \
    python3-pip \
    libopencv-dev \
    libeigen3-dev \
    libpcl-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    libboost-all-dev \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-cv-bridge \
    ros-humble-pcl-conversions \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR /app

# Clone FAST-LIO repository
RUN git clone https://github.com/hku-mars/FAST_LIO.git /app/src/fast_lio

# Clone ORB-SLAM3 repository
RUN git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git /app/src/orb_slam3

# Build FAST-LIO
RUN cd /app/src/fast_lio && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc)

# Build ORB-SLAM3
RUN cd /app/src/orb_slam3 && \
    chmod +x build.sh && \
    ./build.sh

# Install Python dependencies
COPY requirements.txt /app/
RUN pip3 install --no-cache-dir -r requirements.txt

# Copy application code
COPY . /app/

# Set environment variables for ROS2
ENV ROS_DISTRO=humble
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/app/src/orb_slam3/lib:/app/src/fast_lio/lib
ENV PYTHONPATH=$PYTHONPATH:/app

# Set entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 slam_node.py"]
