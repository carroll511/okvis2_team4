# Multi-stage Dockerfile for OKVIS2 with ROS2 and deep learning support
# Supports FlowFormer and MAC-VO integration

# Stage 1: Base ROS2 Jazzy image with CUDA support
# Note: Using Ubuntu 22.04 as base since 24.04 CUDA images may not be available
# ROS2 Jazzy can be installed on Ubuntu 22.04 as well
FROM nvidia/cuda:12.1.0-devel-ubuntu22.04 AS base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
# ROS2 Humble for Ubuntu 22.04 (Jazzy requires Ubuntu 24.04)
ENV ROS_DISTRO=humble
# Note: To use ROS2 Jazzy, you need Ubuntu 24.04 base image
# For Ubuntu 22.04, we use Humble which is the LTS version
ENV ROS_VERSION=2
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV LD_LIBRARY_PATH=${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    wget \
    git \
    build-essential \
    cmake \
    pkg-config \
    unzip \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Humble (Ubuntu 22.04 compatible)
# Using official ROS2 installation method
RUN apt-get update && apt-get install -y \
    locales \
    software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8 \
    && add-apt-repository universe \
    && apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -o /usr/share/keyrings/ros-archive-keyring.asc \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.asc] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null \
    && apt-get update \
    && apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true \
    && rosdep update

# Install OKVIS2 core dependencies
RUN apt-get update && apt-get install -y \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    libboost-dev \
    libboost-filesystem-dev \
    libopencv-dev \
    libopenblas-openmp-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 specific dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-rosbag2 \
    ros-${ROS_DISTRO}-rosbag2-cpp \
    ros-${ROS_DISTRO}-rosbag2-storage \
    && rm -rf /var/lib/apt/lists/*

# Install RealSense SDK (optional but useful)
# RealSense SDK can be installed from source if apt repo is unavailable
# For now, we'll skip it and it can be installed manually if needed
# To install RealSense SDK, you can:
# 1. Build from source: https://github.com/IntelRealSense/librealsense
# 2. Or use the apt repository if it becomes available
RUN echo "RealSense SDK installation skipped - can be installed manually if needed" || true

# Install Python and pip
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip
RUN pip3 install --upgrade pip setuptools wheel

# Install PyTorch with CUDA support (for FlowFormer/MAC-VO)
# Using PyTorch 2.1+ with CUDA 12.1
# Using --ignore-installed to handle distutils-installed packages
RUN pip3 install --ignore-installed torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# Install common deep learning and computer vision libraries
RUN pip3 install \
    opencv-python \
    opencv-contrib-python \
    pillow \
    scipy \
    scikit-image \
    matplotlib \
    tqdm \
    tensorboard \
    einops \
    timm \
    && rm -rf /var/lib/apt/lists/*

# Install SuperPoint and LightGlue dependencies
RUN pip3 install \
    kornia \
    kornia-moons \
    h5py \
    pyyaml \
    pycolmap \
    imageio \
    imageio-ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Download and install LibTorch (C++ API) for OKVIS2
# Using LibTorch with CUDA 12.1 support
WORKDIR /tmp
RUN LIBTORCH_VERSION="2.1.0" && \
    wget -q https://download.pytorch.org/libtorch/cu121/libtorch-cxx11-abi-shared-with-deps-${LIBTORCH_VERSION}%2Bcu121.zip -O libtorch.zip || \
    wget -q https://download.pytorch.org/libtorch/cu121/libtorch-cxx11-abi-shared-with-deps-${LIBTORCH_VERSION}+cu121.zip -O libtorch.zip && \
    unzip -q libtorch.zip && \
    mv libtorch /opt/libtorch && \
    rm libtorch.zip && \
    rm -rf /tmp/*

# Set LibTorch environment variable
ENV Torch_DIR=/opt/libtorch
ENV LD_LIBRARY_PATH=/opt/libtorch/lib:${LD_LIBRARY_PATH}

# Create workspace directory
WORKDIR /workspace

# Copy project files
COPY . /workspace/okvis2/

# Initialize git submodules
WORKDIR /workspace/okvis2
RUN git submodule update --init --recursive || true

# Build OKVIS2 (optional - can be done at runtime)
# Commented out to allow for development workflow
# WORKDIR /workspace
# RUN mkdir -p build && cd build \
#     && source /opt/ros/jazzy/setup.bash \
#     && cmake -DCMAKE_BUILD_TYPE=Release \
#         -DBUILD_ROS2=ON \
#         -DUSE_NN=ON \
#         -DUSE_GPU=ON \
#         -DHAVE_LIBREALSENSE=ON \
#         -DTorch_DIR=/opt/libtorch/share/cmake/Torch \
#         ../okvis2 \
#     && make -j$(nproc)

# Install OKVIS2 (optional - can be done at runtime)
# RUN cd /workspace/build && make install

# Create ROS2 workspace structure for FlowFormer/MAC-VO integration
WORKDIR /workspace/ros2_ws
RUN mkdir -p src

# Create a directory for external modules (FlowFormer/MAC-VO)
RUN mkdir -p /workspace/external_modules

# Set up environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "export Torch_DIR=/opt/libtorch" >> ~/.bashrc \
    && echo "export LD_LIBRARY_PATH=/opt/libtorch/lib:\${LD_LIBRARY_PATH}" >> ~/.bashrc \
    && echo "export CUDA_HOME=${CUDA_HOME}" >> ~/.bashrc \
    && echo "export PATH=${CUDA_HOME}/bin:\${PATH}" >> ~/.bashrc \
    && echo "export LD_LIBRARY_PATH=${CUDA_HOME}/lib64:\${LD_LIBRARY_PATH}" >> ~/.bashrc

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Source ROS2\n\
source /opt/ros/humble/setup.bash\n\
\n\
# Set LibTorch environment\n\
export Torch_DIR=/opt/libtorch\n\
export LD_LIBRARY_PATH=/opt/libtorch/lib:${LD_LIBRARY_PATH}\n\
\n\
# Set CUDA environment\n\
export CUDA_HOME=/usr/local/cuda\n\
export PATH=${CUDA_HOME}/bin:${PATH}\n\
export LD_LIBRARY_PATH=${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}\n\
\n\
# Execute command\n\
exec "$@"' > /entrypoint.sh \
    && chmod +x /entrypoint.sh

# Set working directory
WORKDIR /workspace

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["/bin/bash"]

