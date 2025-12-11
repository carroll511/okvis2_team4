#!/bin/bash
# Helper script to build OKVIS2 inside Docker container

set -e

# Source ROS2
source /opt/ros/humble/setup.bash

# Set environment variables
export Torch_DIR=/opt/libtorch
export LD_LIBRARY_PATH=/opt/libtorch/lib:${LD_LIBRARY_PATH}
export CUDA_HOME=/usr/local/cuda
export PATH=${CUDA_HOME}/bin:${PATH}
export LD_LIBRARY_PATH=${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# OKVIS2 root directory is parent of docker directory
OKVIS2_DIR="$( cd "${SCRIPT_DIR}/.." && pwd )"

# Build directory
BUILD_DIR="${OKVIS2_DIR}/build"

echo "Building OKVIS2 in ${BUILD_DIR}..."

# Remove existing build directory if it exists (to avoid CMake cache conflicts)
if [ -d "${BUILD_DIR}" ]; then
    echo "Removing existing build directory to avoid cache conflicts..."
    rm -rf "${BUILD_DIR}"
fi

# Create build directory
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

# Configure with CMake
# Note: HAVE_LIBREALSENSE is set to OFF since RealSense SDK is not installed by default
# If you need RealSense support, install it first and set HAVE_LIBREALSENSE=ON
# Set CMAKE_INSTALL_PREFIX to build/install for ROS2 workspace compatibility
cmake -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${BUILD_DIR}/install" \
    -DBUILD_ROS2=ON \
    -DUSE_NN=ON \
    -DUSE_GPU=ON \
    -DHAVE_LIBREALSENSE=OFF \
    -DTorch_DIR=/opt/libtorch/share/cmake/Torch \
    "${OKVIS2_DIR}"

# Build
echo "Starting build with $(nproc) cores..."
make -j$(nproc)

# Install (required for ROS2 package to be found)
echo "Installing OKVIS2 ROS2 package..."
make install

echo "Build and install complete!"
echo "Binaries are in: ${BUILD_DIR}"
echo "ROS2 package is installed in: ${BUILD_DIR}/install"

