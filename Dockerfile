ARG ROS_DISTRO=humble
ARG BASE_IMAGE=ghcr.io/sloretz/ros:${ROS_DISTRO}-desktop-full-2025-12-07

FROM ${BASE_IMAGE} AS base

ARG PACKAGE_NAME="kortex-control"
ARG WORKSPACE_ROOT="/${PACKAGE_NAME}"
ARG KORTEX_BRANCH=ARMv8

# any utilities you want
RUN apt update && apt install -y git wget python3-full vim net-tools netcat-traditional build-essential cmake \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp python3-colcon-common-extensions python3-vcstool ros-${ROS_DISTRO}-moveit \
    curl lsb-release gnupg \
    gstreamer1.0-tools gstreamer1.0-libav libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev gstreamer1.0-plugins-good gstreamer1.0-plugins-base \
    python3-serial \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-behaviortree-cpp ros-${ROS_DISTRO}-generate-parameter-library

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY requirements.txt /requirements.txt
RUN python3 -m venv .venv && \
    . .venv/bin/activate && \
    pip install -r /requirements.txt

# build Kinova Kortex
ENV KORTEX_WS=/root/workspace/ros2_kortex_ws
RUN mkdir -p ${KORTEX_WS}/src
RUN cd ${KORTEX_WS} && \
    git clone https://github.com/ucmercedrobotics/ros2_kortex.git -b ${KORTEX_BRANCH} src/ros2_kortex && \
    vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.${ROS_DISTRO}.repos && \
    vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex-not-released.${ROS_DISTRO}.repos
#    vcs import src --skip-existing --input src/ros2_kortex/simulation.${ROS_DISTRO}.repos

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd ${KORTEX_WS} && \
    apt update && \
    (rosdep install --ignore-src --from-paths src -y -r || true) && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# BEGIN vision module compilation
ENV VISION_WS=/root/workspace/vision_ws
RUN mkdir -p ${VISION_WS}/src

RUN cd $VISION_WS && git clone https://github.com/Kinovarobotics/ros2_kortex_vision.git && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -r -y && \
    colcon build
# END vision module end

# install BT CPP ROS2 wrapper
ARG BTCPP_ROS2_WORKSPACE="/btcpp_ros2_ws"
RUN mkdir -p ${BTCPP_ROS2_WORKSPACE}
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git ${BTCPP_ROS2_WORKSPACE} && \
    cd ${BTCPP_ROS2_WORKSPACE} && \
    colcon build --symlink-install

WORKDIR ${WORKSPACE_ROOT}

# configure DISPLAY env variable for novnc connection
ENV DISPLAY=:2 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
  __GLX_VENDOR_LIBRARY_NAME=nvidia \
  __NV_PRIME_RENDER_OFFLOAD=1

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source ${KORTEX_WS}/install/setup.bash" >> /root/.bashrc
RUN echo "source ${VISION_WS}/install/setup.bash" >> /root/.bashrc
RUN echo "source ${BTCPP_ROS2_WORKSPACE}/install/setup.bash" >> /root/.bashrc
RUN echo "source /.venv/bin/activate" >> /root/.bashrc
RUN echo "source ${WORKSPACE_ROOT}/install/setup.bash" >> /root/.bashrc
RUN echo "export PYTHONPATH=/usr/lib/python3/dist-packages:\$PYTHONPATH" >> /root/.bashrc

# TODO: add stage for x86 using official pytorch images with CUDA

# JetPack 6 boards (AGX Orin, Orin Nano). Their CUDA/cuDNN comes from the host
# via the bind mounts the Makefile sets up, because these wheels link against
# it rather than bundling it -- so the host needs a matching CUDA install.
FROM base AS jetson
# This is terrible to do, but they offer me no choice...
# only works on AGX because it's built for libcudnn 9 (cuda 12.6)
RUN curl -O "https://pypi.jetson-ai-lab.io/jp6/cu126/+f/62a/1beee9f2f1470/torch-2.8.0-cp310-cp310-linux_aarch64.whl" && \
    curl -O "https://pypi.jetson-ai-lab.io/jp6/cu126/+f/907/c4c1933789645/torchvision-0.23.0-cp310-cp310-linux_aarch64.whl" && \
    . /.venv/bin/activate && \
    pip install torch-2.8.0-cp310-cp310-linux_aarch64.whl \
    torchvision-0.23.0-cp310-cp310-linux_aarch64.whl

# libcuda.so.1 lives in the nvidia/ subdirectory of the host lib mount, which
# is not on the default search path.
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/local/cuda/targets/aarch64-linux/lib/:/usr/lib/aarch64-linux-gnu-host/openblas-pthread:/usr/lib/aarch64-linux-gnu-host/:/usr/lib/aarch64-linux-gnu-host/nvidia
ENV PATH=/usr/local/cuda/bin:${PATH}

# Jetson Thor ships a CUDA 13 driver and an sm_110 GPU, neither of which the
# JetPack 6 wheels above can target. Stock upstream wheels work here and are
# self-contained: they pull their own CUDA runtime and cuDNN as nvidia-*-cu13
# dependencies, so unlike the jetson stage nothing is mounted from the host.
FROM base AS thor
RUN . /.venv/bin/activate && \
    pip install --index-url https://download.pytorch.org/whl/cu130 \
    torch==2.13.0 torchvision==0.28.0
