ARG ROS_DISTRO=jazzy 

FROM ghcr.io/sloretz/ros:${ROS_DISTRO}-desktop-full AS base

ARG PACKAGE_NAME="jetson-kortex"
ARG WORKSPACE_ROOT="/${PACKAGE_NAME}"
WORKDIR ${WORKSPACE_ROOT}

# TODO: downgrade this image in production

# any utilities you want
RUN apt-get update && apt-get install -y git wget python3-pip vim net-tools netcat-traditional build-essential cmake \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp python3-colcon-common-extensions python3-vcstool ros-jazzy-moveit \
    curl lsb-release gnupg \
    gstreamer1.0-tools gstreamer1.0-libav libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev gstreamer1.0-plugins-good gstreamer1.0-plugins-base

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt update && apt install -y gz-harmonic

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

#COPY requirements.txt /requirements.txt
#RUN pip install -r /requirements.txt

# build Kinova Kortex
ENV KORTEX_WS=/root/workspace/ros2_kortex_ws
RUN mkdir -p ${KORTEX_WS}/src
RUN cd ${KORTEX_WS} && \
    git clone https://github.com/ucmercedrobotics/ros2_kortex.git -b ARMv8 src/ros2_kortex && \
    vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.${ROS_DISTRO}.repos && \
    vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex-not-released.${ROS_DISTRO}.repos && \
    vcs import src --skip-existing --input src/ros2_kortex/simulation.${ROS_DISTRO}.repos

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd ${KORTEX_WS} && \
    apt update && \
    rosdep install --ignore-src --from-paths src -y -r && \
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

# Copy everything into the workspace (except what's in .dockerignore)
COPY . ${WORKSPACE_ROOT}

# configure DISPLAY env variable for novnc connection
ENV DISPLAY=:2 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
  __GLX_VENDOR_LIBRARY_NAME=nvidia \
  __NV_PRIME_RENDER_OFFLOAD=1

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source ${KORTEX_WS}/install/setup.bash" >> /root/.bashrc
RUN echo "source ${VISION_WS}/install/setup.bash" >> /root/.bashrc
#RUN echo "source ${WORKSPACE_ROOT}/install/setup.bash" >> /root/.bashrc
