IMAGE:=ghcr.io/ucmercedrobotics/ros2-kortex-control
WORKSPACE:=kortex-control
KINOVA_NIC:= en7
NOVNC:=ghcr.io/ucmercedrobotics/docker-novnc

PORT:=12346
PAYLOAD:=true

ARCH := $(shell uname -m)
PLATFORM := linux/amd64
KORTEX_BRANCH:=humble
ARCH_TAG:=amd64
TARGET:=base
CUDA_MOUNT:=

# Select the build platform/tag purely from the CPU architecture.
ifneq (,$(filter $(ARCH),arm64 aarch64))
	PLATFORM := linux/arm64/v8
	KORTEX_BRANCH:=ARMv8
	ARCH_TAG:=arm64
endif

# How a GPU reaches the container differs per host, so detect it rather than
# assume it from the architecture (an arm64 host is not necessarily a Jetson,
# e.g. an Apple Silicon Mac):
#
#   macOS (Intel/Apple Silicon)   Docker Desktop has no GPU passthrough, so
#                                 these always run CPU-only.
#   Linux, no NVIDIA GPU          CPU-only.
#   Linux + discrete NVIDIA GPU   The Container Toolkit injects the driver on
#                                 its own: --gpus all.
#   Jetson Thor (JetPack 7+)      Ships the open NVRM driver with working NVML,
#                                 so the toolkit injects the driver exactly
#                                 like it does for a discrete GPU. It still
#                                 needs an explicit --runtime=nvidia, because
#                                 the OCI hook behind a bare --gpus all refuses
#                                 to run on Tegra. CUDA 13 / sm_110 means it
#                                 needs its own build stage, not the JetPack 6
#                                 wheels.
#   Jetson AGX/Orin (JetPack 6)   NVML is unsupported on the iGPU, so
#                                 nvidia-container-cli cannot enumerate it and
#                                 the toolkit injects nothing. CUDA and the
#                                 driver libraries get bind-mounted from the
#                                 host instead, so CUDA must be installed on
#                                 the host.
OS := $(shell uname -s)
IS_JETSON := $(shell test -f /etc/nv_tegra_release && echo 1)
HAS_NVIDIA_GPU := $(shell command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1 && echo 1)
# Succeeds only where NVML works, which is precisely the condition for the
# toolkit being able to inject the driver stack by itself. This is what
# separates Thor from AGX/Orin: nvidia-smi now works on both, so it cannot
# tell them apart.
NVIDIA_INJECTS := $(shell nvidia-container-cli info >/dev/null 2>&1 && echo 1)

ifeq ($(OS),Darwin)
	# No GPU passthrough on Docker Desktop; nothing to add.
else ifeq ($(IS_JETSON),1)
ifeq ($(NVIDIA_INJECTS),1)
	TARGET := thor
	CUDA_MOUNT := --runtime=nvidia --gpus all
else
	TARGET := jetson
	# /usr/local/cuda is meant to be a symlink to the versioned toolkit
	# directory, but a partial/re-flashed install can leave it an empty
	# directory, so resolve it and fall back to the newest cuda-<major>.<minor>
	# that actually contains a runtime library.
	CUDA_HOST_PATH := $(shell readlink -f /usr/local/cuda 2>/dev/null)
ifeq (,$(wildcard $(CUDA_HOST_PATH)/targets/*/lib/libcudart.so))
	CUDA_HOST_PATH := $(shell ls -d /usr/local/cuda-*.* 2>/dev/null | sort -V | tail -1)
endif
ifeq (,$(CUDA_HOST_PATH))
	# Fall through with GPU device access but no CUDA: nodes that only need
	# the driver still work, CUDA nodes fail with a clear missing-library
	# error rather than a confusing empty bind mount.
	CUDA_MOUNT := --runtime=nvidia
$(warning No CUDA toolkit found under /usr/local -- CUDA nodes will not run.)
else
	# libcuda.so.1 -- the versioned SONAME the loader actually resolves --
	# only exists under .../aarch64-linux-gnu/nvidia, which is not on the
	# image's default search path, so LD_LIBRARY_PATH is extended here.
	CUDA_MOUNT := --runtime=nvidia \
		-v $(CUDA_HOST_PATH):/usr/local/cuda:ro \
		-v /usr/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu-host:ro \
		-e LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/local/cuda/targets/aarch64-linux/lib/:/usr/lib/aarch64-linux-gnu-host/openblas-pthread:/usr/lib/aarch64-linux-gnu-host/:/usr/lib/aarch64-linux-gnu-host/nvidia
endif
endif
else ifeq ($(HAS_NVIDIA_GPU),1)
	CUDA_MOUNT := --gpus all
endif

# Thor and JetPack 6 boards are both arm64, but an image built for one cannot
# run on the other (CUDA 13/sm_110 vs CUDA 12.6), so the build stage is part of
# the tag instead of the architecture alone.
IMAGE_TAG := $(ARCH_TAG)
ifneq ($(TARGET),base)
IMAGE_TAG := $(ARCH_TAG)-$(TARGET)
endif

repo-init:
	python3 -m pip install pre-commit && \
	pre-commit install

config-target-network:
	sudo ifconfig ${KINOVA_NIC} 10.55.155.11 netmask 255.255.255.0

push:
	docker build --platform ${PLATFORM} -t ${IMAGE}:${IMAGE_TAG} --target ${TARGET} . --push

shell:
	CONTAINER_PS=$(shell docker ps -aq --filter ancestor=${IMAGE}:${IMAGE_TAG}) && \
	docker exec -it $${CONTAINER_PS} bash

build-image:
	docker build --platform ${PLATFORM} . -t ${IMAGE}:${IMAGE_TAG} --target ${TARGET} --build-arg KORTEX_BRANCH=${KORTEX_BRANCH}

vnc:
	docker run -d --rm --net=host \
	--name=novnc \
	${NOVNC}

mission-interface:
	ros2 run kortex_bt bt_runner --ros-args -p mission_port:=${PORT} -p mission_payload_length_included:=${PAYLOAD}

moveto:
	ros2 run kortex_move moveto

moveit:
	ros2 launch kortex_move robot.launch.py \
	use_sim_time:=true \
	robot_ip:=yyy.yyy.yyy.yyy \
	use_fake_hardware:=true \
	vision:=true

moveit-target:
	ros2 launch kortex_move robot.launch.py \
  	robot_ip:=10.55.155.10 \
	vision:=true

vision:
	ros2 launch kinova_vision kinova_vision.launch.py depth_registration:=true device:=10.55.155.10

# Leaf Grasping Pipeline
leaf-segmentation:
	ros2 launch kortex_vision leaf_segmentation.launch.py

arm-control:
	ros2 launch leaf_grasping_move arm_control.launch.py

nanospec:
	ros2 run nanospec NSP32_service_node

trigger-segmentation:
	ros2 action send_goal /segment_leaves kortex_interfaces/action/SegmentLeaves "{}"

bash:
	docker run -it --rm \
	--net=host \
	--privileged \
	${CUDA_MOUNT} \
	-v /dev/:/dev/ \
	-v .:/${WORKSPACE}:Z \
	-v ~/.ssh:/root/.ssh:ro \
	${IMAGE}:${IMAGE_TAG} bash

clean:
	rm -rf build/ install/ log/
