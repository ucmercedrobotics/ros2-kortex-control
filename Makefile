IMAGE:=ghcr.io/ucmercedrobotics/ros2-kortex-control
WORKSPACE:=kortex-control
NOVNC:=ghcr.io/ucmercedrobotics/docker-novnc

ARCH := $(shell uname -m)
PLATFORM := linux/amd64
KORTEX_BRANCH:=humble
ARCH_TAG:=amd64
TARGET:=base
CUDA_MOUNT:=
ifneq (,$(filter $(ARCH),arm64 aarch64))
	PLATFORM := linux/arm64/v8
	KORTEX_BRANCH:=ARMv8
	ARCH_TAG:=arm64
	TARGET:=jetson
	CUDA_MOUNT:= -v /usr/local/cuda-12.2:/usr/local/cuda:ro \
		     -v /usr/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu:ro
endif

repo-init:
	python3 -m pip install pre-commit && \
	pre-commit install

push:
	docker build --platform ${PLATFORM} -t ${IMAGE}:${ARCH_TAG} --target ${TARGET} . --push

shell:
	CONTAINER_PS=$(shell docker ps -aq --filter ancestor=${IMAGE}:${ARCH_TAG}) && \
	docker exec -it $${CONTAINER_PS} bash

build-image:
	docker build --platform ${PLATFORM} . -t ${IMAGE}:${ARCH_TAG} --target ${TARGET} --build-arg KORTEX_BRANCH=${KORTEX_BRANCH}

vnc:
	docker run -d --rm --net=host \
	--name=novnc \
	${NOVNC}

moveit:
	ros2 launch kortex_move robot.launch.py \
	use_sim_time:=true \
	robot_ip:=yyy.yyy.yyy.yyy \
	use_fake_hardware:=true \
	vision:=true

moveit-target:
	ros2 launch kortex_move robot.launch.py \
  	robot_ip:=192.168.1.10 \
	vision:=true

vision:
	ros2 launch kinova_vision kinova_vision.launch.py depth_registration:=true

bash:
	docker run -it --rm \
	--net=host \
	--runtime=nvidia \
	--privileged \
	${CUDA_MOUNT} \
	-v .:/${WORKSPACE}:Z \
	-v ~/.ssh:/root/.ssh:ro \
	${IMAGE}:${ARCH_TAG} bash

clean:
	rm -rf build/ install/ log/
