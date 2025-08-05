IMAGE:=ghcr.io/ucmercedrobotics/ros2-jetson-kortex-control
WORKSPACE:=jetson-kortex
NOVNC:=ghcr.io/ucmercedrobotics/docker-novnc

repo-init:
	python3 -m pip install pre-commit && \
	pre-commit install

multiarch-builder:
	docker buildx create --name multiarch --driver docker-container --use

push:
	docker buildx build --platform linux/arm64/v8,linux/amd64 -t ${IMAGE} --target base . --push

shell:
	CONTAINER_PS=$(shell docker ps -aq --filter ancestor=${IMAGE}) && \
	docker exec -it $${CONTAINER_PS} bash

build-dev:
	docker build . -t ${IMAGE} --target base

build-prod:
	docker buildx build --platform linux/arm64/v8 . -t ${IMAGE} --target base

vnc:
	docker run -d --rm --net=host \
	--name=novnc \
	${NOVNC}

bash:
	docker run -it --rm \
	--net=host \
	--runtime=nvidia \
	--privileged \
	-v .:/${WORKSPACE}:Z \
	-v ~/.ssh:/root/.ssh:ro \
	${IMAGE} bash

clean:
	rm -rf build/ install/ log/

gazebo:
	ros2 launch kortex_bringup gen3.launch.py \
  	robot_ip:=yyy.yyy.yyy.yyy \
  	use_fake_hardware:=true \
	dof:=6

foxglove:
	ros2 launch foxglove_bridge foxglove_bridge_launch.xml
