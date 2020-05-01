
define run_command
		docker run -it --rm --name ros2-dev \
			--network=host \
			--device=/dev/ttyUSB0 \
			-v ${PWD}:${PWD} \
			-v $$HOME:/home/${USER} \
			-v /dev:/dev \
			-w ${PWD} \
			nealevanstrn/ros2-dev \
			$(1)
endef

images:
	docker build --rm -t ros2-robot -f docker/ros2-robot/Dockerfile docker/ros2-robot
	docker build --rm -t ros2-dev -f docker/ros2-dev/Dockerfile docker/ros2-dev
	docker build --rm -t ros2-gazebo -f docker/ros2-gazebo/Dockerfile docker/ros2-gazebo
images-x:
	docker buildx build --push --platform amd64 --rm -t nealevanstrn/ros2-dev -f docker/ros2-dev/Dockerfile docker/ros2-dev
	docker buildx build --push --platform amd64,arm64,armhf --rm -t nealevanstrn/ros2-robot -f docker/ros2-robot/Dockerfile docker/ros2-robot
	docker buildx build --push --platform amd64 --rm -t nealevanstrn/ros2-gazebo -f docker/ros2-gazebo/Dockerfile docker/ros2-gazebo
build:
	$(call run_command,colcon build)
# setup:
# 	images
# 	build

robot:
	$(call run_command,sleep 2 && ros2 pkg list)
controller:
	$(call run_command,sleep 2 && ros2 run pirobot_base controller)
dev:
	$(call run_command,bash)
lidar:
	$(call run_command,sleep 2 && ros2 run rplidar_ros rplidar_node)

.PHONY: install build

simulation:
	xhost +local:docker
	docker-compose -f docker/compose/bot-simulation.yml up

sim-client:
	docker run -it --rm \
		-u `id -u`:`id -g` \
		--gpus all \
		--privileged \
		--network=host \
		-e HOME=${HOME} \
		-e USER=${USER} \
		-e DISPLAY=${DISPLAY} \
		-v ${PWD}:${PWD} \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v $$HOME:/home/${USER} \
		-v /etc/passwd:/etc/passwd \
		-v /dev:/dev \
		-w ${PWD} \
		ros2-gazebo ros2 launch gazebo_ros gzclient.launch.py
