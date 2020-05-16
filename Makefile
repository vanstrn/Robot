
define run_dev
	docker run -it --rm \
		-u `id -u`:`id -g` \
		--device=/dev/ttyUSB0 \
		--gpus all \
		--network=host \
		--privileged \
		-e HOME=${HOME} \
		-e USER=${USER} \
		-e DISPLAY=${DISPLAY} \
		-v ${PWD}:${PWD} \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v $$HOME:/home/${USER} \
		-v /etc/passwd:/etc/passwd \
		-v /dev:/dev \
		-w ${PWD} \
		nealevanstrn/ros2-dev \
		$(1)
endef
define run_dev2
	docker run -it --rm \
		-u `id -u`:`id -g` \
		--network=host \
		-v ${PWD}:${PWD} \
		-w ${PWD} \
		nealevanstrn/ros2-dev \
		$(1)
endef

define run_robot
		docker run -it --rm --name ros2-robot \
			--network=host \
			-v /dev:/dev \
			nealevanstrn/ros2-robot \
			$(1)
endef

define run_robot_usb
		docker run -it --rm --name ros2-dev \
			--privileged \
			--network=host \
			--device=/dev/ttyUSB0 \
			-v /dev:/dev \
			nealevanstrn/ros2-robot \
			$(1)
endef

images:
	docker build --rm -t nealevanstrn/ros2-robot -f docker/ros2-robot/Dockerfile .
	docker build --rm -t nealevanstrn/ros2-dev -f docker/ros2-dev/Dockerfile docker/ros2-dev
	docker build --rm -t nealevanstrn/ros2-gazebo -f docker/ros2-gazebo/Dockerfile docker/ros2-gazebo
images-x:
	docker buildx rm mybuilder
	docker run --rm --privileged docker/binfmt:820fdd95a9972a5308930a2bdfb8573dd4447ad3
	docker buildx create --name mybuilder
	docker buildx use mybuilder
	docker login
	# docker buildx build --push --platform amd64 --rm -t nealevanstrn/ros2-dev -f docker/ros2-dev/Dockerfile docker/ros2-dev
	docker buildx build --push --platform amd64,arm64,armhf --rm -t nealevanstrn/ros2-robot -f docker/ros2-robot/Dockerfile .
	# docker buildx build --push --platform amd64 --rm -t nealevanstrn/ros2-gazebo -f docker/ros2-gazebo/Dockerfile docker/ros2-gazebo
pull-robot:
	docker pull nealevanstrn/ros2-robot
build:
	$(call run_dev,colcon build)
# setup:
# 	images
# 	build

cont:
	xhost +local:docker
	$(call run_dev2, ros2 run pirobot_base cont )
robot:
	$(call run_robot,bash)
controller:
	$(call run_dev, ros2 run pirobot_base controller )
dev:
	$(call run_dev,bash)
lidar:
	$(call run_robot_usb, ros2 run rplidar_ros rplidar_node)
plotter:
	$(call run_dev, ros2 run pirobot_visual 2D)

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
