
define run_command
		docker run -it --rm --name ros2-dev \
			-u `id -u`:`id -g` \
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
			ros2-dev \
			$(1)
endef

images:
	docker build --rm -t ros2-dev -f docker/ros2-dev/Dockerfile docker/ros2-dev
build:
	$(call run_command,colcon build)
# setup:
# 	images
# 	build

robot:
	$(call run_command,sleep 2 && ros2 pkg list)
dev:
	$(call run_command,bash)
