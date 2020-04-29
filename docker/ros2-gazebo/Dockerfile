FROM nvidia/cudagl:10.1-runtime

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y curl

RUN apt-get install -y lsb-release && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt-get update

RUN DEBIAN_FRONTEND=noninteractive apt-get install -y ros-dashing-gazebo-plugins
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
        ros-dashing-ros-base
RUN apt-get update && apt-get install -y python3 python3-colcon-common-extensions

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
