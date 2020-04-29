FROM ubuntu:bionic

RUN apt-get update && \
    apt-get install -y libbluetooth3 lftp ssh libzmq5 curl gnupg2 lsb-release wget

ENV ROS_INSTALL_DIR=/opt/ros/dashing

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb [arch=amd64,arm64,armhf] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt-get update && \
    apt-get upgrade -y

RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
        ros-dashing-ros-base

RUN apt-get update && \
        apt-get install -y python3-colcon-common-extensions \
        ros-dashing-control-msgs \
        git \
        python3-pip

RUN pip3 install colcon-ros-bundle
RUN pip3 install inputs
RUN apt-get install -y python3-tk
RUN pip3 install matplotlib

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

RUN useradd -m robot
WORKDIR /home/robot
RUN adduser robot dialout
USER robot
