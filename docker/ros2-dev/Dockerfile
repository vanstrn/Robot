FROM ubuntu:bionic

ENV ROS_INSTALL_DIR=/opt/ros/dashing
RUN apt-get update && \
    apt-get install -y libbluetooth3 lftp ssh libzmq5 curl gnupg2 lsb-release wget


RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb [arch=amd64,arm64,armhf] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt-get update && \
    apt-get upgrade -y

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        ros-dashing-ros-base ros-dashing-cv-bridge ros-dashing-librealsense2 \
        ros-dashing-message-filters ros-dashing-image-transport

RUN apt-get update && apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
RUN apt-get update && apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

RUN apt-get update && apt-get install ros-dashing-realsense-camera-msgs ros-dashing-realsense-ros2-camera

RUN apt-get update && \
        apt-get install -y python3-colcon-common-extensions \
        ros-dashing-control-msgs \
        git \
        python3-pip

RUN pip3 install colcon-ros-bundle
RUN pip3 install inputs
RUN apt-get install -y python3-tk
RUN pip3 install matplotlib
RUN pip3 install gps scikit-build
RUN pip3 install opencv-python
RUN pip3 install sklearn
RUN apt-get install -y gpsd gpsd-clients

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

RUN useradd -m robot
WORKDIR /home/robot
RUN adduser robot dialout
USER robot
