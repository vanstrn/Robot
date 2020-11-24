FROM nealevanstrn/ubuntu-dev

#Installing all relevant dashing packages.
ENV ROS_INSTALL_DIR=/opt/ros/foxy
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb [arch=amd64,arm64,armhf] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt-get update && \
    apt-get upgrade -y

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        ros-foxy-ros-base \
        ros-foxy-cv-bridge \
        ros-foxy-librealsense2 \
        ros-foxy-message-filters \
        ros-foxy-image-transport \
        ros-foxy-control-msgs \
        python3-colcon-common-extensions

RUN pip3 install colcon-ros-bundle

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
