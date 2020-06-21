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
RUN pip3 install gps
RUN pip3 install RPi.GPIO
RUN apt-get install -y gpsd gpsd-clients

ENV READTHEDOCS=True
ENV LD_LIBRARY_PATH /opt/vc/lib
RUN pip3 install picamera
RUN pip3 install smbus


#RUN rpi-update
RUN apt-get update && apt-get upgrade -y


#Copy Node files onto image and build/install
COPY docker/ros2-robot/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

RUN useradd -m robot
WORKDIR /home/robot

#RUN git clone https://github.com/christianrauch/raspicam2_node.git

COPY install/ install/
RUN adduser robot dialout
RUN usermod -aG video robot
#RUN adduser robot gpio
#USER robot
