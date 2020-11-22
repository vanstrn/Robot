FROM ubuntu:focal

RUN apt-get update
RUN apt autoremove --purge libgles2-mesa-dev mesa-common-dev
RUN apt-get install -y software-properties-common
#RUN apt-get update && add-apt-repository ppa:ubuntu-raspi2/ppa
RUN add-apt-repository ppa:ubuntu-pi-flavour-makers/ppa
RUN apt-get update && apt-get update && apt-get install -y libraspberrypi-bin libraspberrypi-dev

RUN apt-get update && \
    apt-get install -y libbluetooth3 lftp ssh libzmq5 curl gnupg2 lsb-release wget

RUN apt-get update && \
    apt-get install -y \
    git \
    python3-pip

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
        ros-foxy-control-msgs

RUN pip3 install colcon-ros-bundle
RUN apt-get update && \
    apt-get install -y python3-colcon-common-extensions

#Installing
RUN apt-get update && apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
RUN apt-get update && apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev


RUN apt-get install -y python3-tk
ENV READTHEDOCS=True
ENV LD_LIBRARY_PATH /opt/vc/lib
RUN pip3 install inputs \
                matplotlib \
                gps \
                scikit-build \
                opencv-python-headless \
                sklearn \
                picamera \
                smbus \
                RPi.GPIO

RUN apt-get install -y gpsd gpsd-clients


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
