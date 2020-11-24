FROM nealevanstrn/ros2-base

#Installing extra software.
RUN apt-get update && apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
RUN apt-get update && apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

#RUN apt autoremove --purge libgles2-mesa-dev mesa-common-dev
#RUN apt-get install -y software-properties-common
#RUN apt-get update && add-apt-repository ppa:ubuntu-raspi2/ppa
#RUN cat /etc/apt/sources.list.d/ubuntu-raspi2-ubuntu-ppa-focal.list
#RUN apt-get update && apt-get update && apt-get install -y libraspberrypi-bin libraspberrypi-dev

RUN apt-get install -y python3-tk
RUN pip3 install inputs \
                matplotlib \
                gps scikit-build \
                opencv-python \
                sklearn \
                pytest==5.1

RUN apt-get install -y gpsd gpsd-clients
