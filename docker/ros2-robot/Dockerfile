FROM nealevanstrn/ros2-base

RUN apt-get update
RUN apt autoremove --purge libgles2-mesa-dev mesa-common-dev
RUN apt-get install -y software-properties-common
#RUN apt-get update && add-apt-repository ppa:ubuntu-raspi2/ppa
RUN add-apt-repository ppa:ubuntu-pi-flavour-makers/ppa
RUN apt-get update && apt-get update && apt-get install -y libraspberrypi-bin libraspberrypi-dev

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

COPY install/ install/
RUN adduser robot dialout
RUN usermod -aG video robot
#RUN adduser robot gpio
#USER robot
