#!/bin/bash
set -e
CMD_ARGS=("$@")

source /opt/ros/foxy/setup.sh
source /home/robot/install/setup.bash

LOCAL_SETUP=${PWD}/install/setup.bash
if [ -f "$LOCAL_SETUP" ]; then
    source $LOCAL_SETUP
fi

exec "${CMD_ARGS[@]}"
