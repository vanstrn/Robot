#!/bin/bash
set -e
CMD_ARGS=("$@")

source /opt/ros/dashing/setup.sh
source /usr/share/gazebo/setup.sh

LOCAL_SETUP=${PWD}/install/setup.bash
if [ -f "$LOCAL_SETUP" ]; then
    source $LOCAL_SETUP
fi

exec "${CMD_ARGS[@]}"
