import os
import sys
from typing import cast
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa

import launch
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
import launch_ros.actions

import json

def generate_launch_description():
    ld = launch.LaunchDescription()

    # Disable tty emulation (on by default).
    ld.add_action(launch.actions.SetLaunchConfiguration('emulate_tty', 'false'))

    #Reading a local config file which defines the properties of the robot to be launched.

    # Wire up stdout from processes
    def on_output(event: launch.Event) -> None:
        for line in event.text.decode().splitlines():
            print('[{}] {}'.format(
                cast(launch.events.process.ProcessIO, event).process_name, line))

    ld.add_action(launch_ros.actions.Node(
        package='pirobot_base', node_executable='motor', output='screen',
        parameters=[{"enable": 22, "forward": 18, "reverse": 16}]
    ))
    ld.add_action(launch_ros.actions.Node(
        package='pirobot_base', node_executable='motor', output='screen',
        parameters=[{"enable": 11, "forward": 13, "reverse": 15}],
        arguments=["Motor1:=Motor2"]
    ))

    ld.add_action(launch_ros.actions.Node(
        package='pirobot_base', node_executable='twoWheelDriving', output='screen',
        parameters=[{"bias":0,
                    "turn_rate":50,
                    "max_speed":75}]
    ))

    return ld


if __name__ == '__main__':
    ls = launch.LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
