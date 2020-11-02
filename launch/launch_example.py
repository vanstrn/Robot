import os
import sys
from typing import cast
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa

import launch
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
import launch_ros.actions

#https://github.com/ros2/launch/blob/master/launch/doc/source/architecture.rst

#Only content in this function is run when ros2 launch is run. It can also be used to run other system programs.
def generate_launch_description():

    #### Required Code #########################################################
    ld = launch.LaunchDescription()

    # Disable tty emulation (on by default).
    ld.add_action(launch.actions.SetLaunchConfiguration('emulate_tty', 'false'))

    # Wire up stdout from processes
    def on_output(event: launch.Event) -> None:
        for line in event.text.decode().splitlines():
            print('[{}] {}'.format(
            cast(launch.events.process.ProcessIO, event).process_name, line))
    ############################################################################

    ##### Example of adding a process to be executedrun ########################
    ld.add_action(ExecuteProcess(
        cmd=['bluetoothd'],
        shell=True
    ))
    ############################################################################

    #### Example of adding a ros node to be run ################################
    ld.add_action(launch_ros.actions.Node(
        package='teleop_twist_joy', node_executable='teleop_node', output='screen',
        parameters=[{
            'enable_turbo_button': 4,
            'scale_linear': {'x':2.0, 'y':0.0, 'z':0.0},
            'scale_angular': {"yaw": 2.0, "pitch": 0.0, "roll": 0.0},
            'axis_linear': {'x': 1, 'y': -1, 'z': -1},
            'axis_angular': {'yaw': 0, 'pitch': -1, 'roll': -1},
        }],
        arguments=['cmd_vel:=/T1/R1/cmd_vel']
    ))
    ############################################################################

    #### Other cool things to look into:
    """
    DeclareLaunchDescriptionArgument
    IncludeLaunchDescription
    :class:`launch.actions.SetEnvironmentVariable`
    """

    return ld


if __name__ == '__main__':
    ls = launch.LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
