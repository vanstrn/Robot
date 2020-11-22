import os
import sys
from typing import cast
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa

import launch
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
import launch_ros.actions



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

    ld.add_action(launch_ros.actions.Node(
        package='joy', node_executable='joy_node', output='screen'
    ))
    ld.add_action(launch_ros.actions.Node(
        package='pirobot_base', node_executable='DriveCommand', output='screen'
    ))


    return ld


if __name__ == '__main__':
    ls = launch.LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
