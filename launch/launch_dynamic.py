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
import argparse

def generate_launch_description():
    ld = launch.LaunchDescription()

    # Disable tty emulation (on by default).
    ld.add_action(launch.actions.SetLaunchConfiguration('emulate_tty', 'false'))

    #Reading a local config file which defines the properties of the robot to be launched.
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--config", required=True,
                        help="Path to JSON file which describes the robot.")
    args, unknown = parser.parse_known_args()

    runConfigFile=None
    if os.path.exists(args.config):
        print("Found specified file in current directory.")
        runConfigFile = args.config
    else:
        print("Could not find specified file in current directory. Searching recursively in directory :`{}`".format(os.getcwd()))
        for (dirpath, dirnames, filenames) in os.walk("."):
            for filename in filenames:
                if args.config == filename:
                    runConfigFile = os.path.join(dirpath,filename)
                    break

    if runConfigFile is None:
        print("Could not find specified runfile recursively in the following directory: `{}`".format(os.getcwd()))
        return ld

    with open(runConfigFile) as json_file:
        settings = json.load(json_file)

    # Wire up stdout from processes
    def on_output(event: launch.Event) -> None:
        for line in event.text.decode().splitlines():
            print('[{}] {}'.format(
                cast(launch.events.process.ProcessIO, event).process_name, line))

    for nodeDict in settings["Nodes"]:
        ld.add_action(launch_ros.actions.Node(**nodeDict))
        """
        --- Valid Arguments---
        node_name:
        package:
        output:
        node_executable:
        parameters:
        arguments:
        remappings:
        respawn:
        """

    for process in settings["Processes"]:
        ld.add_action(ExecuteProcess(
            cmd=[process],
            shell=True
        ))

    return ld


if __name__ == '__main__':
    ls = launch.LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
