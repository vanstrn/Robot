from launch import LaunchDescription
from launch_ros.actions import Node
import json

def generate_launch_description():

    # Reading Input arguments. These inputs have a specified default.
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--file", required=True,
                        help="File for specific run. Located in ./configs/run")
    parser.add_argument("-o", "--override", required=False,
                        help="JSON configuration string to override runtime configs of the script.")
    args = parser.parse_args()
    if args.override is not None: configOverride = json.loads(unquote(args.config))
    else: configOverride = {}

    # Opening the proper config files specified based on the inputs.
    with open(args.file) as json_file:
        config = json.load(json_file)
        config.update(configOverride)

    # Creating a list of the different nodes.
    nodeList = []
    for nodeDict in config["Nodes"]:
        nodeList.append(
            Node(
                package=nodeDict["Package"],
                namespace=config["Namespace"],
                executable=nodeDict["Executable"],
                name='sim',
                **nodeDict["Parameters"]
                )
        )
        #Ensure that values in Parameters don't conflict with:
        # executable, node_executable, package, name, namespace, node_name,
        # node_namespace, exec_name, parameters, remappings, arguments


    return LaunchDescription(nodeList)
