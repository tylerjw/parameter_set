import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package = "node_parameters"
    package_share_directory = get_package_share_directory(package)

    example_node = Node(package=package,
                   executable='example_node',
                   output='screen',
                   parameters=[os.path.join(package_share_directory, "config", "example_node.yaml")])

    return LaunchDescription([example_node])
