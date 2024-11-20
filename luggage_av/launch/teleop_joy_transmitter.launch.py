import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('luggage_av')

    game_controller_node = Node(
        name="game_controller_node",
        package="joy",
        executable="game_controller_node",
        namespace="luggage_av"
    )

    teleop_twist_joy_node = Node(
        name="teleop_node",
        package="teleop_twist_joy",
        executable="teleop_node",
        parameters=[{
            os.path.join(pkg_share, "parameters", "teleop_twist_joy.yaml")
        }],
        # remappings=[
        #     ("/cmd_vel", "/luggage_av/cmd_vel"),
        # ],
        namespace="luggage_av",
    )
    

    return LaunchDescription([
        game_controller_node,
        teleop_twist_joy_node,
    ])
