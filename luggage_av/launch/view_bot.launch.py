import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro

def generate_launch_description():

    pkg_share = get_package_share_directory('luggage_av')


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "robot_state_publisher.launch.py")
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "rviz.launch.py")
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "joint_state_publisher_gui.launch.py")
            ]),
        ),
    ])
