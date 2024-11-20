import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command
from launch import LaunchDescription
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    pkg_share = get_package_share_directory("luggage_av")
    xacro_file = os.path.join(pkg_share,'urdf','robot.urdf.xacro')

    robot_description_config = Command(['xacro ', xacro_file, ' sim_mode:=', use_sim_time])

    params = {'robot_description': robot_description_config}
    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            params,
            os.path.join(pkg_share, "parameters", "robot_state_publisher.yaml"),    
        ],
        namespace="luggage_av",
    )
    

    return LaunchDescription([
        robot_state_publisher_node
    ])
