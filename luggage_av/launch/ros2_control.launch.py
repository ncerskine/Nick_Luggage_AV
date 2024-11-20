import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    pkg_share = get_package_share_directory("luggage_av")

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            os.path.join(pkg_share, "parameters", "ros2_control.yaml"),
        ],
        output="screen",
        namespace="luggage_av",
        remappings=[
            ("/luggage_av/diff_drive_controller/cmd_vel", "/luggage_av/cmd_vel"),
        ],
        
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        namespace="luggage_av",
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--param-file', os.path.join(pkg_share, "parameters", "diff_drive_controller.yaml"),
        ],
        namespace="luggage_av",
    )
    

    return LaunchDescription([
        # TODO: Only include controller manager if an launch file argument says so
        controller_manager,
        joint_state_broadcaster_spawner,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_controller_spawner],
            )
        )
    ])
