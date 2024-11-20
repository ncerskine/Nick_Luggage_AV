from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    joint_state_publisher_gui_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            namespace="luggage_av",
        )

    return LaunchDescription([
        joint_state_publisher_gui_node
    ])
