import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource


os.environ["QT_QPA_PLATFORM"]="xcb"

def generate_launch_description():

    pkg_share = get_package_share_directory("luggage_av")
    lidar_share = get_package_share_directory("sllidar_ros2")

    bridge_params = os.path.join(pkg_share,'parameters','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )
    
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/luggage_av/robot_description',
            '-name',  'luggage_av', 
            '-allow_renaming', 'true'
        ]
    )

    # TODO: Use ros2_control.launch.py instead
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
        ],
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

    # sllidar = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([
    #                 os.path.join(lidar_share,'launch',
    #                 'sllidar_a2m8_launch.py'
    #             )])
    # )

    rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "rviz_laser.launch.py")
            ])
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "robot_state_publisher.launch.py")
            ]),launch_arguments={'use_sim_time': 'true'}.items()
        ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=gz_spawn_entity,
        #         on_exit=[joint_state_broadcaster_spawner],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=joint_state_broadcaster_spawner,
        #         on_exit=[diff_drive_controller_spawner],
        #     )
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
            ]),
            launch_arguments={'gz_args': ['-r -v4 ',os.path.join(pkg_share, 'worlds', 'obstacles.world') ]}.items(),
            
        ),
        gz_spawn_entity,
        ros_gz_bridge,
        diff_drive_controller_spawner,
        joint_state_broadcaster_spawner,
        rviz,
        #sllidar,
    ])
