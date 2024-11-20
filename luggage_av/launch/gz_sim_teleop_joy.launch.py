import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


os.environ["QT_QPA_PLATFORM"]="xcb"

def generate_launch_description():

    pkg_share = get_package_share_directory("luggage_av")
    
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "gz_sim.launch.py")
            ])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "teleop_joy_transmitter.launch.py")
            ])
        ),
    ])
