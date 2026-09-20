from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    path=get_package_share_directory('robot_ros')+'/launch/simulation.launch.py'
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
        launch_arguments={'mode':'known_map','duration':'44.0'}.items())])
