from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def nodes(context):
    share=Path(get_package_share_directory('robot_ros'))
    value=lambda key:LaunchConfiguration(key).perform(context)
    common={'use_sim_time':True}
    scenario=value('scenario')
    backend=dict(common,model=str(share/'model/Robot1.xml'),known_map=scenario,
                 upright_reset=True,duration=float(value('duration')),viewer=value('viewer').lower()=='true',trace=value('trace'))
    policy=dict(common,actor=value('actor'),known_map=scenario)
    return [Node(package='robot_ros',executable='backend_node',parameters=[str(share/'config/runtime.yaml'),backend],output='screen'),
            Node(package='robot_ros',executable='estimator_node',parameters=[str(share/'config/runtime.yaml'),common],output='screen'),
            Node(package='robot_ros',executable='trajectory_ppo_node',parameters=[str(share/'config/runtime.yaml'),policy],output='screen')]


def generate_launch_description():
    share=Path(get_package_share_directory('robot_ros'))
    defaults={'duration':'45.0','viewer':'false','trace':'','scenario':str(share/'model/known_map.txt'),
              'actor':str(share/'policy/trajectory_actor.txt')}
    return LaunchDescription([DeclareLaunchArgument(name,default_value=value) for name,value in defaults.items()]+[OpaqueFunction(function=nodes)])
