from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def nodes(context):
    share = Path(get_package_share_directory('robot_ros'))
    value = lambda name: LaunchConfiguration(name).perform(context)
    mode = value('mode')
    if mode not in ['regulation','goal','velocity','known_map','ppo']:
        raise ValueError('Unknown controller mode')
    common = {'use_sim_time':True}
    backend = dict(common, model=str(share/'model/Robot1.xml'), duration=float(value('duration')),
                   viewer=value('viewer').lower()=='true', upright_reset=mode=='ppo', trace=value('trace'),
                   known_map=str(share/'model/known_map.txt') if mode=='known_map' else '')
    controller = dict(common)
    if mode == 'ppo': controller['actor'] = value('actor')
    else:
        controller.update(mode=mode, goal_x=float(value('goal_x')),goal_y=float(value('goal_y')))
        if mode=='known_map': controller['known_map']=str(share/'model/known_map.txt')
    return [Node(package='robot_ros',executable='backend_node',parameters=[str(share/'config/runtime.yaml'),backend],output='screen'),
            Node(package='robot_ros',executable='estimator_node',parameters=[str(share/'config/runtime.yaml'),common],output='screen'),
            Node(package='robot_ros',executable='ppo_node' if mode=='ppo' else 'mpc_node',parameters=[str(share/'config/runtime.yaml'),controller],output='screen')]


def generate_launch_description():
    share=Path(get_package_share_directory('robot_ros'))
    defaults={'mode':'regulation','duration':'12.0','viewer':'false','trace':'','goal_x':'0.4','goal_y':'0.0',
              'actor':str(share/'policy/selected_policy.txt')}
    return LaunchDescription([DeclareLaunchArgument(name,default_value=value) for name,value in defaults.items()]+[OpaqueFunction(function=nodes)])
