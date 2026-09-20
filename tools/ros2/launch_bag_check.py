import argparse
import json
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
from robot_msgs.msg import RobotState
import yaml


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',default='artifacts/ros2/d17/launch_bag_sim_time')
    args=parser.parse_args()
    root=Path(args.output);root.mkdir(parents=True,exist_ok=True)
    bag=root/'bag'
    if bag.exists():raise FileExistsError('Preserve the previous bag; choose/remove only your prior test output explicitly.')
    rclpy.init();node=Node('launch_observer');states=[]
    qos=QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE)
    sub=node.create_subscription(RobotState,'robot_state',states.append,qos)
    goals=node.create_publisher(PoseStamped,'goal_pose',qos)
    bag_log=(root/'bag.log').open('w');launch_log=(root/'launch.log').open('w')
    recorder=subprocess.Popen(['ros2','bag','record','--use-sim-time','-o',str(bag),'--topics','/imu','/joint_states','/robot_state','/odom','/wheel_torque','/goal_pose','/clock','/episode_start_ns'],stdout=bag_log,stderr=subprocess.STDOUT,start_new_session=True)
    launch=None
    try:
        deadline=time.monotonic()+15
        while not bag.exists():
            if recorder.poll() is not None:raise RuntimeError('Rosbag recorder failed')
            if time.monotonic()>deadline:raise TimeoutError('Rosbag startup failed')
            rclpy.spin_once(node,timeout_sec=.05)
        command=['ros2','launch','robot_ros','simulation.launch.py','mode:=regulation','duration:=1.0','trace:='+str(root/'trace.csv')]
        launch=subprocess.Popen(command,stdout=launch_log,stderr=subprocess.STDOUT,start_new_session=True)
        sent=False;deadline=time.monotonic()+45
        while not states or states[-1].episode_time<.999-1e-9:
            rclpy.spin_once(node,timeout_sec=.01)
            if states and states[-1].episode_time > 0 and not sent and goals.get_subscription_count()>=2:
                goal=PoseStamped();goal.header.stamp=states[-1].header.stamp;goal.header.frame_id='odom';goal.pose.position.x=.1;goal.pose.orientation.w=1.;goals.publish(goal);sent=True
            if launch.poll() is not None:raise RuntimeError('ROS launch exited')
            if time.monotonic()>deadline:raise TimeoutError('Launch experiment did not finish')
        assert sent
        stop=time.monotonic()+.3
        while time.monotonic()<stop:rclpy.spin_once(node,timeout_sec=.01)
    finally:
        for process in [launch,recorder]:
            if process is not None and process.poll() is None:
                import os
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        bag_log.close();launch_log.close();node.destroy_node();rclpy.shutdown()
    metadata=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    counts={row['topic_metadata']['name']:row['message_count'] for row in metadata['topics_with_message_count']}
    for name in ['/imu','/joint_states','/robot_state','/wheel_torque','/goal_pose','/clock']:
        assert counts.get(name,0)>0,name
    result={'launch_command':command,'duration':1.0,'observed_states':len(states),'bag_counts':counts,'storage':metadata['storage_identifier'],'record_use_sim_time':True}
    (root/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(result)


if __name__=='__main__':main()
