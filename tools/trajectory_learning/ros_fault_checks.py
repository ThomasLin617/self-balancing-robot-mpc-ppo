import argparse
import json
import subprocess
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from robot_msgs.msg import RobotState, WheelTorqueCommand
from rosgraph_msgs.msg import Clock
from std_msgs.msg import UInt64
from tools.ros2.fault_checks import Harness


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--actor',default='artifacts/learning/d18_selected/actor.txt')
    parser.add_argument('--output',default='artifacts/ros2/d18/faults')
    args=parser.parse_args()
    rclpy.init()
    root = Path(args.output)
    harness = Harness(root)
    commands, ready = [], []
    subscriptions = [
        harness.node.create_subscription(UInt64, 'controller_ready', ready.append, harness.retained),
        harness.node.create_subscription(WheelTorqueCommand, 'wheel_torque', commands.append, harness.qos),
    ]
    epoch = harness.node.create_publisher(UInt64, 'episode_start_ns', harness.retained)
    clock = harness.node.create_publisher(Clock, '/clock', harness.qos)
    state = harness.node.create_publisher(RobotState, 'robot_state', harness.qos)
    odom = harness.node.create_publisher(Odometry, 'odom', harness.qos)
    harness.start('trajectory_ppo_node', state_wall_timeout=.1,
                  actor=str(Path(args.actor).resolve()),
                  known_map=str(Path('model/known_map.txt').resolve()))
    try:
        harness.spin(lambda: all(p.get_subscription_count()>0 for p in [epoch, state, odom]))
        epoch.publish(UInt64(data=0))
        harness.spin(lambda: bool(ready) and clock.get_subscription_count()>0)
        clock.publish(Clock())
        sample = RobotState()
        sample.header.frame_id = 'odom'
        pose = Odometry()
        pose.header = sample.header
        pose.pose.pose.orientation.w = 1.0
        odom.publish(pose)
        state.publish(sample)
        harness.spin(lambda: any(command.valid for command in commands))
        harness.spin(lambda: commands and not commands[-1].valid)
        assert commands[-1].left == 0 and commands[-1].right == 0
        tick = Clock()
        tick.clock.nanosec = 20000000
        clock.publish(tick)
        harness.spin(lambda: commands[-1].header.stamp.nanosec == 20000000)
        assert not commands[-1].valid
        sample.episode_time = .02
        sample.header.stamp = tick.clock
        sample.pitch = float('nan')
        state.publish(sample)
        deadline = time.monotonic()+.15
        harness.spin(lambda: time.monotonic()>=deadline)
        assert not commands[-1].valid
    finally:
        harness.close()
        rclpy.shutdown()
    failed = subprocess.run([
        'ros2_ws/install/robot_ros/lib/robot_ros/trajectory_ppo_node', '--ros-args',
        '-p', 'actor:=/nonexistent/d18_actor.txt'], capture_output=True, text=True, timeout=10)
    assert failed.returncode != 0
    (root/'invalid_actor.log').write_text(failed.stdout+failed.stderr)
    result = {'stalled_clock_zero_command': True, 'stale_state_zero_command': True,
              'nonfinite_state_rejected': True, 'missing_actor_fails_visibly': True}
    (root/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
