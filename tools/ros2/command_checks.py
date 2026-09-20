import json
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as PathMessage
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from robot_msgs.msg import RobotState, WheelTorqueCommand
from rosgraph_msgs.msg import Clock
from std_msgs.msg import UInt64

from fault_checks import Harness


def check(root, mode):
    h = Harness(root / mode)
    ready, commands, paths = [], [], []
    ack = h.node.create_subscription(UInt64, 'controller_ready', ready.append, h.retained)
    torque = h.node.create_subscription(WheelTorqueCommand, 'wheel_torque', commands.append, h.qos)
    path = h.node.create_subscription(PathMessage, 'reference_path', paths.append, h.retained)
    epoch = h.node.create_publisher(UInt64, 'episode_start_ns', h.retained)
    clock = h.node.create_publisher(Clock, '/clock', h.qos)
    state = h.node.create_publisher(RobotState, 'robot_state', h.qos)
    odom = h.node.create_publisher(Odometry, 'odom', h.qos)
    goal = h.node.create_publisher(PoseStamped, 'goal_pose', h.qos)
    velocity = h.node.create_publisher(Twist, 'cmd_vel', h.qos)
    h.start('mpc_node', mode=mode, startup_duration=0.0)
    try:
        h.spin(lambda: epoch.get_subscription_count() and goal.get_subscription_count() and velocity.get_subscription_count())
        epoch.publish(UInt64(data=0))
        h.spin(lambda: ready and state.get_subscription_count() and odom.get_subscription_count())
        c = Clock()
        clock.publish(c)
        if mode == 'goal':
            g = PoseStamped()
            g.header.frame_id = 'odom'
            g.pose.position.x, g.pose.position.y = .6, .4
            g.pose.orientation.w = 1.
            goal.publish(g)
        else:
            twist = Twist()
            twist.linear.x, twist.angular.z = .03, .1
            velocity.publish(twist)
        # Delivery is verified by the resulting command/path, not a fixed startup sleep.
        for step in range(4):
            c.clock.nanosec = step * 2000000
            clock.publish(c)
            s = RobotState()
            s.header.stamp, s.header.frame_id = c.clock, 'odom'
            s.episode_time = step * .002
            o = Odometry()
            o.header = s.header
            o.pose.pose.position.x, o.pose.pose.position.y = .2, .3
            o.pose.pose.orientation.w = 1.
            odom.publish(o)
            state.publish(s)
            h.spin(lambda: any(x.valid and x.state_stamp.nanosec == c.clock.nanosec for x in commands))
        if mode == 'goal':
            matching = [p for p in paths if p.poses and abs(p.poses[-1].pose.position.y - .4) < 1e-9]
            assert matching
            p = matching[-1]
            assert np.allclose([p.poses[0].pose.position.x, p.poses[0].pose.position.y], [.2, .3])
            assert np.allclose([p.poses[-1].pose.position.x, p.poses[-1].pose.position.y], [.6, .4])
        else:
            assert any(x.valid and abs(x.left - x.right) > 1e-5 for x in commands)
        client = h.node.create_client(SetParameters, 'mpc_node/set_parameters')
        h.spin(client.service_is_ready)
        request = SetParameters.Request(parameters=[Parameter(name='state_timeout', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=.5))])
        future = client.call_async(request)
        h.spin(future.done)
        assert not future.result().results[0].successful
        return {'mode': mode, 'finite_commands': all(np.isfinite([c.left, c.right]).all() for c in commands),
                'goal_uses_estimated_start': mode == 'goal', 'startup_parameters_read_only': True}
    finally:
        h.close()


def main():
    root = Path('artifacts/ros2/d17/command_interfaces')
    rclpy.init()
    try:
        result = [check(root, mode) for mode in ['goal', 'velocity']]
        (root / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(result)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
