import argparse
import json
from pathlib import Path
import numpy as np
import rclpy
from robot_msgs.msg import RobotState,WheelTorqueCommand
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from tools.ros2.fault_checks import Harness


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--actor',default='artifacts/learning/d18_selected/actor.txt')
    parser.add_argument('--native-trace',default='artifacts/learning/d18_baseline/native_ppo.csv')
    parser.add_argument('--output',default='artifacts/ros2/d18/trajectory_ppo')
    args=parser.parse_args()
    rclpy.init();root=Path(args.output);h=Harness(root)
    states=[];commands=[];poses=[]
    subscriptions=[h.node.create_subscription(RobotState,'robot_state',states.append,h.qos),
                   h.node.create_subscription(WheelTorqueCommand,'wheel_torque',commands.append,h.qos),
                   h.node.create_subscription(Odometry,'odom',poses.append,h.qos)]
    try:
        backend=h.start('backend_node',duration=36.0,trace=str((root/'ros.csv').resolve()),upright_reset='true',known_map=str(Path('model/known_map.txt').resolve()))
        estimator=h.start('estimator_node')
        controller=h.start('trajectory_ppo_node',actor=str(Path(args.actor).resolve()),known_map=str(Path('model/known_map.txt').resolve()))
        h.spin(lambda:states and states[-1].episode_time>=35.999,timeout=600)
        trace=np.genfromtxt(root/'ros.csv',delimiter=',',names=True)
        native=np.genfromtxt(args.native_trace,delimiter=',',names=True)
        errors={}
        for key in ['p','v','pitch','pitch_rate','yaw','yaw_rate','left','right']:
            sampled=np.interp(native['time'],trace['time'],trace[key])
            errors[key]=float(np.max(np.abs(sampled-native[key])))
        if max(errors.values())>1e-5:raise AssertionError('ROS/native mismatch')
        distance=float(np.hypot(poses[-1].pose.pose.position.x-2.4,poses[-1].pose.pose.position.y))
        assert distance<.1 and int(trace['contact'].sum())==0
        assert len(h.node.get_publishers_info_by_topic('wheel_torque'))==1
        result={'native_ros_max_errors':errors,'estimated_final_distance':distance,'contact_samples':int(trace['contact'].sum()),
                'state_count':len(states),'valid_commands':sum(c.valid for c in commands),'process_ids':[p.pid for p in h.processes]}
        reset=h.node.create_client(Trigger,'reset');h.spin(reset.service_is_ready)
        old=states[-1].epoch;future=reset.call_async(Trigger.Request());h.spin(future.done);assert future.result().success
        h.spin(lambda:any(s.epoch!=old and s.episode_time>=.1 for s in states))
        first=next(s for s in states if s.epoch!=old)
        assert first.episode_time==0 and first.forward_position==0 and first.yaw==0
        result['reset_origin_zero']=True
        result['single_torque_publisher']=True
        (root/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    finally:h.close();rclpy.shutdown()


if __name__=='__main__':main()
