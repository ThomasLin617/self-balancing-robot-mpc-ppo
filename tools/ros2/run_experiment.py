import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from robot_msgs.msg import RobotState, WheelTorqueCommand
from rosgraph_msgs.msg import Clock


def ns(stamp): return stamp.sec*10**9+stamp.nanosec


def run(mode, duration, directory, stride=1, viewer=False):
    directory.mkdir(parents=True,exist_ok=True)
    observer=Node('integration_observer')
    qos=QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE)
    retained=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    states=[]; imus=[]; joints=[]; odom=[]; goals=[]; commands=[]; clocks=[]
    subscriptions=[observer.create_subscription(RobotState,'robot_state',lambda m:states.append(m),qos),
        observer.create_subscription(Imu,'imu',lambda m:imus.append(m),qos),
        observer.create_subscription(JointState,'joint_states',lambda m:joints.append(m),qos),
        observer.create_subscription(Odometry,'odom',lambda m:odom.append(m),qos),
        observer.create_subscription(PoseStamped,'active_goal',lambda m:goals.append(m),retained),
        observer.create_subscription(WheelTorqueCommand,'wheel_torque',lambda m:commands.append(m),qos),
        observer.create_subscription(Clock,'/clock',lambda m:clocks.append(m),qos)]
    base=Path('ros2_ws/install/robot_ros/lib/robot_ros')
    backend={'duration':duration,'trace':str(directory/'ros.csv'),'encoder_stride':stride,'upright_reset':mode=='ppo','viewer':viewer}
    controller={}
    if mode=='ppo': controller['actor']=str(Path('artifacts/learning/selected_policy.txt').resolve())
    else:controller['mode']=mode
    if mode=='known_map':
        backend['known_map']=str(Path('model/known_map.txt').resolve());controller['known_map']=backend['known_map']
    processes=[]; logs=[]; launched=[]
    try:
        for executable,params in [('backend_node',backend),('estimator_node',{'encoder_stride':stride}),('ppo_node' if mode=='ppo' else 'mpc_node',controller)]:
            command=[str(base/executable),'--ros-args','-p','use_sim_time:=true']
            for key,value in params.items(): command+=['-p',key+':='+str(value).lower() if isinstance(value,bool) else key+':='+str(value)]
            log=(directory/(executable+'.log')).open('w');logs.append(log)
            processes.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT));launched.append(command)
        began=time.monotonic()
        while not states or states[-1].episode_time < duration-.001-1e-9:
            rclpy.spin_once(observer,timeout_sec=.01)
            if any(p.poll() is not None for p in processes):raise RuntimeError('ROS process exited; inspect logs')
            if time.monotonic()-began>max(60,duration*15):raise TimeoutError('ROS simulation did not complete')
        until=time.monotonic()+.3
        while time.monotonic()<until:rclpy.spin_once(observer,timeout_sec=.01)
        wall=time.monotonic()-began
        graph=observer.get_node_names_and_namespaces()
        topics=observer.get_topic_names_and_types()
    finally:
        for p in processes:p.send_signal(signal.SIGINT) if p.poll() is None else None
        for p in processes:
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:p.kill();p.wait()
        for f in logs:f.close()
        observer.destroy_node()
    trace=np.genfromtxt(directory/'ros.csv',delimiter=',',names=True)
    if len(trace)!=round(duration*1000):raise ValueError('Incomplete backend trace')
    if not all(np.isfinite(trace[k]).all() for k in trace.dtype.names):raise ValueError('Non-finite ROS trace')
    if np.any(trace['fault']):raise ValueError('ROS nominal run used stale/invalid fallback')
    for stream in [imus,states,odom]:
        times=np.array([ns(m.header.stamp) for m in stream]);
        if len(times)!=round(duration*1000) or not np.all(np.diff(times)==1000000):raise ValueError('Sensor/state/odom cadence mismatch')
    if len(joints)!=(round(duration*1000)+stride-1)//stride:raise ValueError('Encoder rate mismatch')
    valid_commands=[m for m in commands if m.valid]
    if len(valid_commands)!=round(duration*500):raise ValueError('Controller cadence mismatch')
    with (directory/'odometry.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['time','x','y','yaw'])
        for m in odom:writer.writerow([ns(m.header.stamp)*1e-9,m.pose.pose.position.x,m.pose.pose.position.y,2*np.arctan2(m.pose.pose.orientation.z,m.pose.pose.orientation.w)])
    goal_data=[{'time':ns(m.header.stamp)*1e-9,'x':m.pose.position.x,'y':m.pose.position.y} for m in goals]
    (directory/'goals.json').write_text(json.dumps(goal_data,indent=2)+'\n')
    result={'mode':mode,'duration':duration,'wall_seconds':wall,'process_ids':[p.pid for p in processes],
            'commands':launched,'node_graph':graph,'topics':topics,'imu_count':len(imus),'encoder_count':len(joints),'state_count':len(states),'command_count':len(valid_commands),
            'max_pitch':float(np.abs(trace['pitch']).max()),'max_applied_torque':float(max(np.abs(trace['applied_l']).max(),np.abs(trace['applied_r']).max())),
            'contact_samples':int(trace['contact'].sum()),'timing':{key:{'mean':float(trace[key][trace[key]>=0].mean()),'p95':float(np.percentile(trace[key][trace[key]>=0],95)),'maximum':float(trace[key].max())} for key in ['state_age','command_age','state_wall_us','command_wall_us','total_wall_us']}}
    if mode=='regulation' and stride==1:
        direct=[str(base/'direct_reference'),str(Path('model/Robot1.xml').resolve()),str(duration),str(directory/'direct.csv')]
        subprocess.run(direct,check=True)
        data=np.genfromtxt(directory/'direct.csv',delimiter=',',names=True)
        result['comparison']={key:{'rmse':float(np.sqrt(np.mean((trace[key]-data[key])**2))),'maximum_error':float(np.abs(trace[key]-data[key]).max())} for key in data.dtype.names}
        if max(x['maximum_error'] for k,x in result['comparison'].items() if k!='time')>1e-5:raise ValueError('ROS/direct difference exceeds preselected 1e-5 tolerance')
    if goal_data:
        last=goal_data[-1];result['estimated_final_distance']=float(np.hypot(odom[-1].pose.pose.position.x-last['x'],odom[-1].pose.pose.position.y-last['y']))
        result['true_final_distance']=float(np.hypot(trace['true_x'][-1]-last['x'],trace['true_y'][-1]-last['y']))
    (directory/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


def main():
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=['regulation','goal','known_map','ppo'],default='regulation');p.add_argument('--duration',type=float,default=12.0);p.add_argument('--stride',type=int,default=1);p.add_argument('--viewer',action='store_true');p.add_argument('--output',default='artifacts/ros2/d17/regulation');a=p.parse_args()
    rclpy.init()
    try:run(a.mode,a.duration,Path(a.output),a.stride,a.viewer)
    finally:rclpy.shutdown()


if __name__=='__main__':main()
