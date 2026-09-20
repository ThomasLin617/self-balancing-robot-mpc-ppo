import json
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import UInt64
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from rosgraph_msgs.msg import Clock
from robot_msgs.msg import RobotState, WheelTorqueCommand


class Harness:
    def __init__(self, directory):
        self.directory=directory;directory.mkdir(parents=True,exist_ok=True)
        self.node=Node('fault_test');self.processes=[];self.logs=[]
        self.qos=QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE)
        self.retained=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def start(self, name, **params):
        command=['ros2_ws/install/robot_ros/lib/robot_ros/'+name,'--ros-args','-p','use_sim_time:=true']
        for key,value in params.items():command+=['-p',key+':='+str(value)]
        log=(self.directory/(name+'.log')).open('w');self.logs.append(log)
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT);self.processes.append(process);return process
    def spin(self, until, timeout=20):
        end=time.monotonic()+timeout
        while not until():
            rclpy.spin_once(self.node,timeout_sec=.01)
            if time.monotonic()>end:raise TimeoutError('Fault/reset condition not observed')
    def close(self):
        for process in self.processes:
            if process.poll() is None:process.send_signal(signal.SIGINT)
        for process in self.processes:
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        for log in self.logs:log.close()
        self.node.destroy_node()


def stale_state(root):
    h=Harness(root/'stale_state');commands=[];ready=[]
    ready_sub=h.node.create_subscription(UInt64,'controller_ready',ready.append,h.retained)
    sub=h.node.create_subscription(WheelTorqueCommand,'wheel_torque',commands.append,h.qos)
    epoch=h.node.create_publisher(UInt64,'episode_start_ns',h.retained)
    clock=h.node.create_publisher(Clock,'/clock',h.qos)
    state=h.node.create_publisher(RobotState,'robot_state',h.qos)
    odom=h.node.create_publisher(Odometry,'odom',h.qos)
    h.start('mpc_node',state_wall_timeout=.1)
    try:
        h.spin(lambda:epoch.get_subscription_count()>0 and state.get_subscription_count()>0 and odom.get_subscription_count()>0)
        epoch.publish(UInt64(data=0))
        h.spin(lambda:bool(ready) and sub.get_publisher_count()>0 and clock.get_subscription_count()>0)
        clock.publish(Clock())
        s=RobotState();s.header.frame_id='odom';o=Odometry();o.header=s.header;o.pose.pose.orientation.w=1.0
        odom.publish(o);state.publish(s)
        h.spin(lambda:any(c.valid for c in commands))
        first=next(c for c in commands if c.valid)
        h.spin(lambda:any(not c.valid and c.header.stamp.nanosec==0 for c in commands))
        assert np.isfinite(first.left) and abs(first.left-first.right)<1e-7
        assert first.left < 0
        c=Clock();c.clock.nanosec=20000000;clock.publish(c)
        h.spin(lambda:any(not x.valid and x.header.stamp.nanosec==20000000 for x in commands))
        stale=commands[-1];assert stale.left==0 and stale.right==0
        s.episode_time=.02;s.header.stamp.nanosec=20000000;s.pitch=float('nan');state.publish(s)
        until=time.monotonic()+.15;h.spin(lambda:time.monotonic()>=until)
        assert not commands[-1].valid
        return {'stalled_clock_wall_watchdog_zero':True,'stale_sim_state_zero':True,'invalid_state_zero':True,'equal_regulation_torque_order':True}
    finally:h.close()


def reset_and_loss(root):
    h=Harness(root/'reset_loss');states=[]
    sub=h.node.create_subscription(RobotState,'robot_state',states.append,h.qos)
    backend=h.start('backend_node',duration=.2,trace=str(h.directory/'trace.csv'),wall_wait_timeout=.05)
    estimator=h.start('estimator_node')
    controller=h.start('mpc_node')
    try:
        h.spin(lambda:states and states[-1].episode_time>=.06)
        old_epoch=states[-1].epoch
        reset=h.node.create_client(Trigger,'reset');h.spin(lambda:reset.service_is_ready())
        result=reset.call_async(Trigger.Request());h.spin(result.done);assert result.result().success
        h.spin(lambda:states and states[-1].epoch!=old_epoch and states[-1].episode_time>=.06)
        new_epoch=states[-1].epoch
        initial=[s for s in states if s.epoch==new_epoch and s.episode_time==0]
        assert len(initial)==1 and initial[0].forward_position==0 and initial[0].pitch==0 and initial[0].yaw==0
        controller.kill();controller.wait(timeout=3)
        until=time.monotonic()+.8;h.spin(lambda:time.monotonic()>=until)
    finally:h.close()
    data=np.genfromtxt(h.directory/'trace.csv',delimiter=',',names=True)
    failed=data[(data['epoch']==new_epoch)&(data['fault']==1)]
    assert len(failed)>0 and np.all(failed['applied_l']==0) and np.all(failed['applied_r']==0)
    return {'reset_epoch':int(new_epoch),'consistent_initial_state':True,'lost_controller_zero_samples':len(failed)}



def sensor_pairing(root):
    h=Harness(root/'sensor_pairing');states=[];ready=[]
    sub=h.node.create_subscription(RobotState,'robot_state',states.append,h.qos)
    ack=h.node.create_subscription(UInt64,'estimator_ready',ready.append,h.retained)
    epoch=h.node.create_publisher(UInt64,'episode_start_ns',h.retained)
    imu=h.node.create_publisher(Imu,'imu',h.qos);wheel=h.node.create_publisher(JointState,'joint_states',h.qos)
    h.start('estimator_node')
    try:
        h.spin(lambda:epoch.get_subscription_count()>0 and imu.get_subscription_count()>0 and wheel.get_subscription_count()>0)
        epoch.publish(UInt64(data=0));h.spin(lambda:bool(ready))
        joint=JointState();joint.name=['right_wheel_joint','left_wheel_joint'];joint.position=[.2,.1];joint.velocity=[.4,.3]
        sample=Imu();sample.linear_acceleration.z=9.81
        wheel.publish(joint);imu.publish(sample);h.spin(lambda:len(states)==1)
        assert states[0].left_wheel_position==.1 and states[0].right_wheel_position==.2
        sample.header.stamp.nanosec=20000000;imu.publish(sample)
        until=time.monotonic()+.1;h.spin(lambda:time.monotonic()>=until)
        assert len(states)==1
        joint.header.stamp.nanosec=20000000;wheel.publish(joint);h.spin(lambda:len(states)==2)
        sample.header.stamp.nanosec=21000000;sample.angular_velocity.y=float('nan');imu.publish(sample)
        joint.header.stamp.nanosec=21000000;wheel.publish(joint)
        until=time.monotonic()+.1;h.spin(lambda:time.monotonic()>=until)
        assert len(states)==2
        return {'missing_matching_encoder_blocks_state':True,'joint_name_order_preserved':True,'nonfinite_sensor_rejected':True}
    finally:h.close()


def torque_clamp(root):
    h=Harness(root/'torque_clamp');states=[];epochs=[]
    epoch_sub=h.node.create_subscription(UInt64,'episode_start_ns',epochs.append,h.retained)
    ready=h.node.create_publisher(UInt64,'controller_ready',h.retained)
    command=h.node.create_publisher(WheelTorqueCommand,'wheel_torque',h.qos)
    def control(state):
        states.append(state)
        if round(state.episode_time*1000)%2==0:
            torque=WheelTorqueCommand();torque.header=state.header;torque.state_stamp=state.header.stamp;torque.epoch=state.epoch
            torque.left=-3.0;torque.right=3.0;torque.valid=True
            if state.episode_time>=.01: torque.header.stamp=Clock().clock
            command.publish(torque)
    sub=h.node.create_subscription(RobotState,'robot_state',control,h.qos)
    h.start('backend_node',duration=.02,trace=str(h.directory/'trace.csv'),upright_reset='true')
    h.start('estimator_node')
    try:
        h.spin(lambda:bool(epochs) and command.get_subscription_count()>0)
        ready.publish(epochs[-1]);h.spin(lambda:states and states[-1].episode_time>=.019-1e-9)
        until=time.monotonic()+.1;h.spin(lambda:time.monotonic()>=until)
    finally:h.close()
    trace=np.genfromtxt(h.directory/'trace.csv',delimiter=',',names=True)
    assert len(trace)==20
    active=trace['time']<=.01+1e-9
    assert np.all(trace['applied_l'][active]==-2) and np.all(trace['applied_r'][active]==2)
    assert np.all(trace['applied_l'][~active]==0) and np.all(trace['applied_r'][~active]==0)
    assert trace['yaw_rate'][-1]>0
    return {'backend_clamp_and_left_right_sign':True,'stale_command_timestamp_zero':True}



def policy_transport_parity(root):
    errors=[]
    lines=Path('artifacts/learning/selected_policy.parity.txt').read_text().splitlines()
    for index,line in enumerate(lines):
        fields=line.split();observation=np.array([float(x) for x in fields[1:7]]);expected=np.clip([float(x) for x in fields[8:10]],-1,1)*2
        h=Harness(root/f'policy_parity_{index}');ready=[];commands=[]
        ack=h.node.create_subscription(UInt64,'controller_ready',ready.append,h.retained)
        sub=h.node.create_subscription(WheelTorqueCommand,'wheel_torque',commands.append,h.qos)
        epoch=h.node.create_publisher(UInt64,'episode_start_ns',h.retained);clock=h.node.create_publisher(Clock,'/clock',h.qos)
        state=h.node.create_publisher(RobotState,'robot_state',h.qos);odom=h.node.create_publisher(Odometry,'odom',h.qos)
        h.start('ppo_node',actor=str(Path('artifacts/learning/selected_policy.txt').resolve()),goal_sequence='false',goal_x=float(observation[0]),goal_y=float(observation[1]))
        try:
            h.spin(lambda:epoch.get_subscription_count()>0 and state.get_subscription_count()>0 and odom.get_subscription_count()>0)
            epoch.publish(UInt64(data=index*10**9));h.spin(lambda:bool(ready) and sub.get_publisher_count()>0)
            c=Clock();c.clock.sec=index;clock.publish(c)
            s=RobotState();s.epoch=index*10**9;s.header.stamp=c.clock;s.header.frame_id='odom';s.forward_velocity=float(observation[2]);s.pitch=float(observation[3]*.35);s.pitch_rate=float(observation[4]*2);s.yaw_rate=float(observation[5]*2)
            o=Odometry();o.header=s.header;o.pose.pose.orientation.w=1.;odom.publish(o);state.publish(s)
            h.spin(lambda:any(x.valid for x in commands));command=next(x for x in commands if x.valid)
            error=float(np.max(np.abs(np.array([command.left,command.right])-expected)));assert error<2e-6;errors.append(error)
        finally:h.close()
    bad=subprocess.run(['ros2_ws/install/robot_ros/lib/robot_ros/ppo_node','--ros-args','-p','actor:=/nonexistent/actor.txt'],capture_output=True,text=True,timeout=10)
    assert bad.returncode!=0
    (root/'invalid_actor.log').write_text(bad.stdout+bad.stderr)
    return {'ros_policy_parity_cases':len(errors),'ros_policy_torque_maximum_error':max(errors),'missing_actor_fails_visibly':True}


def main():
    rclpy.init();root=Path('artifacts/ros2/d17/faults')
    try:
        result={**stale_state(root),**sensor_pairing(root),**torque_clamp(root),**reset_and_loss(root),**policy_transport_parity(root)}
        (root/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(result)
    finally:rclpy.shutdown()


if __name__=='__main__':main()
