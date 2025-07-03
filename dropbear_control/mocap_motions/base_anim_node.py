#!/usr/bin/env python3
import os, yaml, rclpy
from rclpy.node          import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg        import Float64MultiArray
from ament_index_python.packages import get_package_share_directory

class BaseAnimNode(Node):
    HIGH_HZ = 60.0               # effort publishers
    LOW_HZ  = 1.0                # trajectory publishers

    def __init__(self, anim_name: str):
        super().__init__(f'{anim_name.lower()}_node')
        self.anim_name = anim_name
        self.pose      = self.load_pose()

        self.init_joint_maps()
        self.init_publishers()

        self.hz_counter = 0
        self.timer = self.create_timer(1.0 / self.HIGH_HZ, self.tick)

    # ---------- core ----------
    def load_pose(self):
        cfg_dir = os.path.join(get_package_share_directory('dropbear'), 'config')
        yaml_path = os.path.join(cfg_dir, 'joint_values.yaml')
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        if self.anim_name not in data:
            self.get_logger().error(f"'{self.anim_name}' not in joint_values.yaml")
            rclpy.shutdown()
        return data[self.anim_name]

    def tick(self):
        # effort controllers (60 Hz)
        self.publish_effort('right_leg_knee',  self.pose['right_leg']['knee'])
        self.publish_effort('left_leg_knee',   self.pose['left_leg']['knee'])
        self.publish_effort('right_leg_lower', self.pose['right_leg']['lower_leg'])
        self.publish_effort('left_leg_lower',  self.pose['left_leg']['lower_leg'])
        self.publish_effort('right_elbow',     self.pose['right_arm']['elbow'])
        self.publish_effort('left_elbow',      self.pose['left_arm']['elbow'])

        # trajectory controllers (1 Hz)
        self.hz_counter += 1
        if self.hz_counter >= self.HIGH_HZ / self.LOW_HZ:
            self.hz_counter = 0
            dur = 0.9
            # hips
            self.publish_traj('right_leg_hip', [self.pose['right_leg']['hip']], dur)
            self.publish_traj('left_leg_hip',  [self.pose['left_leg']['hip' ]], dur)
            # feet
            self.publish_traj('right_leg_foot',[self.pose['right_leg']['foot']],dur)
            self.publish_traj('left_leg_foot', [self.pose['left_leg']['foot']],dur)
            # hands
            rh = self.pose['right_arm']; lh = self.pose['left_arm']
            self.publish_traj('right_hand', [rh[k] for k in ('yaw','pitch','roll','wrist')], dur)
            self.publish_traj('left_hand',  [lh[k] for k in ('yaw','pitch','roll','wrist')], dur)

    # ---------- helpers ----------
    def init_joint_maps(self):
        self.effort_targets = {
            'right_leg_knee': ['/right_leg_knee_controller/commands'],
            'left_leg_knee':  ['/left_leg_knee_controller/commands'],
            'right_leg_lower':['/lower_right_leg_controller/commands'],
            'left_leg_lower': ['/lower_left_leg_controller/commands'],
            'right_elbow':    ['/right_hand_elbow_controller/commands'],
            'left_elbow':     ['/left_hand_elbow_controller/commands'],
        }
        self.traj_targets = {
            'right_leg_hip':  ['/right_leg_hip_joint_controller/joint_trajectory', ['RL_hip_joint']],
            'left_leg_hip':   ['/left_leg_hip_joint_controller/joint_trajectory',  ['LL_hip_joint']],
            'right_leg_foot': ['/right_leg_foot_controller/joint_trajectory',      ['RL_Revolute87']],
            'left_leg_foot':  ['/left_leg_foot_controller/joint_trajectory',       ['LL_Revolute87']],
            'right_hand':     ['/right_hand_controller/joint_trajectory',
                               ['RH_yaw','RH_pitch','RH_roll','RH_wrist_roll']],
            'left_hand':      ['/left_hand_controller/joint_trajectory',
                               ['LH_yaw','LH_pitch','LH_roll','LH_wrist_roll']],
        }

    def init_publishers(self):
        self.pub_effort = {k: self.create_publisher(Float64MultiArray, v[0], 10)
                           for k,v in self.effort_targets.items()}
        self.pub_traj = {k: self.create_publisher(JointTrajectory, v[0], 10)
                         for k,v in self.traj_targets.items()}

    def publish_effort(self, key, value):
        msg = Float64MultiArray(); msg.data = [value]
        self.pub_effort[key].publish(msg)

    def publish_traj(self, key, positions, dur):
        trg = JointTrajectory(); trg.joint_names = self.traj_targets[key][1]
        pt  = JointTrajectoryPoint(); pt.positions = positions
        from rclpy.duration import Duration
        pt.time_from_start = Duration(seconds=dur).to_msg()
        trg.points.append(pt)
        self.pub_traj[key].publish(trg)
