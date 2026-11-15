#!/usr/bin/env python3
"""Optional ROS 2 plant adapter. All command topics are under /postal_sim.

Use a dedicated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1. This module has no Station API.
"""
import argparse
import json
import time

import mujoco
import numpy as np

from .common import KiwiKinematics, compose, config, rotation
from .env import PostalEnv


def main():
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from builtin_interfaces.msg import Time
        from geometry_msgs.msg import Twist, TransformStamped
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan, JointState, Imu
        from std_msgs.msg import String
        from tf2_ros import TransformBroadcaster
    except ImportError as exc:
        raise SystemExit('ROS 2 absent du PYTHONPATH ; source /opt/ros/humble/setup.bash avant de lancer le pont') from exc
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config')
    parser.add_argument('--duration', type=float, default=float('inf'))
    parser.add_argument('--fast', action='store_true')
    args = parser.parse_args()
    c = config(args.config)
    env = PostalEnv(c)
    carry = env.arm.ik([0, .13, .2], np.zeros(7))
    observation, _ = env.reset(seed=1, options={'arm_q': carry})
    kinematics = KiwiKinematics(c)
    rclpy.init(args=[])

    class Bridge(Node):
        def __init__(self):
            super().__init__('mujoco_plant', namespace='postal_sim', cli_args=[
                '--ros-args', '-r', '/tf:=/postal_sim/tf', '-r', '/tf_static:=/postal_sim/tf_static'])
            self.twist, self.arm, self.gripper = np.zeros(3), carry.copy(), 0.0
            self.received = -np.inf
            self.odometry_pose = np.zeros(3)
            self.scan = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
            self.odom = self.create_publisher(Odometry, 'odom', 10)
            self.joints = self.create_publisher(JointState, 'joint_states', 10)
            self.imu = self.create_publisher(Imu, 'imu', qos_profile_sensor_data)
            self.clock = self.create_publisher(Clock, '/clock', 10)
            self.feedback = self.create_publisher(String, 'task_feedback', 10)
            self.transforms = TransformBroadcaster(self)
            self.create_subscription(Twist, 'cmd_vel', self.velocity, 10)
            self.create_subscription(JointState, 'arm_target', self.arm_target, 10)
            self.create_subscription(String, 'task_command', self.task_command, 10)

        def velocity(self, message):
            values = np.array([message.linear.x, message.linear.y, message.angular.z])
            if np.all(np.isfinite(values)):
                values[:2] *= min(1, c['navigation']['max_speed_m_s']/max(np.linalg.norm(values[:2]), 1e-9))
                values[2] = np.clip(values[2], -c['navigation']['max_yaw_rad_s'], c['navigation']['max_yaw_rad_s'])
                self.twist, self.received = values, env.data.time

        def arm_target(self, message):
            try:
                positions = dict(zip(message.name, message.position))
                target = np.array([positions[f'rev_motor_{i:02d}'] for i in range(1, 8)])
                self.arm = env.arm.check(target)
                if 'rev_motor_08' in positions:
                    value = positions['rev_motor_08']
                    if not 0 <= value <= 2.2028: raise ValueError('pince hors butées')
                    self.gripper = value
            except (KeyError, ValueError) as exc:
                self.get_logger().error(f'Consigne rejetée : {exc}')

        def task_command(self, message):
            try:
                command = json.loads(message.data)
                if command['type'] == 'grasp': result = dict(grasped=env.grasp(command['letter']))
                elif command['type'] == 'release': result = dict(released=env.release())
                else: raise ValueError('type de commande inconnu')
                self.feedback.publish(String(data=json.dumps(result)))
            except (ValueError, KeyError) as exc: self.get_logger().error(str(exc))

        def publish(self, observation):
            now = float(observation['time'][0]); stamp = Time(sec=int(now), nanosec=int((now % 1)*1e9))
            self.clock.publish(Clock(clock=stamp))
            increment = kinematics.body(observation['wheel_delta'])
            increment[:2] = rotation(increment[2]/2) @ increment[:2]
            self.odometry_pose = compose(self.odometry_pose, increment)
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'postal_sim/odom', 'postal_sim/base_link'
            odom.pose.pose.position.x, odom.pose.pose.position.y = map(float, self.odometry_pose[:2])
            yaw = self.odometry_pose[2]
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = float(np.sin(yaw/2)), float(np.cos(yaw/2))
            velocity = kinematics.body(observation['wheel_velocity']*c['odometry']['wheel_scale'])
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = map(float, velocity)
            odom.pose.covariance[0], odom.pose.covariance[7], odom.pose.covariance[35] = .0025, .0025, .01
            self.odom.publish(odom)
            transform = TransformStamped()
            transform.header, transform.child_frame_id = odom.header, odom.child_frame_id
            transform.transform.translation.x, transform.transform.translation.y = map(float, self.odometry_pose[:2])
            transform.transform.translation.z = c['base_height_m']
            transform.transform.rotation = odom.pose.pose.orientation
            laser = TransformStamped()
            laser.header.stamp, laser.header.frame_id, laser.child_frame_id = stamp, 'postal_sim/base_link', 'postal_sim/laser'
            laser.transform.translation.x, laser.transform.translation.y, laser.transform.translation.z = map(float, c['lidar']['xyz_m'])
            laser.transform.rotation.w = 1.0
            self.transforms.sendTransform([transform, laser])
            if observation['scan_fresh']:
                scan = LaserScan()
                scan.header.stamp, scan.header.frame_id = stamp, 'postal_sim/laser'
                scan.angle_min, scan.angle_increment = -np.pi, 2*np.pi/c['lidar']['rays']
                scan.angle_max = scan.angle_min+(c['lidar']['rays']-1)*scan.angle_increment
                scan.range_min, scan.range_max, scan.scan_time = .03, c['lidar']['max_range_m'], c['lidar']['period_s']
                scan.ranges = observation['scan'].tolist()
                self.scan.publish(scan)
            state = JointState()
            state.header.stamp = stamp
            for j in range(env.model.njnt):
                if env.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE: continue
                state.name.append(mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_JOINT, j))
                state.position.append(float(env.data.qpos[env.model.jnt_qposadr[j]]))
                state.velocity.append(float(env.data.qvel[env.model.jnt_dofadr[j]]))
            self.joints.publish(state)
            imu = Imu()
            imu.header.stamp, imu.header.frame_id = stamp, 'postal_sim/base_link'
            imu.orientation_covariance[0] = -1.0  # no oracle orientation supplied
            imu.linear_acceleration_covariance[0] = -1.0  # not published by this bridge
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = map(float, observation['gyro'])
            self.imu.publish(imu)

    node = Bridge()
    try:
        while rclpy.ok() and env.data.time < args.duration:
            start = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0)
            twist = node.twist if env.data.time-node.received < .5 else np.zeros(3)
            observation, _, terminated, _, _ = env.step(dict(twist=twist, arm=node.arm, gripper=[node.gripper]))
            node.publish(observation)
            if terminated: break
            if not args.fast: time.sleep(max(0, c['control_period_s']-(time.monotonic()-start)))
    finally:
        node.destroy_node(); rclpy.shutdown(); env.close()


if __name__ == '__main__': main()
