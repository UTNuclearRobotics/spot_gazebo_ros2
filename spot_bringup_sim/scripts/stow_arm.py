#!/usr/bin/env python3
import rclpy
import rclpy.parameter
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Duration
import time

STOW = {
    'arm0_shoulder_yaw':   -0.0011,
    'arm0_shoulder_pitch': -3.1360,
    'arm0_elbow_pitch':     3.1337,
    'arm0_elbow_roll':      1.5699,
    'arm0_wrist_pitch':    -0.0027,
    'arm0_wrist_roll':     -1.5684,
    'arm0_fingers':        -0.7223,
}
TOLERANCE = 0.05  # rad


class StowArm(Node):
    def __init__(self):
        super().__init__('stow_arm', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])
        self.pub = self.create_publisher(JointTrajectory, '/spot/joint_trajectory', 10)
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.create_subscription(JointState, '/spot/joint_states', self.on_joint_state, 10)

        self.last_sim_time = None
        self.sim_running = False
        self.publish_timer = None
        self.current_positions = {}
        self.done = False
        self.get_logger().info('Waiting for simulation to start...')

    def on_clock(self, msg):
        if self.done or self.sim_running:
            return
        t = msg.clock.sec * 1e9 + msg.clock.nanosec
        if self.last_sim_time is not None and t > self.last_sim_time:
            self.sim_running = True
            self.get_logger().info('Simulation running — starting stow.')
            # Publish immediately, then repeat every second until done
            self.publish_stow()
            self.publish_timer = self.create_timer(1.0, self.publish_stow)
        self.last_sim_time = t

    def on_joint_state(self, msg):
        if self.done:
            return
        for name, pos in zip(msg.name, msg.position):
            if name in STOW:
                self.current_positions[name] = pos
        if len(self.current_positions) == len(STOW) and self.at_stow():
            self.get_logger().info('Arm is stowed.')
            if self.publish_timer:
                self.publish_timer.cancel()
            self.done = True

    def at_stow(self):
        return all(
            abs(self.current_positions.get(j, 999) - target) < TOLERANCE
            for j, target in STOW.items()
        )

    def publish_stow(self):
        if self.done:
            return
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(STOW.keys())
        point = JointTrajectoryPoint()
        point.positions = list(STOW.values())
        point.time_from_start = Duration(sec=5)
        msg.points = [point]
        self.pub.publish(msg)
        self.get_logger().info('Stow trajectory published...')


def main():
    rclpy.init()
    node = StowArm()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()