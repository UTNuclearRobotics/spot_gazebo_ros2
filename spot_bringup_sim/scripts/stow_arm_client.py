#!/usr/bin/env python3
"""One-shot startup client: stow Spot's arm via the sim manipulation driver.

Replaces the standalone stow_arm.py trajectory publisher. Waits for sim time
to start advancing (same guard as the original), waits for the manipulation
driver services, then calls ~/stow_arm followed by ~/close_gripper and exits.
"""

import rclpy
import rclpy.parameter
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger

DRIVER_NS = "/spot_manipulation_driver"
SERVICE_WAIT_TIMEOUT_S = 60.0


class StowArmClient(Node):
    def __init__(self):
        super().__init__('stow_arm_client', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])
        self.last_sim_time = None
        self.sim_running = False
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.get_logger().info('Waiting for simulation to start...')

    def on_clock(self, msg):
        if self.sim_running:
            return
        t = msg.clock.sec * 1e9 + msg.clock.nanosec
        if self.last_sim_time is not None and t > self.last_sim_time:
            self.sim_running = True
        self.last_sim_time = t

    def call_and_wait(self, client, request):
        if not client.wait_for_service(timeout_sec=SERVICE_WAIT_TIMEOUT_S):
            self.get_logger().error(f'Service {client.srv_name} not available')
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    rclpy.init()
    node = StowArmClient()

    # Wait for sim time to advance (mirrors the original stow_arm.py guard)
    while rclpy.ok() and not node.sim_running:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info('Simulation running — requesting stow via manipulation driver.')

    stow_client = node.create_client(Trigger, f'{DRIVER_NS}/stow_arm')
    gripper_client = node.create_client(Trigger, f'{DRIVER_NS}/close_gripper')

    resp = node.call_and_wait(stow_client, Trigger.Request())
    if resp is None or not resp.success:
        node.get_logger().error(f'Stow failed: {resp.message if resp else "no response"}')
    else:
        node.get_logger().info(f'Stow: {resp.message}')
        resp = node.call_and_wait(gripper_client, Trigger.Request())
        if resp is None or not resp.success:
            node.get_logger().error(f'Gripper closing failed: {resp.message if resp else "no response"}')
        else:
            node.get_logger().info('Arm stowed via manipulation driver.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
