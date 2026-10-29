#!/usr/bin/env python3
"""Bridge MovePose topic commands to the ROS 2 Lite6 Cartesian service."""
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from xarm_msgs.srv import MoveCartesian
from lite6_labauto.msg import MovePose


class ArmMotionBridge(Node):
    def __init__(self):
        super().__init__('robot_motion_caller')
        self.declare_parameter('speed', 200.0)
        self.declare_parameter('acc', 1000.0)
        self.declare_parameter('mvtime', 0.0)
        self.declare_parameter('pose_tolerance', 0.1)
        self.speed = float(self.get_parameter('speed').value)
        self.acc = float(self.get_parameter('acc').value)
        self.mvtime = float(self.get_parameter('mvtime').value)
        self.tolerance = float(self.get_parameter('pose_tolerance').value)
        self.previous_pose = None
        self.move_done = self.create_publisher(Bool, '/arm_control/move_done', 1)
        self.client = self.create_client(MoveCartesian, '/ufactory/set_position')
        self.subscription = self.create_subscription(
            MovePose, '/arm_control/move_pose', self.pose_callback, 10)

    def pose_callback(self, message):
        pose = np.asarray(message.pose, dtype=float)
        if self.previous_pose is not None and np.all(np.abs(pose - self.previous_pose) < self.tolerance):
            self.get_logger().info('Ignoring duplicate target pose within tolerance.')
            return
        if not self.client.service_is_ready():
            self.get_logger().warning('/ufactory/set_position is not available.')
            self.move_done.publish(Bool(data=False)); return
        self.previous_pose = pose
        request = MoveCartesian.Request()
        request.pose = pose.tolist(); request.speed = self.speed; request.acc = self.acc
        request.mvtime = self.mvtime; request.wait = True; request.timeout = 30.0; request.radius = 0.0
        self.move_done.publish(Bool(data=False))
        future = self.client.call_async(request)
        future.add_done_callback(self.on_response)

    def on_response(self, future):
        try:
            response = future.result()
            ok = response is not None and response.ret == 0
            if ok: self.get_logger().info('set_position completed.')
            else: self.get_logger().error(f'set_position failed: {response}')
            self.move_done.publish(Bool(data=ok))
        except Exception as error:
            self.get_logger().error(f'set_position service error: {error}')
            self.move_done.publish(Bool(data=False))


def main():
    rclpy.init(); node = ArmMotionBridge()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
