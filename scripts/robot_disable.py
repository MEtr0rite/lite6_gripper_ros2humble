#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from xarm_msgs.srv import SetInt16ById


def main():
    rclpy.init(); node = Node('xarm_disabler')
    client = node.create_client(SetInt16ById, '/ufactory/motion_enable')
    try:
        if not client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error('Timeout waiting for /ufactory/motion_enable'); return
        request = SetInt16ById.Request(); request.id = 8; request.data = 0
        future = client.call_async(request); rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        response = future.result()
        if response is None: node.get_logger().error('No response from motion_enable')
        elif response.ret == 0: node.get_logger().info('xArm disabled successfully.')
        else: node.get_logger().error(f'Disable failed: ret={response.ret}, {response.message}')
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
