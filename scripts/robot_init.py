#!/usr/bin/env python3
"""Enable the Lite6 and move it to the lab's initial Cartesian pose."""
import rclpy
from rclpy.node import Node
from xarm_msgs.srv import MoveCartesian, SetInt16, SetInt16ById


class Initializer(Node):
    def __init__(self):
        super().__init__('xarm_initializer')

    def call(self, srv_type, name, request):
        client = self.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Timeout waiting for {name}')
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        if not future.done() or future.result() is None:
            self.get_logger().error(f'No response from {name}')
            return None
        response = future.result()
        if response.ret != 0:
            self.get_logger().error(f'{name} failed: ret={response.ret}, {response.message}')
            return None
        return response

    def run(self):
        enable = SetInt16ById.Request(); enable.id = 8; enable.data = 1
        if not self.call(SetInt16ById, '/ufactory/motion_enable', enable): return
        mode = SetInt16.Request(); mode.data = 0
        if not self.call(SetInt16, '/ufactory/set_mode', mode): return
        state = SetInt16.Request(); state.data = 0
        if not self.call(SetInt16, '/ufactory/set_state', state): return
        move = MoveCartesian.Request()
        move.pose = [-19.2, 250.6, 400.5, 3.046, 0.11, -1.507]
        move.speed = 100.0; move.acc = 1000.0; move.mvtime = 0.0
        move.wait = True; move.timeout = 30.0; move.radius = 0.0
        self.call(MoveCartesian, '/ufactory/set_position', move)


def main():
    rclpy.init(); node = Initializer()
    try: node.run()
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
