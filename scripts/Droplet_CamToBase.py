#!/usr/bin/env python3
"""Log droplet coordinates transformed from camera_link into link_base."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point


class CameraToBase(Node):
    def __init__(self):
        super().__init__('droplet_camera_to_base')
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer, self)
        self.create_subscription(PointStamped, 'droplet_xyz_in_camera', self.callback, 10)

    def callback(self, point):
        try:
            transform = self.buffer.lookup_transform('link_base', point.header.frame_id or 'camera_link', rclpy.time.Time())
            self.get_logger().info(f'Coordinate in base frame: {do_transform_point(point, transform)}')
        except TransformException as error:
            self.get_logger().warning(f'Transform unavailable: {error}', throttle_duration_sec=5.0)


def main():
    rclpy.init(); node = CameraToBase()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
