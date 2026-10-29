#!/usr/bin/env python3
"""Log ArUco marker coordinates transformed from camera_link into link_base.

Marker IDs are retained in the PointStamped frame_id as camera_link/aruco_<id>.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point


class MarkerCameraToBase(Node):
    def __init__(self):
        super().__init__('marker_camera_to_base')
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer, self)
        self.create_subscription(PointStamped, 'marker_xyz_in_camera', self.callback, 10)

    def callback(self, point):
        try:
            transform = self.buffer.lookup_transform('link_base', 'camera_link', rclpy.time.Time())
            self.get_logger().info(f'Coordinate in base frame: {do_transform_point(point, transform)}')
        except TransformException as error:
            self.get_logger().warning(f'Transform unavailable: {error}')


def main():
    rclpy.init(); node = MarkerCameraToBase()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
