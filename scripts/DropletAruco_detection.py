#!/usr/bin/env python3
"""RealSense ArUco/droplet detector for ROS 2.

The original xarm_msgs/XyzCoor messages do not exist in xarm_ros2.  This
version publishes the same physical coordinates with standard PointStamped:
`droplet_xyz_in_camera` and `marker_xyz_in_camera`.  Marker IDs are encoded
as `camera_link/aruco_<id>` in the latter message's frame_id.
"""
import time
import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


class Detector(Node):
    def __init__(self):
        super().__init__('droplet_aruco_detection')
        self.declare_parameter('marker_size_m', 0.045)
        self.declare_parameter('publish_period_s', 1.0)
        self.declare_parameter('show_window', True)
        self.declare_parameter('yolo_weights', '')
        self.marker_size = float(self.get_parameter('marker_size_m').value)
        self.period = float(self.get_parameter('publish_period_s').value)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.yolo_weights = self.get_parameter('yolo_weights').value
        self.yolo_detect = None
        if self.yolo_weights:
            try:
                from yolov5 import detect
                self.yolo_detect = detect
            except ImportError:
                self.get_logger().error('yolo_weights was set, but the yolov5 Python package is unavailable.')
        self.droplet_pub = self.create_publisher(PointStamped, 'droplet_xyz_in_camera', 10)
        self.marker_pub = self.create_publisher(PointStamped, 'marker_xyz_in_camera', 10)
        self.pipeline = rs.pipeline(); config = rs.config()
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.pipeline.start(config); self.align = rs.align(rs.stream.color)
        self.aruco = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters()
        self.last_publish = 0.0
        self.timer = self.create_timer(1.0 / 30.0, self.process)

    @staticmethod
    def camera_to_ros(point):
        # RealSense optical: right, down, forward -> camera_link: forward, left, up.
        return point[2], -point[0], -point[1]

    def publish(self, publisher, point, frame_id):
        message = PointStamped(); message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        message.point.x, message.point.y, message.point.z = map(float, self.camera_to_ros(point))
        publisher.publish(message)

    def process(self):
        try:
            frames = self.align.process(self.pipeline.wait_for_frames(timeout_ms=100))
            depth = frames.get_depth_frame(); color = frames.get_color_frame()
            if not depth or not color: return
            image = np.asanyarray(color.get_data())
            intrinsics = color.profile.as_video_stream_profile().intrinsics
            matrix = np.array([[intrinsics.fx, 0, intrinsics.ppx], [0, intrinsics.fy, intrinsics.ppy], [0, 0, 1]])
            corners, ids, _ = cv2.aruco.detectMarkers(image, self.aruco, parameters=self.params)
            annotated = image.copy()
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
            now = time.monotonic()
            if self.yolo_detect is not None:
                _, boxes, _, confidences = self.yolo_detect.run(
                    weights=self.yolo_weights, source=image, nosave=True)
                boxes, confidences = np.asarray(boxes), np.asarray(confidences)
                if len(boxes) >= 2 and now - self.last_publish >= self.period:
                    # Keep the ROS 1 policy: use the midpoint of the two strongest droplets.
                    best = np.argsort(confidences)[-2:]
                    h, w = image.shape[:2]
                    midpoint = np.mean(boxes[best, :2] * np.array([w, h]), axis=0).astype(int)
                    distance = depth.get_distance(int(midpoint[0]), int(midpoint[1]))
                    if distance > 0:
                        point = rs.rs2_deproject_pixel_to_point(intrinsics, midpoint.tolist(), distance)
                        self.publish(self.droplet_pub, point, 'camera_link')
            for corner, marker_id in zip(corners or [], ids.flatten() if ids is not None else []):
                center = np.mean(corner[0], axis=0).astype(int)
                distance = depth.get_distance(int(center[0]), int(center[1]))
                if distance <= 0: continue
                point = rs.rs2_deproject_pixel_to_point(intrinsics, center.tolist(), distance)
                if now - self.last_publish >= self.period:
                    self.publish(self.marker_pub, point, f'camera_link/aruco_{int(marker_id)}')
            if now - self.last_publish >= self.period: self.last_publish = now
            if self.show_window:
                cv2.imshow('detection', annotated); cv2.waitKey(1)
        except RuntimeError as error:
            self.get_logger().warning(f'RealSense frame error: {error}')

    def destroy_node(self):
        self.pipeline.stop(); cv2.destroyAllWindows()
        return super().destroy_node()


def main():
    rclpy.init(); node = Detector()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
