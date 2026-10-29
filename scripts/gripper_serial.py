#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class GripperSerialInterface(Node):
    def __init__(self):
        super().__init__('gripper_serial_node')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        self.serial = serial.Serial(port, baud, timeout=0.05)
        self.publisher = self.create_publisher(Float32, '/gripper/current_distance', 10)
        self.subscription = self.create_subscription(Float32, '/gripper/target_distance', self.target_callback, 10)
        self.timer = self.create_timer(0.05, self.read_distance)
        self.get_logger().info(f'Gripper serial connected on {port} at {baud} baud.')

    def target_callback(self, message):
        command = f'{message.data:.2f}\n'
        self.serial.write(command.encode('utf-8'))
        self.get_logger().info(f'Sent target distance: {command.strip()}')

    def read_distance(self):
        try:
            line = self.serial.readline().decode('utf-8', errors='replace').strip()
            if line:
                self.publisher.publish(Float32(data=float(line)))
        except (ValueError, serial.SerialException) as error:
            self.get_logger().warning(f'Serial read error: {error}', throttle_duration_sec=5.0)

    def destroy_node(self):
        if self.serial.is_open: self.serial.close()
        return super().destroy_node()


def main():
    rclpy.init(); node = GripperSerialInterface()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
