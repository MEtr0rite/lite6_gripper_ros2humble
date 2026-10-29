from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.0.166'),
        DeclareLaunchArgument('custom_gripper', default_value='true'),
        DeclareLaunchArgument('gripper_port', default_value='/dev/lite6_openparallelgripper'),
        DeclareLaunchArgument('gripper_baud', default_value='115200'),
        DeclareLaunchArgument('camera', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('xarm_api'), 'launch', 'lite6_driver.launch.py')),
            launch_arguments={'robot_ip': robot_ip}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')),
            condition=IfCondition(LaunchConfiguration('camera'))),
        Node(package='lite6_labauto', executable='gripper_serial.py',
             name='gripper_serial_node', output='screen', respawn=True,
             condition=IfCondition(LaunchConfiguration('custom_gripper')),
             parameters=[{'port': LaunchConfiguration('gripper_port'),
                          'baud': LaunchConfiguration('gripper_baud')}]),
    ])
