# lite6_labauto (ROS 2 Humble)

ROS 2 Humble port of the Lite6 laboratory-automation control layer.  The
project's `MovePose.msg`, `LabwareOBB.msg`, and Arduino firmware are retained
unchanged.

## Dependencies

Install ROS 2 Humble, the Humble branch of
[xarm_ros2](https://github.com/xArm-Developer/xarm_ros2/tree/humble),
`realsense2_camera`, and Python packages `pyserial`, `numpy`, `opencv-python`
and (only for the camera node) `pyrealsense2`.

## Build

```bash
mkdir -p ~/ros2_ws/src
cp -a lite6_labauto ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select lite6_labauto
source install/setup.bash
```

## Run

```bash
ros2 launch lite6_labauto lite6_labauto.launch.py robot_ip:=192.168.0.166
ros2 run lite6_labauto robot_init.py
ros2 run lite6_labauto robot_move.py
ros2 run lite6_labauto robot_disable.py
```

The Humble Lite6 driver exposes `/ufactory/motion_enable`, `/ufactory/set_mode`,
`/ufactory/set_state`, and `/ufactory/set_position`.  `robot_move.py` translates
the existing `/arm_control/move_pose` (`lite6_labauto/msg/MovePose`) into
`/ufactory/set_position` (`xarm_msgs/srv/MoveCartesian`).

### Vision-node migration

The ROS 1 vision scripts referenced `xarm_msgs/XyzCoor` and
`MarkerID_XyzCoor`; those message types are not supplied by the Humble
`xarm_ros2` driver. The ROS 2 scripts use `geometry_msgs/msg/PointStamped` on
the same topic names instead. Camera coordinates are normalized to
`camera_link`; marker IDs are retained in `header.frame_id` as
`camera_link/aruco_<id>`.
