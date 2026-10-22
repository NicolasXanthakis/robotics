# Robotic hand visualization

Build and run from this workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch hand_control visualize.launch.py
```

RViz uses `palm` as its fixed frame. All 14 joints start open, reach their
URDF upper limits after four seconds, and return open after eight seconds.
The publisher reads limits from the same Xacro as robot_state_publisher and
publishes `/joint_states` at 20 Hz. The default sweep respects the model's
velocity limits. Shorter periods that exceed those limits are rejected.

Launch options:

```bash
ros2 launch hand_control visualize.launch.py rviz:=false
ros2 launch hand_control visualize.launch.py period:=12.0
ros2 launch hand_control visualize.launch.py test_motion:=false
```

Disable test motion when supplying joint states from another source. With
`test_motion:=false`, moving-link transforms require an external publisher.
Run the demo in a separate ROS domain if the arm workspace is running too,
because both use `/joint_states` and `/robot_description`.

RViz requires a working graphical display (for example WSLg).
No webcam, MediaPipe, TCP, physics, or motor interface is involved.

Validation:

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```
