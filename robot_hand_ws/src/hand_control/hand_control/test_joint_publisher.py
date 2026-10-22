"""Move every movable hand joint smoothly between its URDF limits."""

import math

from ament_index_python.packages import get_package_share_path
from hand_control.model import load_joints
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import xacro


def sample_positions(joints, elapsed, period):
    """Start fully open and smoothly close/open once per period."""
    fraction = 0.5 * (1.0 - math.cos(2.0 * math.pi * elapsed / period))
    return [lower + (upper - lower) * fraction for _, lower, upper, _ in joints]


class HandTestPublisher(Node):
    """Publish a complete hand state at 20 Hz."""

    def __init__(self):
        super().__init__('hand_test_publisher')
        default_model = (
            get_package_share_path('robot_hand_description') / 'urdf/hand.urdf.xacro'
        )
        self.declare_parameter('model', str(default_model))
        self.declare_parameter('period', 8.0)
        model = self.get_parameter('model').value
        self.joints = load_joints(xacro.process_file(model).toxml())
        self.period = float(self.get_parameter('period').value)
        # A cosine sweep has maximum speed pi * joint_range / period.
        minimum_period = max(
            math.pi * (upper - lower) / velocity
            for _, lower, upper, velocity in self.joints
        )
        if not math.isfinite(self.period) or self.period < minimum_period:
            raise ValueError(f'period must be finite and >= {minimum_period:.3f} seconds')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.publish_state)

    def publish_state(self):
        """Publish all joint positions using elapsed ROS time."""
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = [joint[0] for joint in self.joints]
        msg.position = sample_positions(self.joints, elapsed, self.period)
        self.publisher.publish(msg)


def main(args=None):
    """Run the demonstration publisher until interrupted."""
    rclpy.init(args=args)
    node = None
    try:
        node = HandTestPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
