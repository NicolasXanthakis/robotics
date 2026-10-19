import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')

        self.publisher = self.create_publisher(
            JointState, '/joint_states', 10
        )

        self.timer = self.create_timer(0.05, self.publish_angles)
        self.time = 0.0

    def publish_angles(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = ['shoulder_joint', 'elbow_joint']
        msg.position = [
            0.8 * math.sin(self.time),
            0.6 * math.cos(self.time)
        ]

        self.publisher.publish(msg)
        self.time += 0.05


def main():
    rclpy.init()
    node = JointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()