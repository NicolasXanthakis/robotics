import socket
import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class WebcamReceiver(Node):
    def __init__(self):
        super().__init__('webcam_receiver')

        self.publisher = self.create_publisher(
            JointState, '/joint_states', 10
        )

        self.shoulder_angle = 0.0
        self.elbow_angle = 0.0
        self.shoulder_yaw = 0.0
        self.torso_yaw = 0.0

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind(('127.0.0.1', 8765))
        self.server.listen(1)

        threading.Thread(
            target=self.receive_angles,
            daemon=True
        ).start()

        self.timer = self.create_timer(0.05, self.publish_angles)

    def receive_angles(self):
        while True:
            connection, address = self.server.accept()
            self.get_logger().info(f'Connected: {address}')

            with connection:
                buffer = ''

                while True:
                    data = connection.recv(1024)

                    if not data:
                        break

                    buffer += data.decode('utf-8')

                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)

                        try:
                            shoulder_deg, elbow_deg, yaw_deg, torso_deg = map(float, line.split(','))

                            if all(math.isfinite(v) for v in [shoulder_deg, elbow_deg, yaw_deg, torso_deg]):
                                self.shoulder_angle = max(-2.8,min(2.8, -math.radians(shoulder_deg+180)))

                                # Calculate the new elbow angle
                                new_elbow_angle = max(
                                    -2.8,
                                    min(2.8, math.radians(180 - elbow_deg))
                                )

                                # Smooth the movement
                                alpha = 0.2

                                self.elbow_angle = (
                                    alpha * new_elbow_angle
                                    + (1 - alpha) * self.elbow_angle
                                )
                                
                                self.shoulder_yaw = max(-3.14, min(3.14, math.radians(yaw_deg)))
                                
                                self.torso_yaw = max(
                                    -1.57,
                                    min(1.57, math.radians(torso_deg))
                                )
                        except ValueError:
                            pass

    def publish_angles(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = [
            'torso_yaw_joint',
            'shoulder_yaw_joint',
            'shoulder_joint',
            'elbow_joint'
        ]

        msg.position = [
            self.torso_yaw,
            self.shoulder_yaw,
            self.shoulder_angle,
            self.elbow_angle
        ]

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = WebcamReceiver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()