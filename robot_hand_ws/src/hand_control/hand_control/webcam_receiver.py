"""Receive newline-delimited CSV hand flexion angles in degrees over TCP."""

import math
import socket
import threading
import time

from ament_index_python.packages import get_package_share_path
from hand_control.model import JOINT_NAMES, load_joints
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
import xacro


def parse_angles(line, joints):
    """Validate a whole sample before converting and clamping to URDF limits."""
    values = [float(value) for value in line.decode('ascii').split(',')]
    if len(values) != len(joints) or not all(math.isfinite(v) for v in values):
        raise ValueError('Expected 14 finite angles in degrees')
    return [max(lower, min(upper, math.radians(value)))
            for value, (_, lower, upper, _) in zip(values, joints)]


class LineBuffer:
    """Bound partial lines and recover at the next newline after oversized input."""

    def __init__(self, limit=4096):
        self.limit = limit
        self.buffer = b''
        self.discarding = False

    def feed(self, data):
        """Yield complete lines, supporting fragmented and coalesced TCP reads."""
        for part in data.splitlines(keepends=True):
            # Only LF is a frame delimiter; CR is allowed as trailing whitespace.
            self.buffer += part
            if len(self.buffer) > self.limit:
                self.discarding = True
                self.buffer = b''
            if part.endswith(b'\n'):
                if not self.discarding:
                    yield self.buffer[:-1]
                self.buffer = b''
                self.discarding = False


def advance(positions, targets, joints, alpha, dt):
    """Smooth at the publishing rate and bound each joint's speed."""
    result = []
    for position, target, joint in zip(positions, targets, joints):
        step = alpha * (target - position)
        max_step = joint[3] * dt
        result.append(position + max(-max_step, min(max_step, step)))
    return result


class WebcamReceiver(Node):
    """Keep TCP reception independent from 20 Hz ROS publication."""

    def __init__(self):
        super().__init__('hand_webcam_receiver')
        default_model = str(
            get_package_share_path('robot_hand_description') / 'urdf/hand.urdf.xacro')
        for name, value in [('model', default_model), ('host', '127.0.0.1'),
                            ('port', 8766), ('alpha', 0.2), ('stale_timeout', 1.0)]:
            self.declare_parameter(name, value)
        metadata = load_joints(xacro.process_file(self.get_parameter('model').value).toxml())
        by_name = {joint[0]: joint for joint in metadata}
        if set(by_name) != set(JOINT_NAMES):
            raise ValueError('Model must contain exactly the 14 expected hand joints')
        self.joints = [by_name[name] for name in JOINT_NAMES]
        self.alpha = float(self.get_parameter('alpha').value)
        self.stale_timeout = float(self.get_parameter('stale_timeout').value)
        if not 0.0 < self.alpha <= 1.0:
            raise ValueError('alpha must be in (0, 1]')
        if not math.isfinite(self.stale_timeout) or self.stale_timeout <= 0:
            raise ValueError('stale_timeout must be positive and finite')
        self.positions = [joint[1] for joint in self.joints]
        self.targets = list(self.positions)
        self.last_sample = None
        self.connected = False
        self.holding = True
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.connection = None
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server.bind((self.get_parameter('host').value,
                              self.get_parameter('port').value))
            self.server.listen(1)
            self.server.settimeout(0.2)
        except Exception:
            self.server.close()
            raise
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.last_tick = time.monotonic()
        self.timer = self.create_timer(0.05, self.publish_angles)
        self.worker = threading.Thread(target=self.receive_angles, daemon=True)
        self.worker.start()
        self.get_logger().info(f'Listening on {self.server.getsockname()} (14 CSV degrees)')

    def receive_angles(self):
        """Accept successive clients; malformed frames never replace valid targets."""
        while not self.stop.is_set():
            try:
                connection, address = self.server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with self.lock:
                self.connection = connection
                self.connected = True
                self.last_sample = None
            self.get_logger().info(f'Connected: {address}')
            try:
                with connection:
                    connection.settimeout(0.2)
                    lines = LineBuffer()
                    while not self.stop.is_set():
                        try:
                            data = connection.recv(1024)
                        except socket.timeout:
                            continue
                        if not data:
                            break
                        for line in lines.feed(data):
                            try:
                                targets = parse_angles(line, self.joints)
                            except (ValueError, UnicodeError):
                                continue
                            with self.lock:
                                self.targets = targets
                                self.last_sample = time.monotonic()
            except OSError as error:
                if not self.stop.is_set():
                    self.get_logger().warning(f'TCP connection ended: {error}')
            finally:
                with self.lock:
                    self.connected = False
                    self.connection = None
                if not self.stop.is_set():
                    self.get_logger().warning('Client disconnected; holding pose')

    def publish_angles(self):
        """Hold the current pose when disconnected or when input expires."""
        now = time.monotonic()
        dt = min(0.05, max(0.0, now - self.last_tick))
        self.last_tick = now
        with self.lock:
            active = (self.connected and self.last_sample is not None
                      and now - self.last_sample <= self.stale_timeout)
            if active:
                self.positions = advance(
                    self.positions, self.targets, self.joints, self.alpha, dt)
            if not active and not self.holding:
                self.get_logger().warning('Input unavailable or stale; holding pose')
            self.holding = not active
            positions = list(self.positions)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = positions
        self.publisher.publish(msg)

    def destroy_node(self):
        """Unblock socket reads and join the receiver before destroying ROS entities."""
        self.stop.set()
        self.timer.cancel()
        self.server.close()
        with self.lock:
            connection = self.connection
        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self.worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    """Run the hand TCP receiver."""
    rclpy.init(args=args)
    node = None
    try:
        node = WebcamReceiver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
