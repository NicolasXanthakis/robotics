"""Run a real TCP/ROS smoke test after building and sourcing the workspace.

Run with: python3 src/hand_control/test/tcp_smoke.py
Uses an isolated ROS domain and port 8766; leave the hand receiver stopped.
"""

import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time

from hand_control.model import JOINT_NAMES
import rclpy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


def main():
    """Check launch, reception, stale/disconnect holds, reconnection, and shutdown."""
    os.environ['ROS_DOMAIN_ID'] = '89'
    with tempfile.TemporaryDirectory(prefix='hand_tcp_test_') as directory:
        os.environ['ROS_LOG_DIR'] = directory
        log_path = Path(directory) / 'launch.log'
        with log_path.open('w') as output:
            process = subprocess.Popen(
                ['ros2', 'launch', 'hand_control', 'tcp_visualize.launch.py',
                 'rviz:=false', 'stale_timeout:=0.3'],
                stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            node = None
            client = None
            try:
                rclpy.init()
                node = rclpy.create_node('hand_tcp_validation')
                samples = []
                frames = set()
                node.create_subscription(JointState, '/joint_states', samples.append, 10)
                node.create_subscription(
                    TFMessage, '/tf',
                    lambda msg: frames.update(t.child_frame_id for t in msg.transforms), 10)

                def spin(seconds):
                    end = time.monotonic() + seconds
                    while time.monotonic() < end:
                        rclpy.spin_once(node, timeout_sec=0.02)
                        assert process.poll() is None, log_path.read_text()

                for _ in range(100):
                    try:
                        client = socket.create_connection(('127.0.0.1', 8766), timeout=0.1)
                        break
                    except OSError:
                        spin(0.1)
                assert client is not None, log_path.read_text()
                spin(0.5)
                assert samples and list(samples[-1].position) == [0.0] * 14
                closed = b','.join([b'90'] * 14) + b'\n'
                client.sendall(closed[:11])
                spin(0.1)
                assert list(samples[-1].position) == [0.0] * 14
                client.sendall(closed[11:] + b'nan\n' + b'\xff\n')
                spin(0.2)
                assert all(value > 0.0 for value in samples[-1].position)
                spin(0.7)
                held = list(samples[-1].position)
                spin(0.3)
                assert list(samples[-1].position) == held, 'Stale input kept moving'
                # Oversized lines are discarded; a following valid line restores motion.
                client.sendall(b'x' * 5000 + b'\n' + closed + closed)
                spin(0.2)
                assert all(p > old for p, old in zip(samples[-1].position, held))
                client.close()
                client = None
                spin(0.2)
                held = list(samples[-1].position)
                spin(0.4)
                assert list(samples[-1].position) == held, 'Disconnected input kept moving'
                client = socket.create_connection(('127.0.0.1', 8766), timeout=1)
                client.sendall(b','.join([b'0'] * 14) + b'\n')
                spin(0.2)
                assert all(p < old for p, old in zip(samples[-1].position, held))
                assert len(frames) == 14, frames
                for msg in samples:
                    assert msg.name == list(JOINT_NAMES)
                    assert len(msg.position) == 14 and msg.header.stamp.sec > 0
                print(f'PASS: {len(samples)} joint samples, 14 TF frames, framing, '
                      'invalid input, stale/disconnect holds and reconnect')
            finally:
                # Send SIGINT only to launch; launch forwards it once to its children.
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                    raise AssertionError('Launch did not shut down cleanly')
                if client is not None:
                    client.close()
                if node is not None:
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                log = log_path.read_text()
                assert 'Traceback' not in log, log
                assert 'process has died' not in log, log


if __name__ == '__main__':
    main()
