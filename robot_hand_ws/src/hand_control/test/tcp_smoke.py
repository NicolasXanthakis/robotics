"""Run a real TCP/ROS smoke test after building and sourcing the workspace.

Run with: python3 src/hand_control/test/tcp_smoke.py
Uses an isolated ROS domain and an ephemeral port; leave the hand receiver stopped.
"""

import os
import json
import math
import sys
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time

from hand_control.model import JOINT_NAMES, SPREAD_NAMES
import rclpy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


def main():
    """Check launch, reception, stale/disconnect holds, reconnection, and shutdown."""
    wrist = '--wrist' in sys.argv
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    os.environ['ROS_DOMAIN_ID'] = '89'
    with tempfile.TemporaryDirectory(prefix='hand_tcp_test_') as directory:
        os.environ['ROS_LOG_DIR'] = directory
        log_path = Path(directory) / 'launch.log'
        with log_path.open('w') as output:
            process = subprocess.Popen(
                ['ros2', 'launch', 'hand_control',
                 'wrist_visualize.launch.py' if wrist else 'tcp_visualize.launch.py',
                 'rviz:=false', f'port:={port}'] + (['alpha:=0.2', 'motion_speed_scale:=1.0']
                                    if wrist else ['stale_timeout:=0.3']),
                stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            node = None
            client = None
            try:
                rclpy.init()
                node = rclpy.create_node('hand_tcp_validation')
                samples = []
                frames = set()
                wrist_transforms = []

                def collect_tf(msg):
                    frames.update(t.child_frame_id for t in msg.transforms)
                    wrist_transforms.extend(t for t in msg.transforms
                                            if t.child_frame_id == 'palm')
                node.create_subscription(JointState, '/joint_states', samples.append, 10)
                node.create_subscription(
                    TFMessage, '/tf',
                    collect_tf, 10)

                def spin(seconds):
                    end = time.monotonic() + seconds
                    while time.monotonic() < end:
                        rclpy.spin_once(node, timeout_sec=0.02)
                        assert process.poll() is None, log_path.read_text()

                for _ in range(100):
                    try:
                        client = socket.create_connection(('127.0.0.1', port), timeout=0.1)
                        break
                    except OSError:
                        spin(0.1)
                assert client is not None, log_path.read_text()
                spin(0.5)
                assert samples and list(samples[-1].position) == [0.0] * 20
                closed = b','.join([b'90'] * 14) + b'\n'
                client.sendall(closed[:11])
                spin(0.1)
                assert list(samples[-1].position) == [0.0] * 20
                client.sendall(closed[11:] + b'nan\n' + b'\xff\n')
                spin(0.2)
                assert all(value > 0.0 for value in samples[-1].position[:14])
                spin(1.3 if wrist else 0.7)
                held = list(samples[-1].position)
                spin(0.3)
                assert list(samples[-1].position) == held, 'Stale input kept moving'
                # Oversized lines are discarded; a following valid line restores motion.
                client.sendall(b'x' * 5000 + b'\n' + closed + closed)
                spin(0.2)
                assert all(p > old for p, old in zip(samples[-1].position[:14], held[:14]))
                client.close()
                client = None
                spin(0.2)
                held = list(samples[-1].position)
                spin(0.4)
                assert list(samples[-1].position) == held, 'Disconnected input kept moving'
                client = socket.create_connection(('127.0.0.1', port), timeout=1)
                client.sendall(b','.join([b'0'] * 14) + b'\n')
                spin(0.2)
                assert all(p < old for p, old in zip(samples[-1].position[:14], held[:14]))
                assert len(frames) == (21 if wrist else 20), frames
                if wrist:
                    def send_wrist(sequence, valid=True):
                        client.sendall((json.dumps({
                            'version': 2, 'sequence': sequence, 'frame_id': 'hand_base',
                            'child_frame_id': 'palm', 'handedness': 'right',
                            'finger_angles_deg': [30]*14, 'finger_spread_deg': [20, 5, -5, -20],
                            'orientation_valid': valid, 'thumb_spread_deg': 30, 'thumb_opposition_deg': 40,
                            'wrist_quaternion_xyzw': [0, 0, math.sin(.4), math.cos(.4)]
                            if valid else None}) + '\n').encode())

                    for sequence in range(12):
                        send_wrist(sequence)
                        spin(.06)
                    assert wrist_transforms
                    assert samples[-1].position[19] > .1
                    assert samples[-1].position[18] > .1
                    assert samples[-1].position[14] > .1
                    assert samples[-1].position[17] < -.1
                    transform = wrist_transforms[-1]
                    assert transform.header.frame_id == 'hand_base'
                    assert abs(transform.transform.rotation.z) > .05
                    send_wrist(12, False)
                    spin(.15)
                    held_rotation = wrist_transforms[-1].transform.rotation
                    spin(.3)
                    assert wrist_transforms[-1].transform.rotation == held_rotation
                    assert all(p > 0 for p in samples[-1].position[:14])
                    client.close()
                    spin(.2)
                    assert wrist_transforms[-1].transform.rotation == held_rotation
                    client = socket.create_connection(('127.0.0.1', port), timeout=1)
                    for sequence in range(5):
                        send_wrist(sequence)
                        spin(.06)
                    assert wrist_transforms[-1].transform.rotation != held_rotation

                for msg in samples:
                    assert msg.name == list(JOINT_NAMES + SPREAD_NAMES)
                    assert len(msg.position) == 20 and msg.header.stamp.sec > 0
                print(f'PASS: {len(samples)} joint samples, {len(frames)} TF frames, framing, '
                      'invalid input, stale/disconnect holds and reconnect'
                      + ('; wrist TF motion, invalid hold and reacquisition' if wrist else ''))
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
