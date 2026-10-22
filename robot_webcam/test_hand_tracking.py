"""Synthetic geometry and real loopback protocol tests; no webcam/model required."""

import math
import socket
import threading
import time
import unittest
from types import SimpleNamespace

import numpy as np

from hand_tracking import (JOINT_NAMES, LIMITS, TcpSender, bend, encode_angles,
                           estimate_flexion, map_angles, select_hand,
                           validate_calibration)


def synthetic_hand(mcp_angle=0, pip_angle=0, dip_angle=0, thumb_base=0, thumb_ip=0):
    points = np.zeros((21, 3))
    roots = ((0.03, 0.06, 0), (0.01, 0.07, 0), (-0.01, 0.065, 0), (-0.03, 0.05, 0))
    normal = np.cross(roots[0], roots[-1])
    normal /= np.linalg.norm(normal)
    for start, root in zip((5, 9, 13, 17), roots):
        points[start] = root
        reference = np.array(root) / np.linalg.norm(root)
        angle = 0
        for offset, flexion in enumerate((mcp_angle, pip_angle, dip_angle), 1):
            angle += math.radians(flexion)
            direction = math.cos(angle) * reference + math.sin(angle) * normal
            points[start+offset] = points[start+offset-1] + direction * 0.02
    points[1] = (0.04, 0.01, 0)
    points[2] = points[1] + (0.02, 0, 0)
    a, b = math.radians(thumb_base), math.radians(thumb_base+thumb_ip)
    points[3] = points[2] + np.array((math.cos(a), math.sin(a), 0)) * 0.02
    points[4] = points[3] + np.array((math.cos(b), math.sin(b), 0)) * 0.02
    return points


class GeometryTests(unittest.TestCase):
    def test_open_hand(self):
        np.testing.assert_allclose(estimate_flexion(synthetic_hand()), [0]*14, atol=1e-5)

    def test_known_flexion_and_thumb(self):
        angles = estimate_flexion(synthetic_hand(35, 60, 45, 30, 50))
        np.testing.assert_allclose(angles, [35, 60, 45]*4 + [30, 50], atol=1e-5)

    def test_rotation_scale_translation_and_mirror(self):
        points = synthetic_hand(40, 55, 30, 20, 35)
        angle = 0.8
        rotation = np.array(((1, 0, 0), (0, math.cos(angle), -math.sin(angle)),
                             (0, math.sin(angle), math.cos(angle))))
        expected = estimate_flexion(points)
        transformed = points @ rotation.T * 2 + (1, 2, 3)
        np.testing.assert_allclose(estimate_flexion(transformed), expected, atol=1e-5)
        points[:, 0] *= -1
        np.testing.assert_allclose(estimate_flexion(points), expected, atol=1e-5)

    def test_spread_does_not_curl_mcp(self):
        points = synthetic_hand()
        for start in (5, 9, 13, 17):
            for offset in (1, 2, 3):
                points[start+offset] = points[start] + np.array((0.01, 0.02, 0))*offset
        np.testing.assert_allclose(estimate_flexion(points)[:12], [0]*12, atol=1e-5)

    def test_invalid_geometry(self):
        for points in (np.zeros((21, 3)), np.zeros((20, 3)), np.full((21, 3), np.nan)):
            with self.assertRaises(ValueError):
                estimate_flexion(points)
        with self.assertRaises(ValueError):
            bend(np.zeros(3), np.zeros(3), np.ones(3))

    def test_defaults_and_calibration(self):
        self.assertEqual(map_angles([-10]*14), [0]*14)
        np.testing.assert_allclose(map_angles([500]*14), LIMITS)
        self.assertEqual(map_angles([30]*14), [30]*14)
        calibration = {'joint_names': list(JOINT_NAMES), 'open': [10]*14, 'closed': [70]*14}
        np.testing.assert_allclose(map_angles([40]*14, calibration), np.array(LIMITS)/2)
        calibration['closed'][0] = 11  # weak range retains defaults
        self.assertEqual(map_angles([40]*14, calibration)[0], 40)
        calibration['open'][0] = float('nan')
        with self.assertRaises(ValueError):
            validate_calibration(calibration)

    def test_protocol_fields_units_and_rejection(self):
        wire = encode_angles(map_angles([30]*14))
        self.assertTrue(wire.endswith(b'\n'))
        self.assertEqual(len(wire.splitlines()), 1)
        self.assertEqual([float(v) for v in wire.split(b',')], [30]*14)
        self.assertEqual(JOINT_NAMES[-2:], ('thumb_base', 'thumb_ip'))
        for values in ([0]*13, [float('inf')]*14, [float('nan')]*14):
            with self.assertRaises(ValueError):
                encode_angles(values)

    def test_hand_selection(self):
        result = SimpleNamespace(
            handedness=[[SimpleNamespace(category_name='Left', score=0.9)],
                        [SimpleNamespace(category_name='Right', score=0.8)]],
            hand_world_landmarks=[[], []])
        self.assertEqual(select_hand(result, 'right')[1], 1)
        result.handedness.pop()
        self.assertIsNone(select_hand(result, 'right'))


class NetworkTests(unittest.TestCase):
    def test_send_loss_and_reconnect(self):
        server = socket.socket()
        server.bind(('127.0.0.1', 0))
        server.listen(1)
        server.settimeout(3)
        sender = TcpSender('127.0.0.1', server.getsockname()[1])
        connection = None
        try:
            connection, _ = server.accept()
            connection.settimeout(2)
            sender.update([30]*14)
            data = b''
            while b'\n' not in data:
                data += connection.recv(4096)
            self.assertEqual([float(v) for v in data.split(b'\n')[0].split(b',')], [30]*14)
            sender.update(None)
            time.sleep(0.35)
            connection.settimeout(0.05)
            try:
                while connection.recv(4096):
                    pass
            except socket.timeout:
                pass
            with self.assertRaises(socket.timeout):
                connection.recv(4096)
            connection.close()
            connection = None
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                sender.update([45]*14)
                time.sleep(0.05)
                if 'disconnected' in sender.status:
                    break
            connection, _ = server.accept()
            connection.settimeout(2)
            sender.update([45]*14)
            data = b''
            while b'\n' not in data:
                data += connection.recv(4096)
            self.assertEqual(len(data.split(b'\n')[0].split(b',')), 14)
        finally:
            sender.close()
            if connection is not None:
                connection.close()
            server.close()
        self.assertFalse(sender.worker.is_alive())


if __name__ == '__main__':
    unittest.main()
