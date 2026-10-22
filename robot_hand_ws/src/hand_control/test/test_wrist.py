"""Camera-free wrist geometry, wire compatibility, and receiver hold tests."""

import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'robot_webcam'))
from wrist_orientation import WristTracker, encode_sample, palm_frame
from rotation_math import distance, from_matrix, smooth
from test_hand_tracking import synthetic_hand
from hand_control.wrist_receiver import WristReceiver, parse_frame
from hand_control.model import JOINT_NAMES

JOINTS = [(name, 0., 1.4, 1.) for name in JOINT_NAMES]


def test_geometry_neutral_and_handedness():
    points = synthetic_hand()
    frame, normal = palm_frame(points, 'right')
    assert np.linalg.det(frame) == pytest.approx(1)
    np.testing.assert_allclose(frame.T @ frame, np.eye(3), atol=1e-12)
    mirrored = points * (-1, 1, 1)
    left, _ = palm_frame(mirrored, 'left')
    assert np.linalg.det(left) == pytest.approx(1)
    tracker = WristTracker()
    for i in range(3):
        q, _, _ = tracker.update(points, 'right', i * .05)
    assert q is not None
    assert tracker.capture_neutral(.11)
    for i in range(3):
        q, _, _ = tracker.update(points, 'right', .15 + i * .05)
    assert distance(q, (0, 0, 0, 1)) < 1e-7
    tracker.lost()
    assert not tracker.capture_neutral(.3)
    assert tracker.update(np.zeros((21, 3)), 'right', .4)[0] is None
    tracker.update(mirrored, 'left', .5)
    assert tracker.neutral is None


def test_rotations_and_speed():
    identity = (0, 0, 0, 1)
    for matrix in (np.eye(3), np.diag([1., -1., -1.]),
                   np.diag([-1., 1., -1.]), np.diag([-1., -1., 1.])):
        q = from_matrix(matrix)
        assert sum(v*v for v in q) == pytest.approx(1)
        assert distance(q, tuple(-v for v in q)) < 1e-7
        assert distance(identity, smooth(identity, q, .05)) <= .15 + 1e-7


def test_wire_roundtrip_and_invalid_fields():
    wire = encode_sample([45.] * 14, (0, 0, 0, 1), 1, 'right')
    fingers, q, seq, side = parse_frame(wire, JOINTS)
    assert fingers == pytest.approx([math.pi/4]*14)
    assert (q, seq, side) == ((0., 0., 0., 1.), 1, 'right')
    assert parse_frame(b','.join([b'0']*14), JOINTS)[1:] == (None, None, None)
    assert parse_frame(encode_sample([0]*14, None, 2, 'left'), JOINTS)[1] is None
    for field, value in [('version', True), ('sequence', -1), ('handedness', 'any'),
                         ('wrist_quaternion_xyzw', [0]*4), ('finger_angles_deg', [0]*13),
                         ('orientation_valid', 'yes'), ('frame_id', 'camera')]:
        data = json.loads(wire)
        data[field] = value
        with pytest.raises(ValueError):
            parse_frame(json.dumps(data).encode(), JOINTS)


def test_receiver_reacquisition_duplicates_and_hold():
    # Exercise actual receiver state methods without opening ROS entities or sockets.
    receiver = WristReceiver.__new__(WristReceiver)
    from hand_control.rotation_math import OrientationGate
    receiver.rotation = (0., 0., 0., 1.)
    receiver.target_rotation = receiver.rotation
    receiver.gate = OrientationGate()
    receiver.reset_extra()
    receiver.connected = True
    receiver.wrist_tau, receiver.wrist_speed, receiver.wrist_timeout = .15, 3., .5
    receiver.wrist_holding = True
    target = (0., 0., math.sin(.4), math.cos(.4))
    for seq in range(3):
        receiver.apply_sample(([.5]*14, target, seq, 'right'))
    receiver.update_extra(time.monotonic(), .05)
    assert distance(receiver.rotation, (0, 0, 0, 1)) > 0
    freshness = receiver.last_sample
    receiver.apply_sample(([1.]*14, target, 2, 'right'))
    assert receiver.last_sample == freshness
    assert receiver.targets == [.5]*14
    receiver.apply_sample(([1.]*14, None, 3, 'right'))
    receiver.wrist_holding = True
    held = receiver.rotation
    receiver.update_extra(time.monotonic(), .05)
    assert receiver.rotation == held
    assert receiver.targets == [1.]*14


def test_math_copies_match():
    root = Path(__file__).resolve().parents[4]
    assert (root/'robot_webcam/rotation_math.py').read_bytes() == (
        root/'robot_hand_ws/src/hand_control/hand_control/rotation_math.py').read_bytes()


def test_neutral_capture_reaches_receiver_after_large_change():
    """A deliberate neutral reset must recover through the receiver's jump gate."""
    from hand_control.rotation_math import OrientationGate
    receiver = WristReceiver.__new__(WristReceiver)
    receiver.gate = OrientationGate()
    receiver.rotation = receiver.target_rotation = (0., 0., 0., 1.)
    receiver.reset_extra()
    tracker = WristTracker()
    points = synthetic_hand()
    sequence = 0

    def transfer(now):
        nonlocal sequence
        q, _, _ = tracker.update(points, 'right', now)
        sequence += 1
        receiver.apply_sample(parse_frame(encode_sample([0]*14, q, sequence, 'right'), JOINTS))

    for i in range(8):
        transfer(i*.05)
    assert distance(receiver.target_rotation, (0, 0, 0, 1)) > 1.2
    assert tracker.capture_neutral(.36)
    for i in range(10):
        transfer(.4+i*.05)
    assert receiver.last_orientation is not None
    assert distance(receiver.target_rotation, (0, 0, 0, 1)) < 1e-7


def test_spread_tracks_signed_motion_and_rigid_rotations():
    from wrist_orientation import estimate_spread
    points = synthetic_hand()
    frame, _ = palm_frame(points, 'right')
    expected = [20., 5., -8., -22.]
    for mcp, degrees in zip((5, 9, 13, 17), expected):
        a = math.radians(degrees)
        points[mcp+1] = points[mcp] + .03*(frame @ np.array([math.sin(a), 0, math.cos(a)]))
    assert estimate_spread(points, 'right') == pytest.approx(expected)
    rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    assert estimate_spread(points @ rotation.T + [2, 3, 4], 'right') == pytest.approx(expected)
    assert estimate_spread(points * [-1, 1, 1], 'left') == pytest.approx(expected)
    for mcp in (5, 9, 13, 17):
        points[mcp+1] = points[mcp] - .03*frame[:, 1]
    assert estimate_spread(points, 'right') is None


def test_spread_protocol_validation_and_clamping():
    receiver = WristReceiver.__new__(WristReceiver)
    from hand_control.rotation_math import OrientationGate
    receiver.gate = OrientationGate()
    receiver.reset_extra()
    receiver.joints = JOINTS
    receiver.spread_joints = [(str(i), -.45, .45, 1.) for i in range(4)]
    receiver.spread_targets = [0.]*4
    wire = encode_sample([30]*14, None, 1, 'right', [90, 10, -10, -90])
    receiver.apply_sample(receiver.parse_sample(wire))
    assert receiver.spread_targets == pytest.approx([.45, math.radians(10), -math.radians(10), -.45])
    for invalid in ([1, 2], [True]*4, [float('nan')]*4):
        data = json.loads(wire)
        data['finger_spread_deg'] = invalid
        with pytest.raises(ValueError):
            receiver.parse_sample(json.dumps(data).encode())


def test_thumb_sideways_geometry_and_wire():
    from wrist_orientation import estimate_thumb_spread
    points = synthetic_hand()
    frame, _ = palm_frame(points, 'right')
    for angle in (-.5, 0., .5):
        opening = .9 + angle
        points[2] = points[1] + .03*(frame @ np.array([math.sin(opening), 0, math.cos(opening)]))
        expected = math.degrees(angle)
        assert estimate_thumb_spread(points, 'right') == pytest.approx(expected, abs=1e-9)
        assert estimate_thumb_spread(points*[-1, 1, 1], 'left') == pytest.approx(expected, abs=1e-9)
        rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        assert estimate_thumb_spread(points @ rotation.T, 'right') == pytest.approx(expected, abs=1e-9)
    receiver = WristReceiver.__new__(WristReceiver)
    from hand_control.rotation_math import OrientationGate
    receiver.gate = OrientationGate()
    receiver.reset_extra()
    receiver.joints = JOINTS
    receiver.spread_targets = [0.]*6
    receiver.spread_joints = [('spread', -.8, .8, 1.)]*6
    receiver.apply_sample(receiver.parse_sample(encode_sample([0]*14, None, 1, 'right', thumb_spread=90)))
    assert receiver.spread_targets[4] == .8
    receiver.apply_sample(receiver.parse_sample(encode_sample([0]*14, None, 2, 'right')))
    assert receiver.spread_targets[4] == .8  # old sender holds sideways thumb
    receiver.apply_sample(receiver.parse_sample(b','.join([b'0']*14)))
    assert receiver.spread_targets == [0.]*6
    for value in (True, '30', float('nan')):
        wire = json.loads(encode_sample([0]*14, None, 3, 'right'))
        wire['thumb_spread_deg'] = value
        with pytest.raises(ValueError):
            receiver.parse_sample(json.dumps(wire).encode())


def test_thumb_opposition_separates_from_sideways_motion():
    from wrist_orientation import estimate_thumb_opposition, estimate_thumb_spread
    points = synthetic_hand()
    frame, _ = palm_frame(points, 'right')
    for opening in (.5, .9, 1.3):
        for inward in (0., .3, .8):
            direction = np.array([math.cos(inward)*math.sin(opening),
                                  -math.sin(inward)*math.sin(opening), math.cos(opening)])
            points[2] = points[1] + .03*(frame @ direction)
            assert estimate_thumb_opposition(points, 'right') == pytest.approx(math.degrees(inward), abs=1e-8)
            assert estimate_thumb_spread(points, 'right') == pytest.approx(math.degrees(opening-.9), abs=1e-8)
            mirrored = points*[-1, 1, 1]
            assert estimate_thumb_opposition(mirrored, 'left') == pytest.approx(math.degrees(inward), abs=1e-8)
            rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
            assert estimate_thumb_opposition(points @ rotation.T + [2, 3, 4], 'right') == pytest.approx(math.degrees(inward), abs=1e-8)
    receiver = WristReceiver.__new__(WristReceiver)
    from hand_control.rotation_math import OrientationGate
    receiver.gate = OrientationGate()
    receiver.reset_extra()
    receiver.joints = JOINTS
    receiver.spread_targets = [0.]*6
    receiver.spread_joints = [('extra', 0., 1.2, 1.)]*6
    receiver.apply_sample(receiver.parse_sample(encode_sample([0]*14, None, 1, 'right', thumb_opposition=120)))
    assert receiver.spread_targets[5] == 1.2
    receiver.apply_sample(receiver.parse_sample(encode_sample([0]*14, None, 2, 'right')))
    assert receiver.spread_targets[5] == 1.2
    receiver.apply_sample(receiver.parse_sample(b','.join([b'0']*14)))
    assert receiver.spread_targets == [0.]*6
    for value in (True, '30', float('nan')):
        wire = json.loads(encode_sample([0]*14, None, 3, 'right'))
        wire['thumb_opposition_deg'] = value
        with pytest.raises(ValueError):
            receiver.parse_sample(json.dumps(wire).encode())


def test_thenar_joint_carries_pad_and_thumb_with_preserved_rest_pose():
    import xml.etree.ElementTree as ET
    from ament_index_python.packages import get_package_share_path
    import xacro
    model = get_package_share_path('robot_hand_description') / 'urdf/hand.urdf.xacro'
    root = ET.fromstring(xacro.process_file(str(model)).toxml())
    joint = root.find("joint[@name='thumb_opposition']")
    assert joint.find('parent').get('link') == 'palm'
    assert joint.find('child').get('link') == 'thenar'
    assert root.find("joint[@name='thumb_spread']/parent").get('link') == 'thenar'
    assert root.find("link[@name='thenar']/visual/geometry/mesh") is not None
    pivot = np.fromstring(joint.find('origin').get('xyz'), sep=' ')
    pad_offset = np.fromstring(root.find("link[@name='thenar']/visual/origin").get('xyz'), sep=' ')
    thumb_offset = np.fromstring(root.find("joint[@name='thumb_spread']/origin").get('xyz'), sep=' ')
    np.testing.assert_allclose(pivot+pad_offset, [0, 0, 0], atol=1e-9)
    np.testing.assert_allclose(pivot+thumb_offset, [.052, -.006, .007], atol=1e-9)
    assert joint.find('axis').get('xyz') == '0 0 -1'
    angle = .8
    rotation = np.array([[math.cos(angle), math.sin(angle), 0],
                         [-math.sin(angle), math.cos(angle), 0], [0, 0, 1]])
    moved = pivot + rotation @ thumb_offset
    assert moved[1] < -.02  # inward, toward the palmar side
    assert moved[0] < .052  # also toward the main palm center
