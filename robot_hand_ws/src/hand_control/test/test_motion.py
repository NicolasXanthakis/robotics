"""Check motion against the installed model rather than duplicated limits."""

import math

from ament_index_python.packages import get_package_share_path
from hand_control.test_joint_publisher import load_joints, sample_positions
import pytest
import xacro


def test_full_range_and_speed():
    """Every URDF joint reaches both bounds and stays within position/speed limits."""
    model = get_package_share_path('robot_hand_description') / 'urdf/hand.urdf.xacro'
    joints = load_joints(xacro.process_file(str(model)).toxml())
    expected = {
        f'{finger}_{joint}'
        for finger in ('index', 'middle', 'ring', 'pinky')
        for joint in ('mcp', 'pip', 'dip')
    } | {'thumb_base', 'thumb_ip', 'thumb_spread', 'thumb_opposition'} | {f'{f}_spread' for f in ('index', 'middle', 'ring', 'pinky')}
    assert {joint[0] for joint in joints} == expected
    assert len(joints) == 20
    assert sample_positions(joints, 0.0, 8.0) == pytest.approx([j[1] for j in joints])
    assert sample_positions(joints, 4.0, 8.0) == pytest.approx([j[2] for j in joints])
    assert sample_positions(joints, 8.0, 8.0) == pytest.approx([j[1] for j in joints])
    for step in range(161):
        positions = sample_positions(joints, step * 0.05, 8.0)
        for position, (_, lower, upper, velocity) in zip(positions, joints):
            assert math.isfinite(position)
            assert lower <= position <= upper
            assert math.pi * (upper - lower) / 8.0 <= velocity
