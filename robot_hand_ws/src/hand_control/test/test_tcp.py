"""Validate TCP framing, all-or-nothing parsing, and speed limiting."""

import math

from hand_control.model import JOINT_NAMES
from hand_control.webcam_receiver import advance, LineBuffer, parse_angles
import pytest

JOINTS = [(name, 0.0, 1.4, 1.0) for name in JOINT_NAMES]


def test_fragmented_coalesced_and_oversized_lines():
    """Recover valid frames after fragmentation, CRLF, and oversized input."""
    buffer = LineBuffer(limit=8)
    assert list(buffer.feed(b'1,2')) == []
    assert list(buffer.feed(b',3\r')) == []
    assert list(buffer.feed(b'\n4,5\n')) == [b'1,2,3\r', b'4,5']
    assert list(buffer.feed(b'x' * 50)) == []
    assert len(buffer.buffer) <= 8
    assert list(buffer.feed(b'bad\ngood\n')) == [b'good']


@pytest.mark.parametrize('line', [b'1,2', b','.join([b'0'] * 15),
                                  b','.join([b'nan'] * 14),
                                  b','.join([b'inf'] * 14), b'\xff', b''])
def test_reject_invalid_samples(line):
    """Reject invalid numbers, encoding, and field counts."""
    with pytest.raises((ValueError, UnicodeError)):
        parse_angles(line, JOINTS)


def test_conversion_clamping_and_smoothing():
    """Degrees become radians and motion obeys velocity and position limits."""
    assert parse_angles(b','.join([b'45'] * 14), JOINTS) == pytest.approx(
        [math.pi / 4] * 14)
    assert parse_angles(b','.join([b'-90'] * 14), JOINTS) == [0.0] * 14
    targets = parse_angles(b','.join([b'180'] * 14), JOINTS)
    assert targets == [1.4] * 14
    positions = [0.0] * 14
    for _ in range(100):
        previous = positions
        positions = advance(previous, targets, JOINTS, 0.2, 0.05)
        assert all(0 <= p <= 1.4 for p in positions)
        assert all(abs(p - old) <= 0.05 + 1e-12
                   for p, old in zip(positions, previous))
    assert positions == pytest.approx(targets, abs=1e-5)
