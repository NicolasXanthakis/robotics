"""Signed palm orientation and version-2 wire encoding; no camera dependency."""

import json

import numpy as np

from rotation_math import from_matrix, inverse, multiply, normalize, OrientationGate

# Optical-like model coordinates (right, down, away) -> ROS base (forward, left, up).
# Proper rotation, determinant +1. This is a convention, not camera extrinsic calibration.
BASE_FROM_OPTICAL = np.array(((0., 0., 1.), (-1., 0., 0.), (0., -1., 0.)))
FRAME = 'hand_base'
CHILD = 'palm'


def unit(v):
    norm = np.linalg.norm(v)
    if not np.isfinite(norm) or norm < 1e-8:
        raise ValueError('Degenerate palm geometry')
    return v/norm


def palm_frame(landmarks, side):
    """Return optical-from-model rotation and signed palmar normal.

    Model +Z points along fingers; -Y is palmar. Left hands are retargeted
    to the same right-hand model with a proper (not reflected) basis.
    For unmirrored input, right-hand across vector is index minus pinky.
    """
    if side not in ('right', 'left'):
        raise ValueError('Handedness required')
    p = np.asarray([(v.x, v.y, v.z) if hasattr(v, 'x') else v for v in landmarks])
    if p.shape != (21, 3) or not np.isfinite(p).all():
        raise ValueError('Expected 21 finite world landmarks')
    z = unit((p[5]+p[17])/2-p[0])
    across = p[5]-p[17]
    x = across-np.dot(across, z)*z
    if np.linalg.norm(x)/max(np.linalg.norm(across), 1e-8) < 0.15:
        raise ValueError('Palm landmarks nearly collinear')
    x = unit(x) * (1 if side == 'right' else -1)
    y = unit(np.cross(z, x))
    x = unit(np.cross(y, z))
    matrix = np.column_stack((x, y, z))
    return matrix, -y


class FacingLabel:
    """Facing is signed; edge-on band avoids palm/back chatter."""

    def __init__(self):
        self.label = 'edge-on'

    def update(self, normal):
        alignment = float(np.dot(normal, (0., 0., -1.)))
        if alignment > 0.25:
            self.label = 'palm-facing'
        elif alignment < -0.25:
            self.label = 'back-facing'
        elif abs(alignment) < 0.15:
            self.label = 'edge-on'
        return self.label


class WristTracker:
    """Keep neutral mapping independent from optional finger calibration."""

    def __init__(self):
        self.side = None
        self.neutral = None
        self.latest_absolute = None
        self.latest_time = None
        self.gate = OrientationGate()
        self.facing = FacingLabel()
        self.status = 'wrist acquiring'

    def lost(self):
        self.gate.reset()
        self.latest_absolute = self.latest_time = None
        self.status = 'wrist holding (tracking lost)'

    def clear_neutral(self):
        self.neutral = None
        self.gate.reset()

    def capture_neutral(self, now):
        if (self.latest_absolute is None or self.latest_time is None
                or now-self.latest_time > 0.25 or self.gate.count < 3):
            return False
        self.neutral = self.latest_absolute
        self.status = 'wrist neutral-relative'
        self.gate.reset()
        return True

    def update(self, landmarks, side, now):
        if side != self.side:
            self.side = side
            self.neutral = None
            self.gate.reset()
            self.facing = FacingLabel()
        try:
            frame, normal = palm_frame(landmarks, side)
            absolute = from_matrix(BASE_FROM_OPTICAL @ frame)
            accepted = self.gate.update(absolute, now)
            if accepted is None:
                self.latest_absolute = self.latest_time = None
                self.status = 'wrist acquiring / jump rejected'
                return None, frame, self.facing.update(normal)
            self.latest_absolute, self.latest_time = accepted, now
            # Spatial rotation from neutral to current, expressed in base axes.
            q = multiply(accepted, inverse(self.neutral)) if self.neutral else accepted
            self.status = 'wrist neutral-relative' if self.neutral else 'wrist camera-relative'
            return q, frame, self.facing.update(normal)
        except ValueError:
            self.lost()
            return None, None, 'uncertain'


def estimate_spread(landmarks, side):
    """Signed MCP spread in the palm plane; hold when curled/degenerate."""
    frame, _ = palm_frame(landmarks, side)
    p = np.asarray([(v.x, v.y, v.z) if hasattr(v, 'x') else v for v in landmarks])
    angles = []
    for mcp in (5, 9, 13, 17):
        direction = frame.T @ unit(p[mcp+1]-p[mcp])
        if direction[2] < 0.25:
            return None
        angles.append(float(np.degrees(np.arctan2(direction[0], direction[2])))
                      * (1 if side == 'right' else -1))
    return angles


def estimate_thumb_spread(landmarks, side):
    """Map the thumb metacarpal's palm-plane direction to its sideways joint."""
    frame, _ = palm_frame(landmarks, side)
    p = np.asarray([(v.x, v.y, v.z) if hasattr(v, 'x') else v for v in landmarks])
    direction = frame.T @ unit(p[2]-p[1])
    if np.hypot(direction[0], direction[2]) < 0.25:
        return None
    across = direction[0] * (1 if side == 'right' else -1)
    return float(np.degrees(np.arctan2(np.hypot(across, direction[1]), direction[2]) - 0.9))


def estimate_thumb_opposition(landmarks, side):
    """Estimate inward CMC rotation from metacarpal elevation off the palm plane."""
    frame, _ = palm_frame(landmarks, side)
    p = np.asarray([(v.x, v.y, v.z) if hasattr(v, 'x') else v for v in landmarks])
    direction = frame.T @ unit(p[2]-p[1])
    across = direction[0] * (1 if side == 'right' else -1)
    if np.hypot(across, direction[1]) < .2:
        return None
    return max(0., float(np.degrees(np.arctan2(-direction[1], across))))


def encode_sample(angles, quaternion, sequence, side, spread=None, thumb_spread=None,
                  thumb_opposition=None):
    if len(angles) != 14 or not np.isfinite(angles).all():
        raise ValueError('Expected 14 finite finger degrees')
    q = list(normalize(quaternion)) if quaternion is not None else None
    if spread is not None and (len(spread) != 4 or not np.isfinite(spread).all()):
        raise ValueError('Expected four finite spread degrees')
    if thumb_spread is not None and not np.isfinite(thumb_spread):
        raise ValueError('Expected finite thumb spread degrees')
    if thumb_opposition is not None and not np.isfinite(thumb_opposition):
        raise ValueError('Expected finite thumb opposition degrees')
    return (json.dumps({
        'version': 2, 'sequence': sequence, 'frame_id': FRAME, 'child_frame_id': CHILD,
        'finger_angles_deg': list(angles), 'wrist_quaternion_xyzw': q,
        'orientation_valid': q is not None, 'handedness': side,
        'finger_spread_deg': spread, 'thumb_spread_deg': thumb_spread,
        'thumb_opposition_deg': thumb_opposition,
    }, allow_nan=False, separators=(',', ':'))+'\n').encode('ascii')
