"""Quaternion wrist TF plus finger curl and spread; accepts v2 JSON and legacy CSV."""

import json
import math
import time

from geometry_msgs.msg import TransformStamped
from hand_control.rotation_math import normalize, OrientationGate, smooth
from hand_control.webcam_receiver import WebcamReceiver, parse_angles
import rclpy
from rclpy.executors import ExternalShutdownException
from tf2_ros import TransformBroadcaster


def parse_frame(line, joints):
    """Return fingers, optional orientation and sequence; reject malformed frames atomically."""
    if not line.lstrip().startswith(b'{'):
        return parse_angles(line, joints), None, None, None
    data = json.loads(line.decode('ascii'))
    if (data.get('version') != 2 or type(data.get('version')) is not int
            or data.get('frame_id') != 'hand_base' or data.get('child_frame_id') != 'palm'
            or data.get('handedness') not in ('right', 'left')
            or type(data.get('sequence')) is not int or data['sequence'] < 0
            or type(data.get('orientation_valid')) is not bool):
        raise ValueError('Invalid v2 frame header')
    values = data.get('finger_angles_deg')
    if (not isinstance(values, list) or len(values) != 14
            or not all(type(v) in (int, float) and math.isfinite(v) for v in values)):
        raise ValueError('Invalid finger sample')
    fingers = parse_angles(','.join(str(v) for v in values).encode('ascii'), joints)
    spread = data.get('finger_spread_deg')
    if spread is not None and (not isinstance(spread, list) or len(spread) != 4
            or not all(type(v) in (int, float) and math.isfinite(v) for v in spread)):
        raise ValueError('Expected four finite spread angles')
    thumb = data.get('thumb_spread_deg')
    if thumb is not None and (type(thumb) not in (int, float) or not math.isfinite(thumb)):
        raise ValueError('Expected a finite thumb spread angle')
    opposition = data.get('thumb_opposition_deg')
    if opposition is not None and (type(opposition) not in (int, float)
                                   or not math.isfinite(opposition)):
        raise ValueError('Expected a finite thumb opposition angle')
    q = data.get('wrist_quaternion_xyzw')
    if data['orientation_valid']:
        if not isinstance(q, list):
            raise ValueError('Missing quaternion')
        q = normalize(q)
    elif q is not None:
        raise ValueError('Invalid orientation must use null quaternion')
    return fingers, q, data['sequence'], data['handedness']


class WristReceiver(WebcamReceiver):
    """Sole owner of dynamic hand_base -> palm; URDF root remains palm."""

    def configure_extra(self):
        for name, value in [('wrist_tau', 0.15), ('wrist_max_speed', 3.0),
                            ('wrist_timeout', 0.5)]:
            self.declare_parameter(name, value)
        self.wrist_tau = float(self.get_parameter('wrist_tau').value)
        self.wrist_speed = float(self.get_parameter('wrist_max_speed').value)
        self.wrist_timeout = float(self.get_parameter('wrist_timeout').value)
        if not all(math.isfinite(v) and v > 0 for v in (
                self.wrist_tau, self.wrist_speed, self.wrist_timeout)):
            raise ValueError('Wrist tuning parameters must be positive and finite')
        self.rotation = (0.0, 0.0, 0.0, 1.0)
        self.target_rotation = self.rotation
        self.gate = OrientationGate(timeout=self.wrist_timeout)
        self.last_orientation = None
        self.sequence = -1
        self.side = None
        self.wrist_holding = True
        self.broadcaster = TransformBroadcaster(self)

    def reset_extra(self):
        self.last_orientation = None
        self.sequence = -1
        self.side = None
        self.gate.reset()

    def parse_sample(self, line):
        sample = parse_frame(line, self.joints)
        spread = json.loads(line).get('finger_spread_deg') if line.lstrip().startswith(b'{') else [0.0]*4
        thumb = json.loads(line).get('thumb_spread_deg') if line.lstrip().startswith(b'{') else 0.0
        opposition = json.loads(line).get('thumb_opposition_deg') if line.lstrip().startswith(b'{') else 0.0
        return sample + (spread, thumb, opposition)

    def apply_sample(self, sample):
        fingers, quaternion, sequence, side = sample[:4]
        spread = sample[4] if len(sample) > 4 else None
        if sequence is not None:
            if sequence <= self.sequence:
                return  # repeated sender frames do not refresh input freshness
            self.sequence = sequence
        self.targets = fingers
        if spread is not None:
            self.spread_targets[:4] = [max(j[1], min(j[2], math.radians(v)))
                                       for v, j in zip(spread, self.spread_joints[:4])]
        thumb = sample[5] if len(sample) > 5 else None
        if thumb is not None:
            joint = self.spread_joints[4]
            self.spread_targets[4] = max(joint[1], min(joint[2], math.radians(thumb)))
        opposition = sample[6] if len(sample) > 6 else None
        if opposition is not None:
            joint = self.spread_joints[5]
            self.spread_targets[5] = max(joint[1], min(joint[2], math.radians(opposition)))
        now = time.monotonic()
        self.last_sample = now
        if side != self.side:
            self.gate.reset()
            self.last_orientation = None
            self.side = side
        if quaternion is None:
            self.last_orientation = None
            self.gate.reset()
            return
        accepted = self.gate.update(quaternion, now)
        if accepted is None:
            self.last_orientation = None
            return
        self.target_rotation = accepted
        self.last_orientation = now

    def update_extra(self, now, dt):
        active = (self.connected and self.last_orientation is not None
                  and now-self.last_orientation <= self.wrist_timeout)
        if active:
            self.rotation = smooth(self.rotation, self.target_rotation, dt,
                                   self.wrist_tau, self.wrist_speed)
        if not active and not self.wrist_holding:
            self.get_logger().warning('Wrist input invalid/stale; holding orientation')
        self.wrist_holding = not active

    def publish_extra(self, stamp):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'hand_base'
        transform.child_frame_id = 'palm'
        # Anatomical wrist pivot is 55 mm below the palm's existing center.
        # T_base_palm = R * translate(0,0,0.055); no wrist position tracking.
        x, y, z, w = self.rotation
        transform.transform.translation.x = 0.055 * 2*(x*z+w*y)
        transform.transform.translation.y = 0.055 * 2*(y*z-w*x)
        transform.transform.translation.z = 0.055 * (1-2*(x*x+y*y))
        q = transform.transform.rotation
        q.x, q.y, q.z, q.w = self.rotation
        self.broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WristReceiver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
