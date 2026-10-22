"""Dependency-free quaternion math; xyzw, active child-to-parent rotations.

Kept identical to hand_control/rotation_math.py for independent Windows/ROS use.
"""

import math


def normalize(q):
    if len(q) != 4 or not all(type(v) in (int, float) and math.isfinite(v) for v in q):
        raise ValueError('Expected four finite quaternion components')
    norm = math.hypot(*q)
    if norm < 1e-8:
        raise ValueError('Zero quaternion')
    return tuple(v/norm for v in q)


def multiply(a, b):
    x, y, z, w = a
    u, v, s, t = b
    return normalize((w*u+x*t+y*s-z*v, w*v-x*s+y*t+z*u,
                      w*s+x*v-y*u+z*t, w*t-x*u-y*v-z*s))


def inverse(q):
    x, y, z, w = normalize(q)
    return (-x, -y, -z, w)


def distance(a, b):
    dot = abs(sum(x*y for x, y in zip(normalize(a), normalize(b))))
    return 2*math.acos(min(1.0, dot))


def slerp(a, b, fraction):
    a, b = normalize(a), normalize(b)
    dot = sum(x*y for x, y in zip(a, b))
    if dot < 0:
        b = tuple(-v for v in b)
        dot = -dot
    dot = min(1.0, dot)
    fraction = max(0.0, min(1.0, fraction))
    if dot > 0.9995:
        return normalize(tuple(x+fraction*(y-x) for x, y in zip(a, b)))
    theta = math.acos(dot)
    s = math.sin(theta)
    return normalize(tuple((math.sin((1-fraction)*theta)*x +
                            math.sin(fraction*theta)*y)/s for x, y in zip(a, b)))


def smooth(a, b, dt, tau=0.15, max_speed=3.0):
    angle = distance(a, b)
    fraction = 1-math.exp(-max(0.0, dt)/tau)
    if angle > 1e-8:
        fraction = min(fraction, max_speed*max(0.0, dt)/angle)
    return slerp(a, b, fraction)


def from_matrix(m):
    """Convert a proper orthonormal 3x3 matrix, including rotations near pi."""
    trace = m[0][0]+m[1][1]+m[2][2]
    if trace > 0:
        s = math.sqrt(trace+1)*2
        q = ((m[2][1]-m[1][2])/s, (m[0][2]-m[2][0])/s,
             (m[1][0]-m[0][1])/s, s/4)
    else:
        i = max(range(3), key=lambda j: m[j][j])
        j, k = (i+1) % 3, (i+2) % 3
        s = math.sqrt(max(0, 1+m[i][i]-m[j][j]-m[k][k]))*2
        components = [0.0]*4
        components[i] = s/4
        components[j] = (m[j][i]+m[i][j])/s
        components[k] = (m[k][i]+m[i][k])/s
        components[3] = (m[k][j]-m[j][k])/s
        q = components
    return normalize(tuple(float(v) for v in q))


class OrientationGate:
    """Require stable reacquisition; reject abrupt flips while fingers remain live."""

    def __init__(self, stable_samples=3, jump_limit=1.2, timeout=0.5):
        self.stable_samples = stable_samples
        self.jump_limit = jump_limit
        self.timeout = timeout
        self.reset()

    def reset(self):
        self.previous = None
        self.last_time = None
        self.count = 0

    def update(self, q, now):
        q = normalize(q)
        if self.last_time is None or now-self.last_time > self.timeout:
            self.previous = None
            self.count = 0
        if self.previous is not None and distance(self.previous, q) > self.jump_limit:
            self.count = 0
            self.previous = None
            self.last_time = now
            return None
        if self.previous is not None and sum(a*b for a, b in zip(self.previous, q)) < 0:
            q = tuple(-v for v in q)
        self.previous, self.last_time = q, now
        self.count += 1
        return q if self.count >= self.stable_samples else None
