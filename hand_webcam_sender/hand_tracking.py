"""Windows MediaPipe hand sender; pure geometry/protocol helpers work without a camera."""

import argparse
import json
import math
from pathlib import Path
import socket
import threading
import time

import numpy as np

JOINT_NAMES = tuple(
    f'{finger}_{joint}'
    for finger in ('index', 'middle', 'ring', 'pinky')
    for joint in ('mcp', 'pip', 'dip')
) + ('thumb_base', 'thumb_ip')
LIMITS = tuple(math.degrees(v) for v in (1.57, 1.75, 1.4) * 4 + (1.57, 1.4))
FINGER_CHAINS = ((5, 6, 7, 8), (9, 10, 11, 12), (13, 14, 15, 16), (17, 18, 19, 20))
CONNECTIONS = tuple(
    (a, b) for chain in ((0, 1, 2, 3, 4), (0, 5, 6, 7, 8),
                        (5, 9, 10, 11, 12), (9, 13, 14, 15, 16),
                        (13, 17, 18, 19, 20), (0, 17))
    for a, b in zip(chain, chain[1:])
)


def unit(vector):
    """Reject degenerate geometry rather than returning NaN angles."""
    length = np.linalg.norm(vector)
    if not np.isfinite(length) or length < 1e-8:
        raise ValueError('Degenerate landmark geometry')
    return vector / length


def bend(a, b, c):
    """Return 180 minus the interior joint angle: straight is zero degrees."""
    cosine = np.dot(unit(a - b), unit(c - b))
    return 180.0 - math.degrees(math.acos(float(np.clip(cosine, -1.0, 1.0))))


def estimate_flexion(landmarks):
    """Estimate 12 finger bends and two simplified thumb bends in degrees.

    Uses world coordinates. MCP is the magnitude of out-of-palm-plane bending,
    ignoring lateral spread. This unsigned proxy also responds to hyperextension;
    it is intended for open-to-fist gestures, not anatomical motion capture.
    """
    points = np.asarray([
        (p.x, p.y, p.z) if hasattr(p, 'x') else p for p in landmarks
    ], dtype=float)
    if points.shape != (21, 3) or not np.isfinite(points).all():
        raise ValueError('Expected 21 finite 3D landmarks')
    normal = unit(np.cross(points[5] - points[0], points[17] - points[0]))
    angles = []
    for mcp, pip, dip, tip in FINGER_CHAINS:
        reference = points[mcp] - points[0]
        reference = unit(reference - np.dot(reference, normal) * normal)
        proximal = unit(points[pip] - points[mcp])
        flexion = math.degrees(math.atan2(
            abs(float(np.dot(proximal, normal))), float(np.dot(proximal, reference))))
        angles.extend((flexion, bend(points[mcp], points[pip], points[dip]),
                       bend(points[pip], points[dip], points[tip])))
    angles.extend((bend(points[1], points[2], points[3]),
                   bend(points[2], points[3], points[4])))
    return angles


def validate_calibration(data):
    """Validate optional open/closed samples, keeping defaults for weak ranges."""
    if not isinstance(data, dict) or data.get('joint_names') != list(JOINT_NAMES):
        raise ValueError('Calibration has incompatible joint names/order')
    for key in ('open', 'closed'):
        values = data.get(key)
        if not isinstance(values, list) or len(values) != 14:
            raise ValueError(f'Calibration {key} must have 14 values')
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
            raise ValueError('Calibration contains invalid numbers')
    return data


def map_angles(raw, calibration=None):
    """Defaults are direct degrees; optional calibration scales usable joint spans."""
    if len(raw) != 14 or not all(math.isfinite(v) for v in raw):
        raise ValueError('Expected 14 finite flexion angles')
    if calibration is not None:
        validate_calibration(calibration)
    result = []
    for i, (value, upper) in enumerate(zip(raw, LIMITS)):
        if calibration is not None:
            lower = calibration['open'][i]
            span = calibration['closed'][i] - lower
            # Weak or inverted captures retain direct mapping for this joint.
            if span >= 10.0:
                value = (value - lower) * upper / span
        result.append(max(0.0, min(upper, value)))
    return result


def encode_angles(angles):
    """Encode exactly one 14-field CSV degree frame in receiver joint order."""
    if len(angles) != 14 or not all(math.isfinite(v) for v in angles):
        raise ValueError('Expected 14 finite angles')
    return (','.join(f'{value:.3f}' for value in angles) + '\n').encode('ascii')


class TcpSender:
    """Send only the latest fresh sample; network failures never stop the preview."""

    def __init__(self, host, port):
        self.host, self.port = host, port
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.latest = None
        self.status = 'TCP connecting'
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def update(self, angles):
        with self.lock:
            self.latest = None if angles is None else (time.monotonic(), encode_angles(angles))

    def run(self):
        connection = None
        retry_at = 0.0
        try:
            while not self.stop.is_set():
                now = time.monotonic()
                if connection is None and now >= retry_at:
                    try:
                        connection = socket.create_connection((self.host, self.port), timeout=0.2)
                        connection.settimeout(0.2)
                        self.status = 'TCP connected'
                    except OSError:
                        self.status = 'TCP disconnected - retrying'
                        retry_at = time.monotonic() + 1.0
                with self.lock:
                    sample = self.latest
                if connection is not None and sample and now - sample[0] <= 0.25:
                    try:
                        connection.sendall(sample[1])
                        self.status = 'TCP sending (20 Hz)'
                    except OSError:
                        connection.close()
                        connection = None
                        self.status = 'TCP disconnected - retrying'
                        retry_at = time.monotonic() + 1.0
                elif connection is not None:
                    self.status = 'TCP connected - no fresh tracking'
                self.stop.wait(0.05)
        finally:
            if connection is not None:
                connection.close()

    def close(self):
        self.stop.set()
        self.worker.join(timeout=2.0)


def select_hand(result, side, min_score=0.5):
    """Choose the highest-confidence matching hand; never fall back to the other side."""
    choices = []
    for i, categories in enumerate(result.handedness):
        if not categories or i >= len(result.hand_world_landmarks):
            continue
        category = categories[0]
        label = category.category_name.lower()
        if (side == 'any' or label == side) and category.score >= min_score:
            choices.append((category.score, i, label))
    return max(choices, default=None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--hand', choices=('right', 'left', 'any'), default='right')
    parser.add_argument('--model', type=Path,
                        default=Path(__file__).with_name('hand_landmarker.task'))
    parser.add_argument('--calibration', type=Path, help='Optional saved calibration JSON')
    parser.add_argument('--save-calibration', type=Path,
                        default=Path(__file__).with_name('hand_calibration.json'))
    args = parser.parse_args()
    if not args.model.is_file():
        parser.error(f'Model not found: {args.model}; see README_HAND.md for official download')
    calibration = None
    if args.calibration:
        try:
            calibration = validate_calibration(json.loads(args.calibration.read_text()))
        except (OSError, ValueError, TypeError) as error:
            parser.error(f'Cannot load calibration: {error}')
    # Lazy imports allow geometry/protocol tests without OpenCV or MediaPipe.
    import cv2
    import mediapipe as mp

    options = mp.tasks.vision.HandLandmarkerOptions(
        base_options=mp.tasks.BaseOptions(model_asset_path=str(args.model)),
        running_mode=mp.tasks.vision.RunningMode.VIDEO,
        num_hands=2, min_hand_detection_confidence=0.5,
        min_hand_presence_confidence=0.5, min_tracking_confidence=0.5)
    cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)
    sender = None
    open_capture = None
    capture_side = None
    history = []
    notice = 'Defaults active. Optional: O=open, C=fist, R=defaults, Q=quit'
    previous_timestamp = -1
    start = time.monotonic()
    try:
        if not cap.isOpened():
            raise RuntimeError(f'Cannot open camera {args.camera}; close other camera apps')
        sender = TcpSender(args.host, args.port)
        with mp.tasks.vision.HandLandmarker.create_from_options(options) as landmarker:
            while True:
                success, frame = cap.read()
                if not success:
                    raise RuntimeError('Webcam frame capture failed')
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                timestamp = max(previous_timestamp + 1, int((time.monotonic() - start) * 1000))
                previous_timestamp = timestamp
                image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                result = landmarker.detect_for_video(image, timestamp)
                selected = select_hand(result, args.hand)
                raw = angles = None
                tracking = f'NO {args.hand.upper()} HAND - holding ROS pose'
                if selected is not None:
                    score, index, label = selected
                    try:
                        raw = estimate_flexion(result.hand_world_landmarks[index])
                        angles = map_angles(raw, calibration)
                        tracking = f'TRACKING {label.upper()} ({score:.2f})'
                        now = time.monotonic()
                        history = [(t, v, side) for t, v, side in history
                                   if now - t <= 1.0 and side == label]
                        history.append((now, raw, label))
                        h, w = frame.shape[:2]
                        points = result.hand_landmarks[index]
                        for a, b in CONNECTIONS:
                            pa, pb = points[a], points[b]
                            cv2.line(frame, (int(pa.x*w), int(pa.y*h)),
                                     (int(pb.x*w), int(pb.y*h)), (0, 220, 0), 2)
                        for point in points:
                            cv2.circle(frame, (int(point.x*w), int(point.y*h)),
                                       3, (0, 0, 255), -1)
                    except (ValueError, IndexError):
                        raw = angles = None
                        tracking = 'INVALID LANDMARKS - holding ROS pose'
                if raw is None:
                    history.clear()
                sender.update(angles)
                # A separate side panel keeps values readable at small camera resolutions.
                h = max(frame.shape[0], 520)
                canvas = np.zeros((h, frame.shape[1] + 520, 3), dtype=np.uint8)
                canvas[:frame.shape[0], :frame.shape[1]] = frame
                x = frame.shape[1] + 10
                lines = [tracking, sender.status, f'Target: {args.host}:{args.port}',
                         'CALIBRATED' if calibration else 'DEFAULT DEGREE MAPPING',
                         'Sent targets (degrees):']
                if angles is not None:
                    for i, finger in enumerate(('index', 'middle', 'ring', 'pinky')):
                        values = angles[i*3:i*3+3]
                        lines.append(f'{finger:6} MCP {values[0]:5.1f} PIP {values[1]:5.1f} DIP {values[2]:5.1f}')
                    lines.append(f'thumb  base {angles[12]:5.1f} IP {angles[13]:5.1f}')
                else:
                    lines.append('No valid targets sent')
                lines += ['O: capture open (hold 1 second first)',
                          'C: capture fist + save (after O)',
                          'R: reset to defaults | Q: quit']
                # Wrap operational notices to keep them inside the debug panel.
                import textwrap
                lines.extend(textwrap.wrap(notice, width=56))
                for row, text in enumerate(lines):
                    cv2.putText(canvas, text, (x, 24 + row*25), cv2.FONT_HERSHEY_SIMPLEX,
                                0.48, (230, 230, 230), 1, cv2.LINE_AA)
                cv2.imshow('Hand Tracking - TCP 8766', canvas)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                if key == ord('r'):
                    calibration = open_capture = capture_side = None
                    notice = 'Defaults restored (saved files unchanged)'
                if key in (ord('o'), ord('c')):
                    if raw is None or len(history) < 5:
                        notice = 'Keep one hand tracked steadily before capturing'
                    elif key == ord('o'):
                        open_capture = np.median([v for _, v, _ in history], axis=0).tolist()
                        capture_side = label
                        notice = 'Open pose captured; hold a fist for 1 second, then press C'
                    elif open_capture is None or label != capture_side:
                        notice = 'Capture O first using the same hand'
                    else:
                        closed = np.median([v for _, v, _ in history], axis=0).tolist()
                        calibration = validate_calibration({
                            'joint_names': list(JOINT_NAMES), 'open': open_capture,
                            'closed': closed})
                        weak = sum(c-o < 10 for o, c in zip(open_capture, closed))
                        try:
                            args.save_calibration.write_text(json.dumps(calibration, indent=2))
                            notice = f'Calibration saved; {weak} weak joints retain defaults'
                        except OSError as error:
                            notice = f'Calibration active but not saved: {error}'
    finally:
        if sender is not None:
            sender.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
