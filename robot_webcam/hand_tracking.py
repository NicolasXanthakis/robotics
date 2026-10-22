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


def estimate_flexion(landmarks, image_points=None):
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
    if image_points is not None:
        angles = refine_visible_extension(angles, image_points)
    return angles


def refine_visible_extension(angles, image_points):
    """Reduce inferred-depth curl only for long, visibly straight finger chains.

    Image points must use equal X/Y units (pixels), not normalized image axes.
    Continuous confidence avoids a hard snap at the straightness threshold.
    """
    p = np.asarray(image_points, dtype=float)
    if p.shape != (21, 2) or not np.isfinite(p).all():
        raise ValueError('Expected 21 finite image points in pixels')
    result = list(angles)
    palm_length = np.linalg.norm((p[5]+p[17])/2-p[0])
    if palm_length < 1e-8:
        return result
    for i, (mcp, pip, dip, tip) in enumerate(FINGER_CHAINS):
        chain = p[[mcp, pip, dip, tip]]
        lengths = np.linalg.norm(np.diff(chain, axis=0), axis=1)
        if min(lengths) < .10*palm_length:
            continue  # collapsed/occluded segments cannot establish extension
        visible = [bend(p[mcp], p[pip], p[dip]), bend(p[pip], p[dip], p[tip])]
        straightness = np.linalg.norm(p[tip]-p[mcp])/sum(lengths)
        confidence = min(float(np.clip((sum(lengths)/palm_length-.75)/.15, 0, 1)),
                         float(np.clip((20-max(visible))/10, 0, 1)),
                         float(np.clip((straightness-.94)/.04, 0, 1)))
        for offset, observed in zip((1, 2), visible):
            index = i*3+offset
            extended = min(result[index], max(0., observed-5.))
            result[index] += confidence*(extended-result[index])
    return result


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


def map_angles(raw, calibration=None, open_pose=None):
    """Defaults are direct degrees; optional calibration scales usable joint spans."""
    if len(raw) != 14 or not all(math.isfinite(v) for v in raw):
        raise ValueError('Expected 14 finite flexion angles')
    if calibration is not None:
        validate_calibration(calibration)
    if open_pose is not None and (len(open_pose) != 14
                                  or not all(math.isfinite(v) for v in open_pose)):
        raise ValueError('Expected 14 finite open-pose angles')
    result = []
    for i, (value, upper) in enumerate(zip(raw, LIMITS)):
        if calibration is not None:
            lower = calibration['open'][i]
            span = calibration['closed'][i] - lower
            # Weak or inverted captures retain direct mapping for this joint.
            if span >= 10.0:
                value = (value - lower) * upper / span
        elif open_pose is not None:
            # Apply an explicit open-pose capture immediately, without requiring
            # a fist capture. Three raw degrees absorb near-extension jitter.
            lower = open_pose[i] + 3.0
            if upper-lower >= 10.0:
                value = (value-lower)*upper/(upper-lower)
            else:
                value -= lower
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

    def update_wire(self, wire):
        """Queue a wrist frame with the same freshness policy as finger samples."""
        with self.lock:
            self.latest = None if wire is None else (time.monotonic(), wire)

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
    parser.add_argument('--wrist', action='store_true', help='Send v2 wrist orientation and fingers')
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
    from wrist_orientation import WristTracker, encode_sample, estimate_spread, estimate_thumb_spread, estimate_thumb_opposition
    wrist = WristTracker() if args.wrist else None
    sequence = 0
    facing = 'uncertain'
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
    neutral_click = threading.Event()

    def on_preview_click(event, x, y, flags, userdata):
        if wrist is not None and event == cv2.EVENT_LBUTTONDOWN and 10 <= x <= 250 and 10 <= y <= 45:
            neutral_click.set()

    previous_timestamp = -1
    start = time.monotonic()
    try:
        if not cap.isOpened():
            raise RuntimeError(f'Cannot open camera {args.camera}; close other camera apps')
        sender = TcpSender(args.host, args.port)
        cv2.namedWindow('Hand Tracking - TCP 8766')
        cv2.setMouseCallback('Hand Tracking - TCP 8766', on_preview_click)
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
                raw = angles = spread = thumb_spread = thumb_opposition = None
                tracking = f'NO {args.hand.upper()} HAND - holding ROS pose'
                if selected is not None:
                    score, index, label = selected
                    try:
                        if wrist is not None:
                            quaternion, _, facing = wrist.update(
                                result.hand_world_landmarks[index], label, time.monotonic())
                        if wrist is not None:
                            spread = estimate_spread(result.hand_world_landmarks[index], label)
                            thumb_spread = estimate_thumb_spread(result.hand_world_landmarks[index], label)
                            thumb_opposition = estimate_thumb_opposition(result.hand_world_landmarks[index], label)
                        raw = estimate_flexion(
                            result.hand_world_landmarks[index],
                            [(p.x*frame.shape[1], p.y*frame.shape[0])
                             for p in result.hand_landmarks[index]])
                        angles = map_angles(raw, calibration,
                                            open_capture if label == capture_side else None)
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
                        raw = angles = spread = thumb_spread = thumb_opposition = None
                        tracking = 'INVALID LANDMARKS - holding ROS pose'
                if raw is None:
                    history.clear()
                if wrist is None:
                    sender.update(angles)
                elif angles is None:
                    wrist.lost()
                    sender.update_wire(None)
                else:
                    sequence += 1
                    sender.update_wire(encode_sample(angles, quaternion, sequence, label, spread, thumb_spread, thumb_opposition))
                # A separate side panel keeps values readable at small camera resolutions.
                h = max(frame.shape[0], 700 if wrist else 520)
                canvas = np.zeros((h, frame.shape[1] + 520, 3), dtype=np.uint8)
                canvas[:frame.shape[0], :frame.shape[1]] = frame
                x = frame.shape[1] + 10
                lines = [tracking, sender.status, f'Target: {args.host}:{args.port}',
                         'CALIBRATED' if calibration else (
                             'OPEN-POSE CORRECTION' if open_capture is not None
                             and selected is not None and label == capture_side
                             else 'DEFAULT DEGREE MAPPING'),
                         'Sent targets (degrees):']
                if wrist is not None:
                    lines += [wrist.status, facing, 'N: neutral wrist | X: clear neutral']
                    if thumb_opposition is not None:
                        lines.append(f'Thumb inward: {thumb_opposition:.0f} deg')
                    if thumb_spread is not None:
                        lines.append(f'Thumb sideways: {thumb_spread:+.0f} deg')
                    if spread is not None:
                        lines.append('Spread: ' + ' '.join(f'{v:+.0f}' for v in spread))
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
                if wrist is not None:
                    cv2.rectangle(canvas, (10, 10), (250, 45), (70, 70, 70), -1)
                    cv2.putText(canvas, 'Set wrist neutral (N)', (18, 34),
                                cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.imshow('Hand Tracking - TCP 8766', canvas)
                key = cv2.waitKey(1) & 0xFF
                if ord('A') <= key <= ord('Z'):
                    key += ord('a') - ord('A')
                if neutral_click.is_set():
                    neutral_click.clear()
                    key = ord('n')
                if key == ord('q'):
                    break
                if wrist is not None and key == ord('n'):
                    notice = ('Wrist neutral captured' if wrist.capture_neutral(time.monotonic())
                              else 'Keep wrist tracked steadily before capturing neutral')
                    print(notice, flush=True)
                if wrist is not None and key == ord('x'):
                    wrist.clear_neutral()
                    notice = 'Wrist camera-relative orientation restored'
                    print(notice, flush=True)
                if key == ord('r'):
                    calibration = open_capture = capture_side = None
                    notice = 'Defaults restored (saved files unchanged)'
                if key in (ord('o'), ord('c')):
                    if (raw is None or len(history) < 5
                            or history[-1][0]-history[0][0] < 0.75):
                        notice = 'Keep one hand tracked steadily before capturing'
                    elif key == ord('o'):
                        open_capture = np.median([v for _, v, _ in history], axis=0).tolist()
                        capture_side = label
                        calibration = None
                        notice = ('Open correction active: straight fingers now map to zero. '
                                  'Optional: hold a fist, then C to calibrate full range')
                        print(notice, flush=True)
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
