import cv2
import mediapipe as mp
import time
import numpy as np
import socket

BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
RunningMode = mp.tasks.vision.RunningMode

options = PoseLandmarkerOptions(
    base_options=BaseOptions(
        model_asset_path="pose_landmarker.task"
    ),
    running_mode=RunningMode.VIDEO,
    num_poses=1
)

# Connections for the right arm
connections = [(12, 14), (14, 16)]                                                                             

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
start_time = time.monotonic()

def calculate_angle(shoulder, elbow, wrist):
    a = np.array([shoulder.x - elbow.x, shoulder.y - elbow.y])
    b = np.array([wrist.x - elbow.x, wrist.y - elbow.y])

    cosine = np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))                
    angle = np.arccos(np.clip(cosine, -1.0, 1.0))

    return np.degrees(angle)


def calculate_angle_3d(a, b, c):
    a = np.array([a.x, a.y, a.z])
    b = np.array([b.x, b.y, b.z])                               
    c = np.array([c.x, c.y, c.z])

    ba = a - b
    bc = c - b

    cosine = np.dot(ba, bc) / (
        np.linalg.norm(ba) * np.linalg.norm(bc)
    )

    return np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0)))


def calculate_body_yaw(landmarks):
    # Convert MediaPipe's 3D landmarks into vectors
    def vec(index):                                                 
        p = landmarks[index]
        return np.array([p.x, p.y, p.z], dtype=float)

    left_shoulder = vec(11)
    right_shoulder = vec(12)
    right_elbow = vec(14)

    left_hip = vec(23)
    right_hip = vec(24)

    # Define the torso's sideways axis
    sideways = right_shoulder - left_shoulder
    sideways /= np.linalg.norm(sideways)

    # Define the torso's upward axis
    hip_center = (left_hip + right_hip) / 2
    shoulder_center = (left_shoulder + right_shoulder) / 2

    up = shoulder_center - hip_center

    # Make 'up' perpendicular to 'sideways'
    up -= np.dot(up, sideways) * sideways
    up /= np.linalg.norm(up)

    # Third axis: perpendicular to the other two
    forward = np.cross(sideways, up)
    forward /= np.linalg.norm(forward)

    # Upper-arm direction
    arm = right_elbow - right_shoulder

    # Express the arm direction relative to the torso
    lateral = np.dot(arm, sideways)
    frontal = np.dot(arm, forward)

    yaw = np.degrees(np.arctan2(frontal, lateral))
    return yaw

def calculate_shoulder_pitch_3d(landmarks):
    def vec(index):
        p = landmarks[index]
        return np.array([p.x, p.y, p.z], dtype=float)

    left_shoulder = vec(11)
    right_shoulder = vec(12)
    right_elbow = vec(14)

    left_hip = vec(23)
    right_hip = vec(24)

    # Define the torso coordinate system
    sideways = right_shoulder - left_shoulder
    sideways /= np.linalg.norm(sideways)

    hip_center = (left_hip + right_hip) / 2
    shoulder_center = (left_shoulder + right_shoulder) / 2

    up = shoulder_center - hip_center
    up -= np.dot(up, sideways) * sideways
    up /= np.linalg.norm(up)

    forward = np.cross(sideways, up)
    forward /= np.linalg.norm(forward)

    # Upper-arm direction
    arm = right_elbow - right_shoulder

    # Project onto torso vertical and forward axes
    vertical = np.dot(arm, up)
    frontal = np.dot(arm, forward)

    # Shoulder pitch angle
    pitch = np.degrees(np.arctan2(frontal, -vertical))

    return pitch


def calculate_torso_yaw(landmarks):
    left = landmarks[11]
    right = landmarks[12]

    dx = right.x - left.x
    dz = right.z - left.z

    raw_yaw = np.degrees(np.arctan2(dz, dx))

    # Neutral orientation is approximately 180 degrees
    torso_yaw = (raw_yaw - 180 + 180) % 360 - 180

    return torso_yaw

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(('127.0.0.1', 8765))

print("Connected to ROS 2 receiver!")


with PoseLandmarker.create_from_options(options) as landmarker:
    while True:
        success, frame = cap.read()

        if not success:
            break

        # frame = cv2.flip(frame, 1)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        timestamp_ms = int((time.monotonic() - start_time) * 1000)
        result = landmarker.detect_for_video(image, timestamp_ms)

        if result.pose_landmarks:
            landmarks = result.pose_landmarks[0]

            world_landmarks = result.pose_world_landmarks[0]

            shoulder = world_landmarks[12]
            elbow = world_landmarks[14]

            torso_yaw = calculate_torso_yaw(world_landmarks)
            shoulder_yaw = calculate_body_yaw(world_landmarks)

            print(
                f"Shoulder: ({shoulder.x:.2f}, {shoulder.y:.2f}, {shoulder.z:.2f}) "
                f"Elbow: ({elbow.x:.2f}, {elbow.y:.2f}, {elbow.z:.2f})"
            )

            elbow_angle = calculate_angle_3d(
                world_landmarks[12],  # Shoulder
                world_landmarks[14],  # Elbow
                world_landmarks[16]   # Wrist
            )

            shoulder_angle = calculate_shoulder_pitch_3d(world_landmarks)

            client.sendall(
                f"{shoulder_angle:.2f},{elbow_angle:.2f},"
                f"{shoulder_yaw:.2f},{torso_yaw:.2f}\n".encode("utf-8")
            )

            cv2.putText(
                frame,
                f"Elbow: {elbow_angle:.1f} deg",
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1, (255, 255, 255), 2
            )

            cv2.putText(
                frame,
                f"Shoulder: {shoulder_angle:.1f} deg",
                (30, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                1, (255, 255, 255), 2
            )

            cv2.putText(
                frame,
                f"Shoulder yaw: {shoulder_yaw:.1f} deg",
                (30, 130),
                cv2.FONT_HERSHEY_SIMPLEX,
                1, (255, 255, 255), 2
            )

            cv2.putText(
                frame,
                f"Torso yaw: {torso_yaw:.1f} deg",
                (30, 170),
                cv2.FONT_HERSHEY_SIMPLEX,
                1, (255, 255, 255), 2
            )

            height, width = frame.shape[:2]

            for start, end in connections:
                p1 = landmarks[start]
                p2 = landmarks[end]

                x1, y1 = int(p1.x * width), int(p1.y * height)
                x2, y2 = int(p2.x * width), int(p2.y * height)

                cv2.line(frame, (x1, y1), (x2, y2),
                         (0, 255, 0), 3)

            for index in [12, 14, 16]:
                p = landmarks[index]
                x, y = int(p.x * width), int(p.y * height)

                cv2.circle(frame, (x, y), 7, (0, 0, 255), -1)

        cv2.imshow("Arm Tracking", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


client.close()
cap.release()
cv2.destroyAllWindows()