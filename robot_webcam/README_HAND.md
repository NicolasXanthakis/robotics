# Windows webcam → ROS 2 hand

`hand_tracking.py` is independent of the original `pose_tracking.py` and
`pose_landmarker.task`; neither arm file is changed. It uses the same Windows
DirectShow/OpenCV and MediaPipe Tasks VIDEO architecture. Stop the arm webcam
script before running this one so they do not compete for the camera.

## Run immediately (no calibration)

1. Open **WSL Terminal 1**, build the existing receiver if needed, and launch it:

   ```bash
   cd /home/nicol/Projects/robotics/robot_hand_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ros2 launch hand_control tcp_visualize.launch.py
   ```

   **Keep this terminal and RViz open.** Expect an open stationary hand and
   `Listening on ('127.0.0.1', 8766)`. Stop any previous hand launch first.

2. Open **Windows PowerShell Terminal 2** (not Ubuntu/WSL):

   ```powershell
   Set-Location C:\Users\nicol\Projects\robot_webcam
   .\.venv\Scripts\python.exe .\hand_tracking.py
   ```

   **Keep this terminal and the webcam preview open.** Using the venv executable
   directly avoids PowerShell activation-policy issues. Your existing environment
   supplies NumPy, OpenCV, and MediaPipe; no dependency upgrades are needed.

3. Hold your **right hand**, palm visible and fingers straight, in good light.
   The preview should say `TRACKING RIGHT`, show a hand skeleton, calculated
   degree targets for all joints, and `TCP sending (20 Hz)`. Slowly curl one
   finger at a time, make a fist, then reopen. RViz should follow smoothly.
   The thumb controls `thumb_base` and `thumb_ip` using its two tracked bends.
   A rigid rotation of the whole hand should not itself curl the fingers,
   although inferred depth and occlusion can introduce errors.

4. Move the hand out of view. The preview should report no matching hand;
   sending stops and ROS holds the current pose after its one-second timeout.
   Bring the hand back to resume. Stop/restart Terminal 1 to test automatic
   reconnection; the preview must stay responsive while the receiver is absent.

5. Press **Q with the webcam window focused** to stop the sender. Press
   **Ctrl+C in Terminal 1** to stop the receiver and RViz. Close Terminal 2
   after its Python process exits.

## Optional calibration

Default mapping uses calculated flexion degrees directly and clamps to the
robot's position limits. No JSON file is loaded automatically.

- Hold an open hand steadily for at least one second; press **O**.
- Hold a comfortably closed fist for at least one second; press **C**.
- The script uses the median of the last second of samples and activates/saves
  `hand_calibration.json`. Any joint with less than 10 degrees of useful range
  keeps its default mapping, with the count shown in the preview.
- Press **R** to return to defaults without deleting saved calibration.
- To reuse a calibration next time:

  ```powershell
  .\.venv\Scripts\python.exe .\hand_tracking.py --calibration .\hand_calibration.json
  ```

For the thumb, deliberately bend both the knuckle and tip during the fist
capture. This maps their independent ranges; it cannot add opposition degrees
of freedom that the robotic thumb does not have.

## Camera-free verification

In Windows PowerShell, without starting a camera or ROS:

```powershell
Set-Location C:\Users\nicol\Projects\robot_webcam
.\.venv\Scripts\python.exe -m unittest -v test_hand_tracking
```

Expect nine passing tests: synthetic open/bent fingers and thumb, rigid motion,
mirror invariance, finger spread, invalid geometry, optional calibration, CSV
format, hand selection, and loopback TCP loss/reconnection. The socket test uses
an ephemeral port, so it does not interfere with 8765 or 8766.

To verify ROS output while Terminal 1 remains running, use **WSL Terminal 3**:

```bash
cd /home/nicol/Projects/robotics/robot_hand_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /joint_states --once
```

Expect 14 joint names and positions in **radians**. The Windows preview and
wire protocol use **degrees**; the receiver performs conversion. This command
exits on its own; Terminal 3 may then be closed.

## Protocol and angle method

The sender sends one ASCII CSV line, no header, at up to 20 Hz to
`127.0.0.1:8766`. The order is:

```text
index_mcp,index_pip,index_dip,middle_mcp,middle_pip,middle_dip,ring_mcp,ring_pip,ring_dip,pinky_mcp,pinky_pip,pinky_dip,thumb_base,thumb_ip
```

PIP/DIP use `180 - interior_angle` between adjacent world-coordinate bone
segments. MCP uses the angle of the proximal segment out of the palm plane,
relative to its in-plane outward direction. Lateral spreading is excluded;
bending magnitude is unsigned, so hyperextension can also appear as positive
curl. Defaults target open-to-fist motions. Thumb base uses landmarks 1–2–3
(a proxy for the model's simplified base); thumb IP uses 2–3–4.

The sender validates whole 14-value samples and drops invalid/tracking-lost
samples. A background worker reconnects about once a second and sends only
fresh samples (up to 0.25 seconds old), without a backlog. The receiver handles
smoothing, position limits, velocity limits, and stale-input pose holding.

## Model provenance

The included model was downloaded with permission from Google's official
versioned URL, also referenced by the official MediaPipe sample:

https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task

Official reference:
https://github.com/google-ai-edge/mediapipe-samples-web/blob/main/src/tasks/hand-landmarker.ts

SHA-256:
`fbc2a30080c3c557093b5ddfc334698132eb341044ccee322ccf8bcf3607cde1`

The script never downloads models automatically. If replacement is necessary,
request approval first, then explicitly download from the official source.
`--model` accepts an alternate local path.

## Troubleshooting

- **No webcam / black image:** close the arm tracker, Teams, browser camera
  tabs, and other camera apps. Try `--camera 1`. Check Windows camera privacy
  access for desktop apps. Run the original `webcam_test.py` separately.
- **No matching hand:** default is right hand. Try `--hand left`, or
  `--hand any` for diagnosis. `any` chooses highest handedness confidence and
  can switch between hands; use a specific side for normal operation. The
  preview is unmirrored, matching inference input; check its handedness label.
- **TCP disconnected:** keep WSL Terminal 1 running. In Windows PowerShell:

  ```powershell
  Test-NetConnection -ComputerName 127.0.0.1 -Port 8766
  ```

  Expect `TcpTestSucceeded : True`. This diagnostic connection may produce a
  harmless connect/disconnect log in ROS. Check firewall/WSL localhost forwarding
  if false. If using a WSL-IP fallback, explicitly bind the receiver to an
  interface reachable from Windows (`host:=0.0.0.0`) and pass that IP using
  `--host`; keep localhost as the default.
- **RViz stationary despite TCP connected:** check `TRACKING` and `TCP sending`,
  not merely connection status. Stop the sinusoidal demo publisher. Ensure
  sender and receiver both use 8766; leave the arm on 8765.
- **RViz blank:** set Fixed Frame to `palm`, enable Hand, and select
  `/robot_description`. Confirm `/joint_states` with Terminal 3. No TCP input
  is required to display the open hand.
- **Poor joint estimates:** improve lighting, avoid fingertip occlusion, keep
  the palm visible, and try optional calibration. World depth is inferred from
  a single camera. Thumb opposition and finger spreading are not robot joints.
- **Import errors:** use the `.venv` executable above. Do not upgrade the
  shared arm environment just to troubleshoot this script.
- **Arm and hand ROS visualizations together:** use separate ROS domains
  consistently within each WSL launch/inspection terminal; both projects use
  `/joint_states` and `/robot_description`. Port separation alone does not
  separate ROS topics. The original arm sender remains unchanged.

## Wrist orientation (optional)

The wrist mode adds orientation to the existing finger tracking. Build the hand
workspace as above, then use this receiver launch (stop other hand receivers):

```bash
ros2 launch hand_control wrist_visualize.launch.py rviz:=false
```

This command is headless. When ready for visual inspection, use `rviz:=true`.
The wrist RViz configuration uses `hand_base` as its fixed frame.
On Windows, start the existing sender with the additional flag:

```powershell
.\.venv\Scripts\python.exe .\hand_tracking.py --wrist
```

Hold a steady tracked hand and press **N** in the webcam preview, or click
**Set wrist neutral (N)**, to capture a neutral wrist pose. Uppercase and
lowercase keys both work; the preview and console report capture success.
Press **X** to clear neutral and restore camera-relative orientation. These
controls are independent of O/C/R finger calibration. Neutral is cleared when
the selected handedness changes and retained across temporary tracking loss.
The preview reports wrist acquisition, palm/back/edge facing, and neutral mode.
Three stable samples are required after loss or an abrupt orientation jump;
fingers can continue updating while wrist geometry is invalid.

Wrist mode sends newline-delimited version-2 JSON containing 14 finger degrees,
a sequence number, handedness, and an optional normalized xyzw quaternion.
The wrist receiver also accepts legacy CSV, which updates fingers and holds
wrist orientation. The original finger-only receiver requires CSV: omit
`--wrist` when using it. Do not run both receivers on port 8766.

The quaternion maps the palm model into `hand_base`; camera right/down/away
maps to base forward/left/up through a fixed proper rotation. This is a
coordinate convention, not measured camera calibration. Model +Z points along
the fingers and -Y toward the palm. Left hands map into the same right-hand
robot model. Neutral mode applies the spatial rotation from captured neutral
to current orientation in base axes. There is no wrist position tracking.
MediaPipe estimates depth from one camera, so occlusion and handedness errors
still require a live visual check.

The receiver publishes one dynamic `hand_base -> palm` transform, pivoting
55 mm below the existing palm center. Quaternion smoothing defaults to a
0.15-second time constant and a 3 rad/s speed limit. It holds orientation on
invalid frames, disconnect, or 0.5 seconds without fresh orientation; fingers
retain their independent one-second timeout. Tuning launch parameters are
`wrist_tau`, `wrist_max_speed`, and `wrist_timeout`.

Additional camera-free verification from the sourced ROS workspace:

```bash
python3 -m pytest -q src/hand_control/test
python3 src/hand_control/test/tcp_smoke.py --wrist
```

The smoke test explicitly disables RViz and verifies joint states, wrist TF,
malformed input recovery, holds, and reconnect. Stop other hand receivers first.

## Finger spread and palm model

Wrist mode (`--wrist`) now also tracks four sideways MCP rotations: index,
middle, ring, and pinky. Bring your fingers together, then fan them apart;
the preview shows signed spread degrees. These measurements use proximal
finger directions projected into the palm plane, independently of curl and
wrist neutral capture. Left hands are retargeted to the same right-hand model.
When a finger is too curled to measure spread reliably, all four spread
angles hold their last targets until the palm-plane projection is usable again.

The model now has **18 joints**: the original 14 flexion joints followed by
`index_spread,middle_spread,ring_spread,pinky_spread` in joint-state messages.
The original 14-value CSV protocol is preserved and keeps spread at zero.
Version-2 JSON adds optional `finger_spread_deg` (four signed finite degrees in
that order). Old JSON without this field continues to work and holds spread.
Spread is clamped to ±0.45 radians (about ±26 degrees), smoothed, and limited
to 1 rad/s. Disconnects and stale frames hold the current published pose.
The sinusoidal demo sweeps all 18 joints.

The palm is a single blue, tapered solid with flat faces and bevelled edges,
wider at the knuckles and narrower at the wrist. Its generator is included under
`robot_hand_description/meshes/generate_palm.py`. The thumb attaches lower on the palm and uses 35 mm and 26 mm segments.
This is a simplified visual model, not an anatomical scan. Thumb opposition, thumb sideways movement,
finger twisting, and palm cupping remain outside this model.

### Thumb sideways tracking

The current model has **19 joints**, adding `thumb_spread` after the four
finger spread joints. In wrist mode, the thumb metacarpal (landmarks 1→2)
is projected into the palm plane to measure sideways opening independently
of the two thumb bends. The model's resting opening is 0.9 radians; the
additional joint moves ±0.8 radians with smoothing and a 1 rad/s speed limit.
The preview reports `Thumb sideways` in degrees. Wrist neutral capture does
not change this anatomical angle.

Version-2 JSON adds optional `thumb_spread_deg`. Missing/null values hold the
last sideways target; legacy CSV resets all spread targets to zero.
A thumb pointing nearly perpendicular to the palm plane holds its sideways
target because that projection is unreliable. This adds palm-plane thumb
opening/closing; full thumb opposition and palm cupping remain unmodeled.

### Responsiveness

The wrist visualization launch now defaults to `alpha:=0.55` for fingers and
spread, `wrist_tau:=0.06`, `wrist_max_speed:=6.0`, and
`motion_speed_scale:=2.5`. The last parameter scales URDF velocity bounds for
this visualization only; it does not modify the URDF or command hardware.
The original finger-only receiver still defaults to alpha 0.2 and speed scale
1.0. Sender and ROS publication remain at 20 Hz. Less filtering makes camera
jitter more visible. Camera inference and capture add their own latency.

To return to the earlier wrist visualization response:

```bash
ros2 launch hand_control wrist_visualize.launch.py alpha:=0.2 motion_speed_scale:=1.0 wrist_tau:=0.15 wrist_max_speed:=3.0
```

### Moving thumb base (CMC opposition)

The latest model has **20 movable joints**. `thumb_opposition` rotates the blue
thenar section and all downstream thumb joints relative to the main palm.
At zero, the pad and thumb retain their existing resting positions. Positive
rotation about the local negative Z axis moves the assembly toward the palmar
side and the center of the palm. Its range is 0–1.2 radians, with the same
visual smoothing and speed scaling as other auxiliary joints.

In wrist mode, landmarks 1→2 supply the thumb metacarpal direction. Palm-plane
opening drives sideways spread; elevation toward the palm drives opposition.
These angles are decomposed separately, so inward movement does not itself
close the sideways joint. The preview shows `Thumb inward` in degrees.
Wrist neutral capture and finger calibration do not change this measurement.

Optional v2 field `thumb_opposition_deg` carries the angle. Missing/null holds
the previous target; legacy CSV returns it to zero. A degenerate metacarpal
projection is held. This is a single-hinge approximation of thumb opposition,
not the full coupled rotations of the anatomical saddle joint. Camera depth
errors may cause cross-talk and require adjustment after live testing.

### Straight-finger correction with O

Hold your selected hand open with fingers comfortably straight for one second,
then focus the webcam preview and press **O**. The median tracked open pose is
now applied immediately: the panel shows `OPEN-POSE CORRECTION`. This removes
per-joint bend offsets without requiring a fist capture. A three-degree raw
angle tolerance near extension suppresses small residual bends; the remaining
range is rescaled so full closure stays available. No automatic calibration
is performed while you gesture.

Optional **C** still captures a fist and activates/saves the two-pose range
calibration. **R** clears both corrections and restores defaults. Open-only
correction is session-local and applies only to the same handedness captured
with O. Wrist neutral N and the sideways/opposition measurements are unchanged.
If errors vary strongly with viewing angle, open calibration cannot correct
all camera depth/occlusion errors; keep the hand well lit and visible.

### Visible straight-finger check

Middle/tip curl now also uses the image landmarks when a finger is visibly
long and nearly straight. Confidence blends continuously based on projected
length relative to the palm, chain straightness, and both image bend angles.
At high confidence, small image bends (up to five degrees) map to extension,
reducing false curl caused by inferred world-depth bends. Short/foreshortened,
collapsed, or visibly bent chains keep the 3D estimate. Pixel coordinates
preserve the image aspect ratio. Knuckle curl and thumb angles are unchanged.

This is a conservative visual heuristic, not measured depth: unusual poses
can still be ambiguous. Try the automatic correction before recapturing O;
open-pose correction remains available for residual per-joint offsets.
