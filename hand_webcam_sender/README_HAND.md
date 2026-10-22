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
