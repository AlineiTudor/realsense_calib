# Multi-Camera Extrinsic Calibration — Instructions

## Overview

This package calibrates the extrinsic transform between two cameras using the
**hand-eye (AX=XB)** method. No manual geometry measurements are needed.

You place two ChArUco boards in the scene (they do NOT need to be rigidly
connected). Each camera sees one board. You move the camera rig to multiple
positions, and the solver recovers the camera-to-camera transform automatically.

## 1. Dependencies (install on Jetson)

```bash
# Python libraries
pip3 install opencv-contrib-python numpy scipy pyyaml

# ROS2 packages (should already be available)
sudo apt install ros-${ROS_DISTRO}-image-transport
```

If `opencv-python` is already installed, replace it with `opencv-contrib-python`
(superset that includes `cv2.aruco`):

```bash
pip3 uninstall opencv-python && pip3 install opencv-contrib-python
```

## 2. Prepare the ChArUco Boards

### Print two boards

Generate printable board images:

```bash
python3 -m multicam_calibration.generate_boards --output_dir /tmp/boards
```

Or use your existing boards. The default config expects:
- **DICT_4X4_50**, 7x5 squares, 39.5mm square, 30mm marker

### Mount on rigid surfaces

Glue each board onto a rigid flat surface (foam board, MDF, acrylic). The
board must be perfectly flat — any warping degrades accuracy.

Verify the printed square size with a ruler (should be 39.5mm). Adjust the
`square_length` in `config/calibration_params.yaml` if it differs.

## 3. Set Up the Scene

```
                Board A                       Board B
              (stationary)                  (stationary)
            +-----------+                 +-----------+
            |           |                 |           |
            |  ChArUco  |                 |  ChArUco  |
            |           |                 |           |
            +-----------+                 +-----------+

                 ^                              ^
                 |                              |
              camera1                        camera2
              (D435i)                        (D455)
               \____________________________/
                     Camera rig (moves)
```

- Place board A where **camera1** can see it clearly.
- Place board B where **camera2** can see it clearly.
- The boards must be **stationary** throughout the calibration.
- The boards do NOT need to be attached to each other.
- The boards can be at any distance and any angle relative to each other.

## 4. Build and Run

```bash
cd ~/ROS_workspaces/adi_ros_dev
colcon build --packages-select multicam_calibration
source install/setup.bash

ros2 launch multicam_calibration calibration.launch.py
```

### During calibration

1. Position the camera rig so both cameras see their respective boards.
2. Hold still for a moment — the node captures the pose pair automatically.
3. **Move the camera rig** to a new position (translate and/or rotate it).
   - The node requires at least 3cm translation or 8deg rotation between poses.
   - More diverse positions = better result.
4. Repeat until the required number of poses is collected (default: 15).
5. The node automatically computes the result and prints it.

### Tips for good calibration

- **Vary both translation and rotation** between poses. Don't just slide
  the rig sideways — tilt it, rotate it, move it closer/farther.
- Keep both boards fully visible at each position (no partial occlusion).
- Avoid extreme viewing angles (board nearly edge-on to camera).
- 15 poses is the minimum for decent results. For high-precision work,
  increase `min_samples` to 25-30 in the config.
- The boards should be well-lit with even lighting (avoid strong shadows
  or glare on the boards).

## 5. Output

The node prints results from five different solver methods (TSAI, PARK,
HORAUD, ANDREFF, DANIILIDIS) so you can compare. If they converge to
similar values, the calibration is reliable.

Example output:

```
CAMERA_LINK FRAME results (X forward, Y left, Z up):
  TSAI          t=[-0.098012, -0.031845, 0.000312]  rpy=[0.0012, -0.0008, -1.5709]
  PARK          t=[-0.097998, -0.032001, 0.000298]  rpy=[0.0010, -0.0006, -1.5710]
  HORAUD        t=[-0.098105, -0.031912, 0.000356]  rpy=[0.0014, -0.0009, -1.5707]
  ...

URDF joint (paste into your xacro):
  <origin xyz="-0.097998 -0.032001 0.000298" rpy="0.001000 -0.000600 -1.571000"/>
```

The result is also saved to `/tmp/calibration_result.yaml` (or the path
configured in `output_file`).

### Consistency check

The node reports a "board-to-board transform std" — this measures how
consistent the recovered geometry is across all captured poses. Lower is
better:

- **Translation std < 3mm**: Excellent calibration.
- **Translation std 3-10mm**: Acceptable, consider more poses.
- **Translation std > 10mm**: Poor — check board flatness, lighting, and
  that the boards didn't move during calibration.

## 6. Apply the Result

Paste the URDF `<origin>` line into your camera2 joint in
`ad_r1m_perception_cuvslam/urdf/realsense_calibration.urdf.xacro`:

```xml
<joint name="camera2" type="fixed">
  <parent link="base_link"/>
  <child link="camera2_link"/>
  <origin xyz="NEW_X NEW_Y NEW_Z" rpy="NEW_ROLL NEW_PITCH NEW_YAW"/>
</joint>
```

After applying, run cuVSLAM and check for visual discontinuities at the
camera boundary.

## 7. Config Reference

### `config/calibration_params.yaml`

| Parameter | Description |
|-----------|-------------|
| `board_a.dictionary` | ArUco dictionary (e.g., `DICT_4X4_50`) |
| `board_a.squares_x/y` | Board grid dimensions |
| `board_a.square_length` | Square side length in meters |
| `board_a.marker_length` | Marker side length in meters |
| `calibration.min_samples` | Number of rig positions to collect |
| `calibration.min_translation_m` | Min translation between poses (m) |
| `calibration.min_rotation_deg` | Min rotation between poses (deg) |
| `calibration.camera*_topic` | ROS topic names for images/info |
| `calibration.output_file` | Path to save YAML result |

### `config/realsense_calibration.yaml`

Camera configuration for the RealSense nodes. Adjust serial numbers and
profiles to match your hardware.
