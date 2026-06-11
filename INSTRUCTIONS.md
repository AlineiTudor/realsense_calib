# Multi-Camera Extrinsic Calibration — Instructions

## 1. Dependencies (install on Jetson)

```bash
# ROS2 packages
sudo apt install ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-image-transport

# Python libraries
pip3 install opencv-contrib-python numpy scipy transforms3d
```

If `opencv-python` is already installed, replace it with `opencv-contrib-python`
(superset that includes `cv2.aruco`):

```bash
pip3 uninstall opencv-python && pip3 install opencv-contrib-python
```

## 2. Build the L-Shaped Target

### Materials
- Two printed ChArUco boards (DICT_4X4, 7x5, 39.5mm square, 30mm marker)
- Two rigid flat surfaces (foam board, MDF, acrylic — anything that stays flat)
- An aluminum L-bracket or two pieces joined at 90°
- Glue or double-sided tape
- A ruler or calipers (mm precision)
- A digital angle gauge (optional but recommended)

### Assembly
1. Glue each printed ChArUco board onto a rigid flat surface.
   - Verify the printed square size with a ruler (should be 39.5mm).
   - The board must be perfectly flat — any warping ruins the calibration.

2. Attach the two boards to the L-bracket so they form an ~90° angle.
   - Board A faces camera1 (D455).
   - Board B faces camera2 (D435i).

3. The boards must be RIGIDLY attached. Any flex between them invalidates
   the calibration.

## 3. Measure the L-Shape Geometry

This is the most critical step. You need to measure the 3D transform from
board A's origin to board B's origin.

### Coordinate frame convention

Each ChArUco board's coordinate frame origin is at its **first chessboard
corner** (top-left inner corner when the board is oriented with markers
reading correctly):

```
  Board A (face up, markers readable)

  +------+------+------+------+------+------+------+
  |      |      |      |      |      |      |      |
  | [AR] |      | [AR] |      | [AR] |      | [AR] |
  |      |      |      |      |      |      |      |
  O----->+------+------+------+------+------+------+   O = origin
  |  X   |      |      |      |      |      |      |
  | axis | [AR] |      | [AR] |      | [AR] |      |
  |      |      |      |      |      |      |      |
  +------+------+------+------+------+------+------+
  |      ...
  Y axis (downward)

  Z axis = into the board surface (right-hand rule)
```

### What to measure

The L-shape has the hinge at the **top** of Board A, with Board B
extending horizontally from it:

```
  Side view of L-shape:

       hinge
       +  -------------------------------- Board B (horizontal)
        |   camera2 sees this face ↑
        |
        |  ← camera1 sees this face
        |
        Board A (vertical)
```

The fold runs along Board A's **X axis** (the top edge). This means
the rotation from Board A's frame to Board B's frame is primarily
around the X axis.

**translation_x**: Lateral offset between the two board origins along
the hinge line. If both boards are aligned (their left edges are flush
at the hinge), this is 0. If Board B is shifted left/right relative to
Board A, measure the offset in meters.

**translation_y**: Distance from Board A's origin (top-left inner corner
of the ChArUco pattern) to the hinge line, measured downward. If the
pattern starts right at the top edge of the physical board and the hinge
is at that edge, this is ~0. If there is a margin between the pattern
edge and the hinge, measure it (typically a few mm).

**translation_z**: Depth offset at the hinge (e.g., board thickness if
the boards don't meet perfectly edge-to-edge). Often negligible — set
to 0 if the boards join cleanly.

**rotation_rpy**: The rotation from Board A's frame to Board B's frame
as [roll, pitch, yaw] in radians around Board A's axes. For a 90° fold
around the X axis (the hinge line), use `[1.5708, 0.0, 0.0]`. Measure
the actual angle with a digital angle gauge and convert to radians
(angle_deg × π / 180). If the fold is not exactly 90°, adjust the
roll value accordingly.

### Filling in the config

Edit `config/calibration_params.yaml`:

```yaml
l_shape:
  translation_x: 0.0                   # lateral offset (meters)
  translation_y: 0.0                   # origin-to-hinge along Y (meters)
  translation_z: 0.0                   # depth offset (meters)
  rotation_rpy: [1.5708, 0.0, 0.0]    # [roll, pitch, yaw] radians
```

### Verifying the rotation sign

If you're unsure whether roll should be +90° or -90°, run a quick test:
collect a few samples and check if the output translation roughly matches
your expectation from the physical setup. If the signs are flipped, negate
the roll value (use `-1.5708` instead).

### Tips for accurate measurement
- Use calipers rather than a tape measure.
- Mark the board origin corner on the physical board with a dot.
- Measure from dot to dot if possible.
- The angle matters less than the translation for your use case.
- Run the calibration, check the std deviation. If translation std > 5mm,
  re-check your L-shape rigidity and measurements.

## 4. Build and Run

```bash
cd ~/ROS_workspaces/adi_ros_dev
colcon build --packages-select multicam_calibration
source install/setup.bash

ros2 launch multicam_calibration calibration.launch.py
```

Hold the L-shape target so:
- Camera1 (D455) sees Board A clearly
- Camera2 (D435i) sees Board B clearly
- Both boards are fully visible (no partial occlusion)
- Move the target to slightly different positions/angles between samples
  for better averaging

The node logs each sample's translation. After collecting the configured
number of samples (default: 30), it prints the final result:

```
CALIBRATION RESULT (camera1 -> camera2)
Translation (xyz): [x, y, z]
Rotation    (rpy): [roll, pitch, yaw]

URDF joint (paste into your xacro):
  <origin xyz="..." rpy="..."/>
```

The result is also saved to `/tmp/calibration_result.yaml`.

## 5. Apply the Result

Paste the URDF `<origin>` line into your camera2 joint in
`ad_r1m_perception_cuvslam/urdf/realsense_calibration.urdf.xacro`:

```xml
<joint name="camera2" type="fixed">
  <parent link="base_link"/>
  <child link="camera2_link"/>
  <origin xyz="NEW_X NEW_Y NEW_Z" rpy="NEW_ROLL NEW_PITCH NEW_YAW"/>
</joint>
```

## 6. Quality Checks

- **Translation std < 3mm**: Good calibration.
- **Translation std 3-10mm**: Acceptable, but check board rigidity.
- **Translation std > 10mm**: Problem — board flexing, bad detection, or
  wrong L-shape measurements. Fix and re-run.
- After applying the result, run cuVSLAM and check for visual
  discontinuities at the camera boundary.
