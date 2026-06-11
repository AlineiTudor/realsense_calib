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

For an L-shape where Board B is attached at the bottom edge of Board A,
forming a 90° angle (Board B extends perpendicular, towards camera2):

```
  Side view of L-shape:

       Board A (vertical)
       |
       |  ← camera1 sees this face
       |
       +------------------
         Board B (horizontal)

         camera2 sees this face ↑
```

**translation_y**: Distance from Board A's origin to the hinge line (the
edge where the two boards meet), measured along Board A's Y axis. For a
7x5 board with 39.5mm squares, the full board height is 5 × 39.5mm =
197.5mm. If Board B is flush with Board A's bottom edge, translation_y
= 0.1975 m. If there's extra spacing, add it.

**translation_x**: Lateral offset between the two board origins, measured
along Board A's X axis. If both boards are centered on the hinge line,
this is likely 0. If Board B is shifted left or right, measure the offset.

**translation_z**: Depth offset from Board A's surface plane to Board B's
origin. For a simple L-bracket where the hinge is at Board A's bottom
edge, this is approximately the board thickness (a few mm). Often
negligible — set to 0 if boards meet at the edge cleanly.

**angle_deg**: The angle between the two board surfaces. Use a digital
angle gauge placed across both surfaces. Nominally 90°.

### Filling in the config

Edit `config/calibration_params.yaml`:

```yaml
l_shape:
  translation_x: 0.0       # lateral offset (meters)
  translation_y: 0.1975    # distance to hinge along board A's Y (meters)
  translation_z: 0.0       # depth offset (meters)
  angle_deg: 90.0          # measured angle between faces
```

### Tips for accurate measurement
- Use calipers rather than a tape measure.
- Mark the board origin corner on the physical board with a dot.
- Measure from dot to dot if possible.
- The angle matters less than the translation for your use case (you said
  translation is more important).
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
