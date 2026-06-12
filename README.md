# multicam_calibration

Extrinsic calibration between two RealSense cameras using the **hand-eye (AX=XB)** method.
No manual geometry measurements required.

Two ChArUco boards are placed in the scene (not rigidly connected). The camera
rig is moved to multiple distinct positions while both boards remain visible.
The solver recovers the camera-to-camera transform automatically.

## How it works

At each rig position *i*, each camera detects its ChArUco board via `solvePnP`:

- Camera 1 sees Board A: `T_cam1_boardA_i`
- Camera 2 sees Board B: `T_cam2_boardB_i`

Between any two rig positions *i* and *j*, the rigid camera mounting gives:

```
A_ij @ X = X @ B_ij
```

where `A_ij` and `B_ij` are the relative motions of each camera, and
`X = T_cam1_cam2` is the unknown transform. OpenCV's `calibrateHandEye()`
solves this system using five independent methods (TSAI, PARK, HORAUD,
ANDREFF, DANIILIDIS) for cross-validation.

## Dependencies

```bash
pip3 install opencv-contrib-python numpy scipy pyyaml
```

If `opencv-python` is already installed, replace it — `opencv-contrib-python`
is a superset that includes `cv2.aruco`:

```bash
pip3 uninstall opencv-python && pip3 install opencv-contrib-python
```

## Package structure

```
multicam_calibration/
├── config/
│   ├── calibration_params.yaml      # Board specs, topics, solver params
│   └── realsense_calibration.yaml   # RealSense camera configuration
├── launch/
│   └── calibration.launch.py        # Launches cameras + calibration node
├── multicam_calibration/
│   ├── calibration_node.py          # ROS 2 calibration node
│   └── generate_boards.py          # Generate printable ChArUco board images
├── CMakeLists.txt
└── package.xml
```

## Setup

### 1. Prepare ChArUco boards

Generate printable board images:

```bash
python3 -m multicam_calibration.generate_boards --output_dir /tmp/boards
```

Or use existing boards. The default config expects:

| Parameter | Value |
|-----------|-------|
| Dictionary | DICT_4X4_50 |
| Grid | 7 x 5 |
| Square size | 39.5 mm |
| Marker size | 30.0 mm |

Mount each board on a **rigid flat surface** (foam board, MDF, acrylic). Any
warping degrades accuracy. Verify the printed square size with a ruler and
adjust `square_length` in `config/calibration_params.yaml` if it differs.

### 2. Configure cameras

Edit `config/realsense_calibration.yaml`:

- Set the `serial_no` for each camera (find with `rs-enumerate-devices`).
- Choose the stream type. **Infra is recommended** — it is lighter on CPU
  and avoids RGB debayering overhead that can cause system load issues on
  embedded platforms.

```yaml
cameras:
  - camera_name: camera2
    serial_no: '<YOUR_CAMERA2_SERIAL>'    # e.g., D455
    depth_module.inter_cam_sync_mode: 0

  - camera_name: camera1
    serial_no: '<YOUR_CAMERA1_SERIAL>'    # e.g., D435i
    depth_module.inter_cam_sync_mode: 0
```

To switch between infra and RGB streams, toggle the stream flags in
`common_params`:

| Setting | Infra (default) | RGB |
|---------|----------------|-----|
| `enable_infra1` | `True` | `False` |
| `enable_color` | `False` | `True` |

When using infra streams, the IR emitter **must** be disabled
(`depth_module.emitter_enabled: 0`) to avoid projected patterns interfering
with ChArUco detection.

Then set the matching topics in `config/calibration_params.yaml`:

| Stream | Image topic | Info topic |
|--------|-------------|------------|
| Infra | `/cameraX/infra1/image_rect_raw` | `/cameraX/infra1/camera_info` |
| RGB | `/cameraX/color/image_raw` | `/cameraX/color/camera_info` |

### 3. Build

```bash
colcon build --packages-select multicam_calibration
source install/setup.bash
```

## Usage

### Scene setup

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
           \____________________________/
                 Camera rig (moves)
```

- Place each board where its corresponding camera can see it.
- The boards must remain **stationary** throughout calibration.
- The boards do **not** need to be attached to each other — any distance and
  angle between them is fine.

### Run

```bash
ros2 launch multicam_calibration calibration.launch.py
```

### Capture procedure

1. Position the camera rig so both cameras see their respective boards.
2. Hold still — the node captures the pose pair automatically.
3. **Move the camera rig** to a new position (translate **and** rotate it).
4. Repeat until the required number of poses is collected (default: 15).
5. The node computes and prints the result automatically.

**The node requires both translation (>= 5 cm) and rotation (>= 15 deg) between
consecutive captures**, with a 2-second cooldown. This ensures diverse poses
for an accurate solution.

### Tips for good results

- **Large rotations matter most.** Tilt and rotate the rig 20-30 degrees
  between positions. Sliding sideways alone is not enough — the solver needs
  rotational diversity.
- Keep both boards fully visible (no partial occlusion).
- Avoid extreme viewing angles (board nearly edge-on to camera).
- For higher precision, increase `min_samples` to 25-30.
- Ensure even lighting with no strong shadows or glare on the boards.

## Output

The node prints results from all five solver methods in both optical and
camera_link frames:

```
CAMERA_LINK FRAME results (X forward, Y left, Z up):
  TSAI          t=[-0.074308, -0.052121, 0.030563]  rpy=[0.0295, 0.0193, -1.4954]
  PARK          t=[-0.071515, -0.051905, 0.029778]  rpy=[0.0315, 0.0175, -1.5018]
  HORAUD        t=[-0.071498, -0.051904, 0.029769]  rpy=[0.0315, 0.0176, -1.5020]
  ANDREFF       t=[-0.046353, -0.066534, 0.048527]  rpy=[0.0315, 0.0170, -1.5022]
  DANIILIDIS    t=[-0.069726, -0.051310, 0.028124]  rpy=[0.0328, 0.0165, -1.5032]

URDF joint (paste into your xacro):
  <origin xyz="-0.071515 -0.051905 0.029778" rpy="0.031525 0.017518 -1.501797"/>
```

**PARK** is used as the primary method. When multiple methods converge to
similar values, the calibration is reliable. The result is saved to
`/tmp/calibration_result.yaml` (configurable via `output_file`).

### Consistency check

The node also reports the board-to-board transform standard deviation across
all captured poses. Since both boards are stationary, this reconstructed
transform should be constant — its spread measures calibration quality:

| Translation std | Quality |
|-----------------|---------|
| < 3 mm | Excellent |
| 3–10 mm | Acceptable — consider more poses or more diverse rotations |
| > 10 mm | Poor — check board flatness, lighting, board stability |

### Applying the result

Paste the URDF `<origin>` line into the camera2 fixed joint in your xacro:

```xml
<joint name="camera2_joint" type="fixed">
  <parent link="camera1_link"/>
  <child link="camera2_link"/>
  <origin xyz="..." rpy="..."/>
</joint>
```

After applying, verify by running your visual SLAM pipeline and checking for
drift or discontinuities at the camera boundary.

## Configuration reference

### `config/calibration_params.yaml`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `board_a.dictionary` | `DICT_4X4_50` | ArUco dictionary name |
| `board_a.squares_x` | `7` | Board columns |
| `board_a.squares_y` | `5` | Board rows |
| `board_a.square_length` | `0.0395` | Square side length (m) |
| `board_a.marker_length` | `0.030` | Marker side length (m) |
| `calibration.min_samples` | `15` | Rig positions to collect |
| `calibration.min_translation_m` | `0.05` | Min translation between poses (m) |
| `calibration.min_rotation_deg` | `15.0` | Min rotation between poses (deg) |
| `calibration.capture_cooldown_s` | `2.0` | Seconds between captures |
| `calibration.process_rate_hz` | `2.0` | Frame processing rate (Hz) |
| `calibration.camera*_image_topic` | — | Image topic per camera |
| `calibration.camera*_info_topic` | — | CameraInfo topic per camera |
| `calibration.output_file` | `/tmp/calibration_result.yaml` | Output path |

### `config/realsense_calibration.yaml`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_infra1` | `True` | Enable infrared stream |
| `enable_color` | `False` | Enable RGB stream |
| `depth_module.infra_profile` | `640,480,15` | Infra resolution and FPS |
| `rgb_camera.profile` | `640,480,15` | RGB resolution and FPS |
| `depth_module.emitter_enabled` | `0` | IR emitter (must be off for infra) |
| `cameras[].serial_no` | — | RealSense device serial number |
| `cameras[].inter_cam_sync_mode` | `0` | Hardware sync (0=off, 1=master, 2=slave) |

## Architecture notes

The calibration node subscribes with **BEST_EFFORT QoS** and **depth=1**,
and processes frames via a timer at `process_rate_hz` (default 2 Hz). This
design prevents CPU saturation — the DDS middleware drops frames the
subscriber cannot consume, rather than buffering and serializing every one.
This is important because the calibration node runs as a separate Python
process (not a composable node), so every image crosses a process boundary
via DDS serialization.

## Reference setup

This package was developed and tested with:

- **NVIDIA Jetson AGX Orin** (JetPack / Isaac ROS)
- **Intel RealSense D435i** (camera1 / base_link) + **D455** (camera2)
- Cameras mounted at ~90 degrees with no overlapping field of view
- ROS 2 Humble
