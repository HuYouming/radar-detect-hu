# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is the **RoboMaster 2026 Radar Station** software for HUST (Huazhong University of Science and Technology). A radar station is a fixed elevated robot that uses a wide-angle industrial camera and LiDAR to detect and track all robots on the competition field, localize them in 3D world coordinates, and broadcast their positions to allied robots over serial.

## Runtime Environment

**Two-repo dependency**: This repo (`Hust_Radar_2026`) imports `Log.Log` from the sibling repo `Hust_Radar_2025`. All other core modules (`communication/`, `detect/`, `Lidar/`, `Car/`) live in this repo. The config files (`./configs/*.yaml`) are also in this repo and resolved relative to `Hust_Radar_2026/`.

**Correct run directory**: Always run from `Hust_Radar_2026/`. Set `PYTHONPATH` to include `Hust_Radar_2025` for `Log.Log`:
```bash
export PYTHONPATH=/home/py/Hust_Radar_2025:$PYTHONPATH
```

**Environment**: `conda activate Radar`, then `source /opt/ros/noetic/setup.bash` before running anything.

## Running the System

**Full competition launch** (opens 6 gnome-terminal windows):
```bash
cd /home/py/Hust_Radar_2026
bash 26main.sh
```
Terminals: roscore → Livox SDK → Drone Search (`Counter/init_angle_sender.py`) → UDP Receiver → Radio UDP (`Radio/field_info_publisher.py`) → Main (`26_main.py`).

**Manual single-process run**:
```bash
cd /home/py/Hust_Radar_2026
export PYTHONPATH=/home/py/Hust_Radar_2025:$PYTHONPATH
source /opt/ros/noetic/setup.bash
python3 26_main.py
```

**Entry points**:
- `26_main.py` — 2026 season, vision-only localization (primary). Top-of-file `mode = "camera"` / `"video"` controls live camera vs. offline video playback.
- `25_main.py` — 2025 season, full LiDAR+vision fusion
- `25main_without_lidar.py` — 2025 season, vision-only fallback

**Debug / diagnosis tool**:
```bash
python3 debug_detector_pipeline.py --mode video --video-path /path/to/video.mp4 --skip-messager
```
Monkey-patches timing hooks onto `Detector`, `Capture`, `Converter`, and `Messager`; runs sync probe → async probe → mini main loop, logging every stage to `./debug_logs/`. Use `--skip-converter` to bypass the interactive point-selection step.

**Camera calibration GUI** (PyQt5, requires display):
```bash
python3 camera_locator/calib.py
```
Pick matching pixel ↔ world point pairs on the camera view and field map; supports multi-height-plane calibration; saves homography matrices as `.npy` files.

**ROS-only startup** (LiDAR stack only):
```bash
bash start_ros.sh  # starts roscore, livox_ros_driver, pclmatcher
```

## Architecture

### Thread Model

Three concurrent subsystems sharing a central data store (`CarList`):

```
[Hikrobot Camera / Video file]
       │
       ▼
[detect/Detector.py] ── daemon thread ──► YOLO stage1 track (ByteTrack) → stage2 classify armor
       │  _results (written by detector, read by main)
       ▼
[26_main.py main loop] ~20 fps
       ├── filter by team color (my_color from config)
       ├── get_new_box() → shift detection to chassis bottom
       ├── Lidar/Converter.detection_main() → world XYZ via Vision_Locator
       ├── Car/CarList.update_car_info() → per-car state with lifespan decay
       └── communication/Messager → serial send to allied robots + sentinel alerts

[Lidar/Lidar_25.py] ── ROS spin thread ──► /centroid_points subscriber → centroidsQueue
       │  (only used in 25_main.py)
       ▼
       └── get_all_pc() → merged point cloud → Converter.detection_main(point_cloud=pc_all)
```

### Key Modules

- **`detect/Detector.py`** — Two-stage YOLO inference in a daemon thread. Stage 1: ByteTrack detection at 4024×3036. Stage 2: ROI crop to classify robot label (R1-R5/R7, B1-B5/B7). Vote accumulation per track ID stabilizes classification across frames.
- **`detect/Capture.py`** — Hikrobot MV USB3 industrial camera driver (MvImport SDK).
- **`detect/Video.py`** — Drop-in `VideoCapture` replacement for offline testing with recorded video.
- **`Lidar/Converter.py`** — Core coordinate transformation: camera intrinsics + extrinsics → world XYZ. Calls `Vision_Locator` when no LiDAR is available.
- **`Lidar/vision_locator.py`** — Perspective transform-based depth estimation using known field landmarks. Pre-computes homography matrices per height plane from `rm25_points.yaml`.
- **`Lidar/PointCloud.py`** — Ring-buffer point cloud queue; DBSCAN clustering via open3d to extract robot centroids.
- **`Lidar/fast_search.py`** — GPU-accelerated (CuPy/Torch) spatial search for matching point cloud to 2D bounding boxes.
- **`Car/Car.py`** — Per-robot state machine (`Car`) and thread-safe collection (`CarList`, 12 robots). Tracks field XYZ, trust flag, and lifespan countdown for stale detections.
- **`communication/Messager.py`** — Central communication hub running in its own thread (~5 fps). Subscribes to ROS topics (`/drone_field_xyz`, `/radar/enemy/jam_key`, `/radar/enemy/health_array`); sends mini-map positions, sentinel alert angles, enemy HP, and double-effect decisions over serial. Stale mini-map positions are sent as zero coordinates after life expires.
- **`communication/Sender.py`** / **`Receiver.py`** — Low-level serial frame encode/decode.
- **`Counter/init_angle_sender.py`** — Aerial drone detection and tracking. Processes LiDAR against a pre-loaded map PCD to detect the drone, computes yaw/pitch for the sentry cannon, sends over serial.
- **`Radio/field_info_publisher.py`** — Decodes referee-system UDP broadcast and republishes relevant fields (health, marks, dart target) as ROS topics consumed by `Messager`.
- **`Radio/radar_udp_receiver.py`** — Raw referee system UDP decode (binary protocol with CRC8).
- **`Radio/interferance_level_sender.py`** — Sends interference-level selection via UDP to the radio board.
- **`PointTracker/Tracker.py`** — 2D constant-velocity Kalman filter tracker for associating point cloud detections in field coordinates across frames.
- **`camera_locator/`** — GUI calibration toolkit (PyQt5 + OpenCV). `calib.py` is the main app; `anchor.py` manages selected landmark points; `point_picker.py` handles zoom/pan/click interaction on high-res images.
- **`Watcher/`** — Standalone serial monitor utilities for observing raw incoming frames during debugging.
- **`main_utilities.py`** / **`draw_minimap_from_log.py`** — `get_new_box()` shifts detection to chassis bottom; `visualize()` renders a top-down field map.

### Configuration Files (`./configs/`)

| File | Key settings |
|---|---|
| `main_config.yaml` | `global.my_color` (`"Red"` / `"Blue"`), `global.is_debug`, `car.life_span`, `communication.port` (`/dev/ttyUSB0`), baud rate, `area.*` polygon zones for hero alert |
| `detector_config.yaml` | YOLO model paths (update these to match actual `weights/` files), confidence thresholds, vote `life_time` |
| `converter_config.yaml` | Camera intrinsics (fx, fy, cx, cy), extrinsic R+T, DBSCAN params |
| `bin_cam_config.yaml` | Camera hardware: 4024×3036, exposure, gain, serial numbers |

Model weights are in `weights/` (`new_stage1.pt`, `stage3.pt`). The paths in `detector_config.yaml` must be kept in sync manually.

### Field Coordinate System

28 m × 15 m. Origin (0, 0) at Red team's left-bottom corner (Red base side). Blue origin is mirrored: (28, 15). Car IDs: Red = 1–5, 7 (sentinel); Blue = 101–105, 107. Drone = index 4 (ID 6/106) in the `send_map_infos` array.

## Key External Dependencies

- **ROS Noetic** — LiDAR topics, multi-node coordination, inter-process data bus between `field_info_publisher` and `Messager`
- **ultralytics (YOLOv8)** — detection and tracking
- **open3d** — DBSCAN clustering, PCD file loading
- **cupy / torch** — GPU point cloud search (NVIDIA GPU required)
- **Hikrobot MvImport SDK** — camera driver (in `stereo_camera/MvImport/`)
- **Livox Mid-70** — solid-state LiDAR via `livox_ros_driver`
- **PyQt5** — `camera_locator/calib.py` calibration GUI
- **shapely** — polygon containment checks in hero-alert zone logic
- Serial port `/dev/ttyUSB0` — robot-to-robot communication (wrapped in try/except so system runs without it)

## Season Versioning Convention

Files explicitly named `_25` or `_26` indicate which competition season they belong to (2025 or 2026). The 2026 work is primarily in `26_main.py` and `Counter/init_angle_sender.py`; legacy 2025 code is in `25_main.py` and `Lidar/Lidar_25.py`.
