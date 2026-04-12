# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**ba_depth_node** is the perception component of a 6DOF robot arm control pipeline ("Bracket-Arm"). It runs Depth Anything V2 (Small) on a WSL2 laptop (CPU only) to produce depth maps from camera frames. The depth output feeds into MoveIt for grasp planning.

The architecture is **trigger-based** ("snapshot mode"): on demand, grab one frame, compute depth (~0.9s on CPU), extract target, send MoveIt goal. This is not a continuous depth stream.

## End-to-End Pipeline

```
USB-Kamera (Pi, ubuntu1)
    ↓
usb_cam Node → /camera/image_raw/compressed (ROS2, JPEG over WLAN)
    ↓
Laptop (WSL2):
    ├──→ Depth Anything V2 (Small) → depth map (HxW float)
    └──→ VLM (Groq/Llama 4 Scout or Gemini) → target pixel (u, v)
                    ↓
          depth_map[v, u] → Z
          (u - cx) * Z / fx → X    (backprojection via camera intrinsics)
          (v - cy) * Z / fy → Y
                    ↓
          Hand-Eye Transform → robot coordinates
                    ↓
                 MoveIt → Pi Bridge → Arduino → Servos
```

Overview camera: mounted ~60-80 cm above workspace, ~45° tilt, 30-40 cm lateral offset from arm base. Optional gripper camera for visual servoing in the last ~10 cm (later phase).

## Related Projects

- `ba_arduino_controller` — Arduino-side arm control
- `ba_arduino_controller_moveit_config` — MoveIt configuration
- `ba_camera_bridge` — ROS2 USB camera node on Raspberry Pi (publishes `/ba_overview_camera/image_raw/compressed`)

## Package Structure

This is a ROS2 `ament_python` package. The main node is `depth_estimator_node`.

- `ba_depth_node/depth_estimator_node.py` — ROS2 node: subscribes compressed image + camera_info, rectifies, runs DA V2, publishes depth
- `config/depth_estimator.yaml` — parameters (topics, model_id, device, rectify flag)
- `launch/depth_estimator.launch.py` — launch file

### Topic Contract

| Topic | Type | Direction |
|---|---|---|
| `/ba_overview_camera/image_raw/compressed` | `CompressedImage` | subscribe |
| `/ba_overview_camera/camera_info` | `CameraInfo` | subscribe (for rectification LUT) |
| `/ba_overview_camera/depth/image_raw` | `Image` (32FC1, 0.0–1.0) | publish |

## Environment

- **Host**: WSL2 Ubuntu-22.04 on Windows dev laptop (no dedicated GPU — CPU-only inference)
- **Python venv**: `~/venvs/ba_depth_node/` — must be created with `--system-site-packages` so it sees `rclpy` from `/opt/ros/humble`
- **Key deps**: PyTorch CPU (`torch`, `torchvision` from `https://download.pytorch.org/whl/cpu`), `transformers`, `pillow`, `opencv-python`
- **Note**: `cv_bridge` from ROS2 Humble is compiled against numpy 1.x and incompatible with numpy 2.x (needed by torch). We avoid cv_bridge and construct Image messages manually instead.
- **Model**: `depth-anything/Depth-Anything-V2-Small-hf` via HuggingFace `transformers` pipeline (~100 MB, cached in `~/.cache/huggingface/hub/`)
- **Raspberry Pi server**: `ubuntu@ubuntu1` — use the `ubuntu1-ssh-login` skill to connect

## Build & Run (WSL2)

```bash
# venv setup (once) — must use --system-site-packages for rclpy
python3 -m venv --system-site-packages ~/venvs/ba_depth_node
source ~/venvs/ba_depth_node/bin/activate
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install transformers pillow opencv-python
pip install --ignore-installed scipy  # override old system scipy

# ROS2 environment
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/$USER/cyclonedds.xml

# Build (symlink-install so edits take effect without rebuild)
cd ~/ros2_ws
colcon build --packages-select ba_depth_node --symlink-install
source install/setup.bash

# Run — the launch file uses `prefix` to invoke the venv Python,
# so torch/transformers are on sys.path even without activating the venv.
ros2 launch ba_depth_node depth_estimator.launch.py
```

### Verify

CycloneDDS on WSL2 loopback cannot transport large (~1.2 MB) messages between
local processes — `ros2 topic hz` and rviz2 will not show the depth topic.
Instead, the node saves a debug PNG on every frame:

```bash
# From Windows Explorer:
\\wsl.localhost\Ubuntu-22.04\tmp\depth_latest.png

# Or copy to Windows:
cp /tmp/depth_latest.png /mnt/c/Users/thomas/Desktop/depth_latest.png
```

Disable debug saving by setting `debug_save_path: ""` in the config YAML.

## ROS2 Configuration

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/$USER/cyclonedds.xml
```

## Performance Baseline

DA V2 Small on CPU (6 threads): ~861 ms/frame at 640x480 (~1.16 FPS). Inference time is nearly independent of input resolution due to the model's fixed internal size (~518px).
