# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**ba_depth_node** is the perception component of a 6DOF robot arm control pipeline ("Bracket-Arm"). It runs Depth Anything V2 (Small) on a WSL2 laptop (CPU only) to produce depth maps from camera frames. The depth output feeds into MoveIt for grasp planning.

The architecture is **trigger-based** ("snapshot mode"): on demand, grab one frame, compute depth (~0.9s on CPU), extract target, send MoveIt goal. This is not a continuous depth stream.

## Related Projects

- `ba_arduino_controller` — Arduino-side arm control
- `ba_arduino_controller_moveit_config` — MoveIt configuration
- `ba_camera_bridge` — ROS2 USB camera node on Raspberry Pi (publishes `/ba_overview_camera/image_raw/compressed`)

## Environment

- **Host**: WSL2 Ubuntu-22.04 on Windows dev laptop (no dedicated GPU — CPU-only inference)
- **Python venv**: `~/venvs/ba_depth_node/` (lives outside the repo to avoid conflicts with future `colcon build`)
- **Activate**: `source ~/venvs/ba_depth_node/bin/activate`
- **Key deps**: PyTorch CPU (`torch`, `torchvision` from `https://download.pytorch.org/whl/cpu`), `transformers`, `pillow`, `opencv-python`
- **Model**: `depth-anything/Depth-Anything-V2-Small-hf` via HuggingFace `transformers` pipeline (~100 MB, cached in `~/.cache/huggingface/hub/`)
- **Raspberry Pi server**: `ubuntu@ubuntu1` — use the `ubuntu1-ssh-login` skill to connect

## ROS2 Configuration

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/$USER/cyclonedds.xml
```

Camera topic: `/ba_overview_camera/image_raw/compressed` (JPEG via `image_transport`)

## Project Status

Phase 1 (standalone depth estimation smoke test) is complete. Phase 2 (ROS2 node integration) is next. Key open decisions for Phase 2:
- venv vs system-site-packages for `rclpy` compatibility
- Rectification pipeline placement (needed for pixel-to-3D backprojection)
- Metrische depth scaling approach (ArUco markers vs reference sensor)

## Performance Baseline

DA V2 Small on CPU (6 threads): ~861 ms/frame at 640x480 (~1.16 FPS). Inference time is nearly independent of input resolution due to the model's fixed internal size (~518px).
