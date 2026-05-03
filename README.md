# Sensorless Bilateral Force Feedback for Low-Cost Scaled Teleoperation

A low-cost bimanual teleoperation platform that gives the operator bilateral force feedback without dedicated force sensors. Two commodity leader arms control two custom 2x-scaled 3D-printed follower arms; contact resistance is estimated from Dynamixel motor current and reflected back to the operator.

**[Preprint PDF](sensorless-bilateral-teleop.pdf)** | **[DOI](https://doi.org/10.13140/RG.2.2.31526.28487)** | **[Video](https://youtu.be/PqzKTUld3uw)** | **[Project page](https://leocamacho.co/teleop)**

![The teleoperation system stacking cups with sensorless force feedback](media/teleop-cups.gif)

## Key Results

- **91.1% obstacle detection sensitivity** via sensorless force feedback (n=22 user study)
- **~$3,000 total cost** for the complete dual-pair system (10x cheaper than ALOHA)
- **55% motor cost savings** via torque-tiered actuator selection
- **7/7 median learnability** rating from novice users

## Why It Matters

Low-cost imitation-learning systems such as ALOHA, GELLO, and the Koch arm made teleoperation cheaper, but most affordable platforms still lack haptic feedback. This project explores a narrower question: how far can bilateral feedback go if the only force signal available is motor current from commodity servos?

The answer is useful but bounded. Motor current is strong enough for operators to feel contact and detect hidden obstacles, but it cannot reliably distinguish gravity load from external contact without a dynamics model. The repository includes the ROS 2 control stack, 3D-printable follower arm links, the preprint, and the scripts used to create the README demo media.

## System Overview

| Component | Specification |
|-----------|---------------|
| Leaders (x2) | PincherX-100, all XL430-W250 motors, PWM mode (backdrivable) |
| Followers (x2) | Custom 2x-scaled, XM430-W210 (shoulder/elbow) + XL430-W250 (waist/wrist/gripper), position mode |
| Force feedback | Motor current as force proxy, 300 mA threshold, proportional PWM resistance |
| Software | ROS 2 Humble, 20 nodes across 4 namespaces, 100 Hz joint state publishing |
| Safety | Graceful shutdown (POSIX signal masking), smooth startup, gripper PWM limiting, invisible wall collision avoidance |

## Repository Structure

```
├── paper/                          # Preprint and figures
│   ├── paper.tex                   # LaTeX source
│   ├── paper.pdf                   # Compiled paper
│   ├── references.bib              # Bibliography
│   └── figures/                    # Paper figures
│
├── media/
│   └── teleop-cups.gif             # README demo GIF
│
├── scripts/
│   └── make_readme_gif.py          # Frame-range GIF generation helper
│
├── dissertation_ws/                # ROS 2 workspace
│   └── src/teleop_controller/
│       ├── teleop_controller/      # Python source
│       │   ├── teleop_node.py      # Joint-space teleoperation + collision avoidance
│       │   ├── force_feedback_node.py  # Sensorless bilateral force feedback
│       │   ├── motion_node.py      # Shared autonomy (RViz interactive markers)
│       │   └── effort_graph_node.py    # Real-time effort visualisation
│       ├── config/                 # Motor configs, teleop params, RViz config
│       ├── launch/                 # dual_arm_bringup.launch.py
│       └── urdf/                   # Scaled follower robot description
│
├── 3d-printing/                    # 3D printing files
│   ├── modified_arm/               # Custom 2x-scaled STLs (final versions)
│   │   ├── left-bicep.stl
│   │   ├── right-bicep.stl
│   │   ├── final-final-xl-forearm.stl
│   │   ├── left-final-gripper.stl
│   │   ├── right-final-gripper.stl
│   │   ├── left-finger-hold.stl
│   │   └── right-finger-hold.stl
│   └── px100-meshes/               # Original PincherX-100 reference meshes
│
└── identify_arms.py                # USB serial port identification utility
```

## Hardware Requirements

- 2x PincherX-100 Robot Arms (leaders)
- 4x XM430-W210 motors (follower shoulder + elbow)
- 6x XL430-W250 motors (follower waist + wrist + gripper)
- 4x U2D2 USB-to-serial adapters
- 12V power supply
- 3D-printed follower arm links (STLs in `3d-printing/modified_arm/`)

## Quick Start

### Prerequisites
- Ubuntu 22.04 with ROS 2 Humble
- [Interbotix XS SDK](https://docs.trossenrobotics.com/interbotix_xsarms_docs/)

### Set up USB latency and udev rules
```bash
python3 identify_arms.py
```

### Build and run
```bash
cd dissertation_ws
colcon build --packages-select teleop_controller
source install/setup.bash
ros2 launch teleop_controller dual_arm_bringup.launch.py
```

### Debug
```bash
ros2 node list
ros2 topic echo /leader_left/joint_states
ros2 topic echo /follower_left/joint_states
```

## Configuration

Teleoperation parameters are in `dissertation_ws/src/teleop_controller/config/teleop_config.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `effort_threshold` | 300 mA | Effort below this is filtered |
| `pwm_scale` | 0.5 | Proportional gain for force feedback |
| `max_pwm` | 350 | Safety cap on leader resistance |
| `smoothing_alpha` | 0.15 | Exponential smoothing factor |
| `wall_threshold` | 0.0 m | Y-position of invisible wall |

## README Media

The demo GIF is generated from the full teleoperation video using a frame range:

```bash
python3 scripts/make_readme_gif.py /path/to/teleop_all.mp4 media/teleop-cups.gif \
  --start-frame 3000 \
  --end-frame 3180
```

## Citation

```bibtex
@misc{camacho2026sensorless,
  title={Sensorless Bilateral Force Feedback for Low-Cost Scaled Teleoperation},
  author={Camacho Pel\'aez, Leonardo},
  year={2026},
  doi={10.13140/RG.2.2.31526.28487},
  url={https://doi.org/10.13140/RG.2.2.31526.28487}
}
```

## License

This project is open-source. Hardware designs (STL files) and software are provided for research and educational use.
