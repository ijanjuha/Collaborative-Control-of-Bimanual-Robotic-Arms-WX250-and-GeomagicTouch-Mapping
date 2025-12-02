# 🤖 Bilateral Teleoperation System
### Collaborative Control of Bimanual Robotic Arms: WX250 & Geomagic Touch Mapping

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10+-green.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

<p align="center">
  <img src="https://img.shields.io/badge/Status-Active-success" alt="Status">
  <img src="https://img.shields.io/badge/Feedback-100Hz-informational" alt="Feedback Rate">
  <img src="https://img.shields.io/badge/Latency-10ms-critical" alt="Latency">
</p>

---

## 📋 Overview

A high-performance **bilateral teleoperation system** enabling intuitive control of an **Interbotix WX250 robotic arm** **(WidowX 250)** using a **Geomagic Touch haptic device**. The system provides realistic force feedback through advanced gravity compensation and velocity damping, creating an immersive robotic manipulation experience.

### Key Features

- **Precise Position Control**: Real-time teleoperation at 1000 Hz with <10ms latency
- **Realistic Force Feedback**: Gravity compensation with configurable torque-to-force mapping
- **Bidirectional Communication**: Simultaneous position commands and force feedback
- **High-Frequency Updates**: 100 Hz force feedback synchronized with robot joint states
- **Safety Limits**: Hardware-constrained force limits (±3.3N) and workspace boundaries
- **Tunable Parameters**: Easily adjustable gravity scaling and damping coefficients

---

## System Architecture

```
┌─────────────────────┐         ┌──────────────────────┐         ┌─────────────────────┐
│  Geomagic Touch     │────────▶│   ROS2 Pipeline      │────────▶│  Interbotix WX250   │
│  Haptic Device      │  1kHz   │   (MoveIt Servo)     │   ~50Hz │   Robotic Arm       │
│  (Position Input)   │         │                      │         │                     │
└─────────────────────┘         └──────────────────────┘         └─────────────────────┘
         ▲                               │                                │
         │                               │                                │
         │         ┌─────────────────────┴────────────────────┐          │
         │         │  Gravity Compensation & Damping Node     │          │
         └─────────│  • Computes joint torques                │◀─────────┘
           100Hz   │  • Maps to Cartesian forces              │   ~100Hz
                   │  • Applies velocity damping              │
                   └──────────────────────────────────────────┘
```

###  Node Pipeline

1. **`geomagic_servo.py`**: Captures Geomagic Touch position → Publishes Twist commands (1000 Hz)
2. **`custom_servo_node.py`**: MoveIt Servo node → Computes joint velocities
3. **`servo_to_interbotix_bridge.py`**: Converts velocities → Robot commands
4. **`robot_to_geomagic_feedback.py`**: Computes gravity torques → Generates force feedback (100 Hz)

---

## Getting Started

### Prerequisites

```bash
# System Requirements
- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10+
- MoveIt2

# Hardware
- Interbotix WX250 Robotic Arm
- Geomagic Touch Haptic Device
```

### Installation

1. **Clone the repository**
```bash
cd ~/
git clone https://github.com/ijanjuha/Collaborative-Control-of-Bimanual-Robotic-Arms-WX250-and-GeomagicTouch-Mapping.git
cd Collaborative-Control-of-Bimanual-Robotic-Arms-WX250-and-GeomagicTouch-Mapping
```

2. **Install dependencies**
```bash
# Install ROS2 Humble (if not already installed)
sudo apt update
sudo apt install ros-humble-desktop-full

# Install MoveIt2
sudo apt install ros-humble-moveit

# Install Interbotix packages
sudo apt install ros-humble-interbotix-*

# Install Geomagic Touch drivers
sudo apt install ros-humble-omni-msgs
```

3. **Build the workspace**
```bash
cd ~/interbotix_ws
colcon build --symlink-install
source install/setup.bash
```

4. **Configure device permissions**
```bash
# For Geomagic Touch (may require udev rules)
sudo usermod -aG dialout $USER
# Logout and login for changes to take effect
```

---

## Usage

### Basic Launch

```bash
# Launch with real hardware
ros2 launch ~/interbotix_ws/src/wx250_bringup/launch/wx250_geomagic_allinone.launch.py use_sim:=false rviz:=true

# Launch with simulation
ros2 launch ~/interbotix_ws/src/wx250_bringup/launch/wx250_geomagic_allinone.launch.py use_sim:=true rviz:=true
```

### Adjustable Parameters

Edit the launch file to customize behavior:

```python
# Force Feedback Tuning
'gravity_force_scale': 0.5,  # Range: 0.3-0.7 (intensity of gravity feedback)
'damping': 1.0,              # Range: 0.5-1.5 (movement resistance)
'max_force': 3.3,            # Hardware limit (Newtons)

# Workspace Scaling
'scale': 2.0,                # Position scaling factor (robot:haptic)
```

---

## Verification & Debugging

### Check System Health

```bash
# Verify force feedback rate
ros2 topic hz /phantom/force_feedback
# Expected: ~100 Hz

# Monitor force values
ros2 topic echo /phantom/force_feedback --field force
# Expected: Changing values based on robot pose

# Check gravity torques
ros2 topic echo /robot/gravity_torques
# Expected: [0.0, 6-8, 1-2, 0.37, 0.0] (Shoulder, Elbow, Wrist)

# View robot joint states
ros2 topic echo /joint_states
```

### Common Issues

| Issue | Solution |
|-------|----------|
| No force feedback | Check if `robot_to_geomagic_feedback.py` is running |
| Robot not moving | Verify `/servo_node/delta_twist_cmds` topic is publishing |
| Inverted axes | Modify transformation matrix in `geomagic_servo.py` |
| Weak force feedback | Increase `gravity_force_scale` parameter |
| Jerky motion | Reduce `damping` coefficient |

---

##  Performance Metrics

| Metric | Value | Notes |
|--------|-------|-------|
| **Control Frequency** | 1000 Hz | Geomagic Touch input rate |
| **Force Feedback Rate** | 100 Hz | Synchronized with joint states |
| **End-to-End Latency** | <10 ms | Position command to robot motion |
| **Workspace Scaling** | 2:1 | Robot moves 2x haptic device motion |
| **Max Force Output** | 3.3 N | Hardware safety limit |
| **Position Accuracy** | ±2 mm | Typical end-effector precision |

---

## Technical Details

### Gravity Compensation Model

The system computes gravity torques for each robot joint:

```python
# Joint torques
τ_shoulder = m_shoulder * g * r_shoulder * cos(θ_shoulder)
τ_elbow = m_elbow * g * r_elbow * cos(θ_elbow_total)
τ_wrist = m_wrist * g * r_wrist * cos(θ_wrist_total)

# Torque-to-force mapping
F_z = (τ_elbow + τ_wrist) * scale      # Primary vertical force
F_x = (τ_wrist * 0.3 + τ_gripper) * scale   # Secondary horizontal
F_y = τ_waist * scale * 0.1            # Minimal rotation effect
```

### Link Parameters

| Link | Mass (kg) | COM Distance (m) | Typical Torque (Nm) |
|------|-----------|------------------|---------------------|
| Shoulder | 1.9 | 0.30 | 6-8 |
| Elbow | 1.1 | 0.175 | 1-2 |
| Wrist | 0.5 | 0.075 | 0.3-0.4 |

### Velocity Damping

Damping forces are applied to create realistic motion resistance:

```python
F_damped = F_gravity - (joint_velocities * damping_coefficient)
```

---

## Project Structure

```
interbotix_ws/
└── src/
    └── wx250_bringup/
        ├── launch/
        │   └── wx250_geomagic_allinone.launch.py    # Main launch file
        ├── wx250_bringup/
        │   ├── geomagic_servo.py                     # Haptic input → Twist
        │   ├── custom_servo_node.py                  # MoveIt Servo wrapper
        │   ├── servo_to_interbotix_bridge.py         # Servo → Robot bridge
        │   └── robot_to_geomagic_feedback.py         # Force feedback generator
        ├── config/
        │   └── wx250_servo_config.yaml               # MoveIt Servo params
        ├── package.xml
        └── setup.py
```

---

##  Future Enhancements

- [ ] **Obstacle Avoidance**: Integrate force feedback for collision detection
- [ ] **Multi-Modal Control**: Add voice commands and gesture recognition
- [ ] **Bimanual Coordination**: Synchronize two robotic arms
- [ ] **Machine Learning**: Adaptive force scaling based on task
- [ ] **Remote Teleoperation**: Network-based control over Internet
- [ ] **VR Integration**: Immersive control interface with visual feedback

---

##  Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

---

## Citation

If you use this work in your research, please cite:

```bibtex
@software{bilateral_teleoperation_wx250,
  author = {Ishaan Janjuha},
  title = {Bilateral Teleoperation System for Interbotix WX250},
  year = {2024},
  url = {https://github.com/ijanjuha/Collaborative-Control-of-Bimanual-Robotic-Arms-WX250-and-GeomagicTouch-Mapping}
}
```

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

##  Author

**Ishaan Janjuha**  
Computer Science Student, IIT Dharwad  
Roll No: CS23BT043

📧 Email: [cs23bt043@iitdh.ac.in](mailto:your-email@iitdh.ac.in)  
🔗 GitHub: [@ijanjuha](https://github.com/ijanjuha)

---

##  Acknowledgments

- **Interbotix** for the WX250 robotic arm ROS2 packages
- **3D Systems** for Geomagic Touch drivers
- **MoveIt2** team for the Servo interface
- **ROS2 Community** for excellent documentation and support

---

<p align="center">
  <b>If you find this project useful, please consider giving it a star! </b>
</p>

<p align="center">
  Made with ❤️ and lots of ☕
</p>
