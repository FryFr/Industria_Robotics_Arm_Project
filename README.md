# 🦾 6-DoF Industrial Robotic Arm — Design, Simulation & Control

<div align="center">

![ROS](https://img.shields.io/badge/ROS-Noetic-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Simulator-F58113?style=for-the-badge&logo=gazebo&logoColor=white)
![SolidWorks](https://img.shields.io/badge/SolidWorks-CAD-red?style=for-the-badge)
![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![C++](https://img.shields.io/badge/C%2FC%2B%2B-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)
![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Completed-brightgreen?style=for-the-badge)

**Full-pipeline research project for the design, 3D modeling, kinematic analysis, ROS simulation, and physical construction of a 6-Degrees-of-Freedom robotic arm with a gear-actuated gripper, capable of object pick-and-place operations.**

[Kinematic Model](#-kinematic-model--denavit-hartenberg-analysis) · [URDF Specification](#-urdf-model-specification) · [Arduino Control](#-arduino-control-system) · [Simulation](#-ros--gazebo-simulation) · [Hardware](#-hardware-components) · [Repository Structure](#-repository-structure)

</div>

---

## 📋 Table of Contents

- [Project Overview](#-project-overview)
- [Academic Context](#-academic-context)
- [System Architecture](#-system-architecture)
- [Mechanical Design — SolidWorks](#-mechanical-design--solidworks)
- [Physical Components Breakdown](#-physical-components-breakdown)
- [Kinematic Model — Denavit-Hartenberg Analysis](#-kinematic-model--denavit-hartenberg-analysis)
- [URDF Model Specification](#-urdf-model-specification)
- [Joint Parameters & Motion Limits](#-joint-parameters--motion-limits)
- [Link Inertial Properties](#-link-inertial-properties)
- [Gripper Mechanism](#-gripper-mechanism)
- [ROS & Gazebo Simulation](#-ros--gazebo-simulation)
- [Arduino Control System](#-arduino-control-system)
- [PCA9685 PWM Servo Driver](#-pca9685-pwm-servo-driver)
- [Control Sequence & Automation](#-control-sequence--automation)
- [Inverse Kinematics](#-inverse-kinematics)
- [Hardware Components](#-hardware-components)
- [Repository Structure](#-repository-structure)
- [Setup & Installation](#-setup--installation)
- [Roadmap](#-roadmap)
- [Contributing](#-contributing)
- [Author](#-author)
- [License](#-license)

---

## 🤖 Project Overview

This repository contains the complete engineering deliverables for the design, simulation, and physical construction of a **6-Degrees-of-Freedom (6-DoF) robotic manipulator arm** equipped with a gear-actuated parallel gripper. The project spans the full development pipeline:

1. **CAD modeling** — Each structural component was individually modeled in SolidWorks, with formal engineering drawings and PDF documentation
2. **Kinematic analysis** — Forward kinematics derived using the Denavit-Hartenberg (DH) convention; inverse kinematics formulated analytically
3. **URDF generation** — The SolidWorks assembly was exported to URDF format using the `sw_urdf_exporter` plugin, preserving mesh geometry, inertial tensors, joint axes, and motion limits
4. **ROS simulation** — The URDF model is integrated into ROS with launch files for both RViz visualization and Gazebo physics simulation
5. **Embedded control** — An Arduino UNO + PCA9685 system drives 5 servomotors over I2C with a serial-command automation interface

The arm is designed for **pick-and-place operations** on objects at known workspace locations, controlled via predefined joint-space trajectories.

![URDF Visualization](https://github.com/FryFr/Industria_Robotics_Arm_Project/assets/79547422/183a9d15-5690-4e02-acbe-10d2588c1b59)

---

## 🎓 Academic Context

| Field | Detail |
|---|---|
| **Course** | Robotics |
| **Institution** | Universidad EAN |
| **Supervisor** | Prof. Nikolay Prieto, Ph.D. |
| **Submission Date** | June 10th, 2024 |
| **Project Type** | Research & Engineering Final Project |

---

## 🏗 System Architecture

```
┌────────────────────────────────────────────────────────────────────┐
│                      FULL SYSTEM PIPELINE                          │
│                                                                    │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────────┐  │
│  │  SolidWorks  │───▶│ URDF Export  │───▶│   ROS / Gazebo       │  │
│  │  CAD Model   │    │ sw_urdf v1.6 │    │  RViz + MoveIt       │  │
│  └──────────────┘    └──────────────┘    └──────────────────────┘  │
│                                                                    │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────────┐  │
│  │  DH Analysis │───▶│  Kinematics  │───▶│   Joint Trajectories │  │
│  │  (Forward)   │    │  (Inverse)   │    │   & Path Planning    │  │
│  └──────────────┘    └──────────────┘    └──────────────────────┘  │
│                                                                    │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────────┐  │
│  │  Arduino UNO │───▶│   PCA9685    │───▶│  5× Servo Motors     │  │
│  │  Serial I/O  │    │  I2C PWM     │    │  MG996R + SG90       │  │
│  └──────────────┘    └──────────────┘    └──────────────────────┘  │
└────────────────────────────────────────────────────────────────────┘
```

---

## 🔩 Mechanical Design — SolidWorks

The mechanical design phase used **SolidWorks** to model every structural component from scratch, producing precise digital models that served as manufacturing inputs for 3D printing.

### Individual Part Inventory

| SolidWorks Part | URDF Link | Physical Role |
|---|---|---|
| `BaseCentral.SLDPRT` | `base_link` | Fixed base, anchors the full kinematic chain |
| `Hombro.SLDPRT` | `Link_1` | Shoulder — full 360° rotation on vertical axis |
| `Brazo.SLDPRT` | `Link_2` | Upper arm — elevation from shoulder |
| `Codo.SLDPRT` | `Link_3` | Elbow — bends arm toward target |
| `Mano1.SLDPRT` | `Link_4` | Wrist roll — full 360° rotation |
| `Mano2.SLDPRT` | `Link_5` | Wrist pitch — orients end-effector |
| `Gripper base mano3.SLDPRT` | `Link_6` | Gripper mount — connects wrist to gripper |
| `gear1.SLDPRT` | `Gear_1` | Gripper drive gear — left finger actuator |
| `gear2.SLDPRT` | `Gear_2` | Gripper driven gear — right finger actuator |
| `Gripper 1.SLDPRT` | — | Finger structure |
| `grip link 1.SLDPRT` | — | Finger linkage |
| `Servo Motor MG996R.SLDPRT` | — | High-torque joint actuator (modeled) |
| `Servo Motor Micro 9g.SLDPRT` | — | Lightweight joint actuator (modeled) |

### Assembly Files

| Assembly | Contents |
|---|---|
| `Ensamble.SLDASM` | Primary arm assembly (base to wrist) |
| `Ensamble2.SLDASM` | Full assembly including gripper |
| `EnsamblePinza.SLDASM` | Isolated gripper sub-assembly with gears |

### Engineering Drawings

All parts have formal 2D engineering drawings available in two formats:

- **SolidWorks Drawings** (`.SLDDRW`) — Full parametric drawings with dimensions, tolerances and views
- **PDF Drawings** — Exported static drawings for reference and printing

Available drawings: `BaseCentral`, `Brazo`, `Codo`, `Hombro`, `Mano1`, `Mano2`, `gear1`, `gear2`, `Gripper 1`, `Gripper base mano3`, `grip link 1`, `Ensamble1_brazo` (sub-assembly), `Ensamble_explosionado_robot` (exploded view of full robot).

> 🔧 The exploded view (`Ensamble_explosionado_robot.pdf`) is particularly useful for understanding assembly order and fastener placement.

### 3D Printing

All structural parts were manufactured using **FDM (Fused Deposition Modeling) 3D printing** with **PLA (Polylactic Acid)** filament. PLA was selected for its:
- Low warping during printing
- Sufficient structural rigidity for low-load robotic applications
- Ease of post-processing
- Biodegradable properties

The SolidWorks parts were exported to STL format (the same STL meshes used in the URDF), ensuring dimensional consistency between the CAD model, the simulation, and the physical prototype.

![Physical Assembly](https://github.com/FryFr/Industria_Robotics_Arm_Project/assets/79547422/d15d2743-2af6-4c10-8b84-d61453d19474)

---

## 📐 Kinematic Model — Denavit-Hartenberg Analysis

The forward kinematics of the arm were derived using the **Denavit-Hartenberg (DH) convention**, which provides a systematic 4-parameter representation of the transformation between consecutive joint frames.

### Why DH over Trigonometric Analysis?

Trigonometric position equations and direct Cartesian formulations are viable for 2- or 3-DoF systems. However, for a 6-DoF manipulator, the complexity of coupled rotations and translations makes these approaches intractable. The DH convention reduces each inter-frame transformation to a product of four elementary operations:

```
T(i-1→i) = Rot_z(θᵢ) · Trans_z(dᵢ) · Trans_x(aᵢ) · Rot_x(αᵢ)
```

Where:
- **θᵢ** — joint angle (rotation around z-axis, variable for revolute joints)
- **dᵢ** — link offset (translation along z-axis)
- **aᵢ** — link length (translation along x-axis)
- **αᵢ** — link twist (rotation around x-axis)

### DH Frame Assignment Convention

1. Frame 0 was fixed to the world/base and assigned to the first joint axis
2. Subsequent frames (1–6) were positioned following the DH rules, strictly avoiding pure rotations or translations about the Y-axis (which would require a modified DH formulation)
3. The final frame (Frame 6) coincides with the end-effector at the gripper center

### DH Table

Based on the URDF joint origins and axes, the DH parameters for the 6 main joints are:

| Joint | θᵢ (variable) | dᵢ (m) | aᵢ (m) | αᵢ (rad) | Type |
|---|---|---|---|---|---|
| Joint_1 | θ₁ | 0.0465 | 0.0308 | 0 | Revolute |
| Joint_2 | θ₂ | 0.1150 | 0.0011 | π/2 | Revolute |
| Joint_3 | θ₃ | 0 | 0.1000 | 0 | Revolute |
| Joint_4 | θ₄ | 0.0818 | 0.0001 | -π/2 | Revolute |
| Joint_5 | θ₅ | 0.0215 | 0 | -π/2 | Revolute |
| Joint_6 | θ₆ | 0 | 0.0350 | -π/2 | Revolute |

> Values extracted from URDF joint `<origin>` tags. The full homogeneous transformation matrix T(0→6) = T₁ · T₂ · T₃ · T₄ · T₅ · T₆ gives the end-effector pose in the base frame.

### Forward Kinematics

The full forward kinematic chain resolves to:

```
T(base→EE) = T₁(θ₁) × T₂(θ₂) × T₃(θ₃) × T₄(θ₄) × T₅(θ₅) × T₆(θ₆)
```

This yields a **4×4 homogeneous transformation matrix** encoding the end-effector position (x, y, z) and orientation (R₃ₓ₃ rotation matrix) in the world frame.

![DH Frame Diagram](https://github.com/FryFr/Industria_Robotics_Arm_Project/assets/79547422/5c642a34-261c-49b6-b497-d15a4341e0b9)

---

## 📄 URDF Model Specification

The URDF was automatically generated from the SolidWorks assembly using the **`sw_urdf_exporter` plugin v1.6.0-4-g7f85cfe (Build 1.6.7995.38578)** by Stephen Brawner.

The model file is located at:
```
1.Software/Ensamblaje_urdf/urdf/Ensamblaje_urdf.urdf
```

### Link Tree

```
base_link
└── Link_1        (Joint_1  — revolute, z-axis)
    └── Link_2    (Joint_2  — revolute, z-axis, limited ±30°)
        └── Link_3  (Joint_3 — revolute, z-axis)
            └── Link_4  (Joint_4 — revolute, z-axis, full)
                └── Link_5  (Joint_5 — revolute, z-axis)
                    └── Link_6  (Joint_6 — revolute, z-axis, full)
                        ├── Gear_1  (joint_gear1 — revolute, y-axis, ±30°)
                        └── Gear_2  (Joint_gear2 — revolute, y-axis, ±30°)
```

### STL Mesh Files

All meshes reside in `1.Software/Ensamblaje_urdf/meshes/`:

| Mesh File | Link | Approx. Size |
|---|---|---|
| `base_link.STL` | base_link | 108 KB |
| `Link_1.STL` | Link_1 (Shoulder) | 114 KB |
| `Link_2.STL` | Link_2 (Upper arm) | 104 KB |
| `Link_3.STL` | Link_3 (Elbow) | 54 KB |
| `Link_4.STL` | Link_4 (Wrist roll) | 55 KB |
| `Link_5.STL` | Link_5 (Wrist pitch) | 23 KB |
| `Link_6.STL` | Link_6 (Gripper mount) | 33 KB |
| `Gear_1.STL` | Gear_1 (Left gripper) | 69 KB |
| `Gear_2.STL` | Gear_2 (Right gripper) | 64 KB |

All meshes use the visual/collision color `rgba(0.792, 0.820, 0.933, 1.0)` — a light steel blue.

---

## 📊 Joint Parameters & Motion Limits

The following table consolidates all joint data extracted directly from the URDF:

| Joint | Type | Parent → Child | Origin (x, y, z) m | Origin (r, p, y) rad | Axis | Lower (rad) | Upper (rad) | Range (°) | Friction |
|---|---|---|---|---|---|---|---|---|---|
| `Joint_1` | revolute | base → Link_1 | (0.031, -0.097, 0.047) | (0, 0, π/2) | Z | -3.141 | 3.141 | 360° | 0.1 |
| `Joint_2` | revolute | Link_1 → Link_2 | (-0.001, -0.019, 0.115) | (π/2, 0, -0.057) | Z | -0.524 | 0.524 | 60° | 0.1 |
| `Joint_3` | revolute | Link_2 → Link_3 | (0, 0.100, 0) | (0, 0, π/2) | Z | -1.570 | 0.349 | 110° | 0.1 |
| `Joint_4` | revolute | Link_3 → Link_4 | (0.000, 0.082, 0.006) | (-π/2, 0, 0) | Z | -3.141 | 3.141 | 360° | 0.1 |
| `Joint_5` | revolute | Link_4 → Link_5 | (0, -0.008, 0.022) | (-π/2, 0, -π) | Z | -3.141 | 0.174 | 190° | 0.1 |
| `Joint_6` | revolute | Link_5 → Link_6 | (0.035, -0.000, -0.008) | (-π/2, 0, -π/2) | Z | -3.141 | 3.141 | 360° | 0.1 |
| `joint_gear1` | revolute | Link_6 → Gear_1 | (-0.014, 0.002, 0.023) | (0, 0, 0) | Y | -0.523 | 0.523 | 60° | — |
| `Joint_gear2` | revolute | Link_6 → Gear_2 | (0.013, -0.003, 0.023) | (0, 0, 0) | Y | -0.523 | 0.523 | 60° | — |

### Total Workspace DoF

- **6 active joints** (Joint_1 through Joint_6) — full arm positioning and orientation
- **2 gripper joints** (joint_gear1 + Joint_gear2) — parallel gear-actuated gripper
- **Total: 8 revolute joints** in the kinematic chain

### Servo-to-Joint Mapping (Physical)

| PCA9685 Channel | Physical Joint | SolidWorks Link | Servo Model | Angle Limit |
|---|---|---|---|---|
| Servo 0 | Base rotation | base_link ↔ Link_1 | MG996R | 0° – 180° |
| Servo 1 | Shoulder elevation | Link_1 ↔ Link_2 | MG996R | 0° – 85° |
| Servo 2 | Elbow flex | Link_2 ↔ Link_3 | MG996R | 0° – 90° |
| Servo 3 | Wrist pitch | Link_3 ↔ Link_4 | SG90 | 2° – 90° |
| Servo 4 | Gripper open/close | Gear_1 + Gear_2 | SG90 | 0° – 80° |

---

## ⚖️ Link Inertial Properties

Inertial data was computed directly by SolidWorks from the PLA density and exported to the URDF. These values are used by Gazebo for realistic physics simulation.

| Link | Mass (kg) | CoM X (m) | CoM Y (m) | CoM Z (m) | Ixx (kg·m²) | Iyy (kg·m²) | Izz (kg·m²) |
|---|---|---|---|---|---|---|---|
| base_link | 0.09282 | 0.03222 | -0.09726 | 0.06830 | 1.236e-4 | 1.267e-4 | 2.077e-4 |
| Link_1 | 0.10004 | 0.00279 | -0.01420 | 0.07333 | 1.228e-4 | 1.201e-4 | 1.071e-4 |
| Link_2 | 0.03836 | 0.00023 | 0.04485 | -0.00621 | 4.759e-5 | 5.827e-6 | 5.167e-5 |
| Link_3 | 0.02286 | 0.00070 | 0.02860 | 0.00472 | 1.938e-5 | 4.279e-6 | 2.256e-5 |
| Link_4 | 0.01138 | ~0 | -0.01420 | 0.00948 | 2.517e-6 | 2.816e-6 | 1.653e-6 |
| Link_5 | 0.01079 | 0.02085 | -0.00257 | -0.00310 | 1.978e-6 | 2.552e-6 | 3.258e-6 |
| Link_6 | 0.01024 | -0.00420 | 0.00847 | 0.01757 | 1.703e-6 | 3.123e-6 | 1.667e-6 |
| Gear_1 | 0.00933 | -0.01598 | 0.00255 | 0.02202 | 1.711e-6 | 2.285e-6 | 6.491e-7 |
| Gear_2 | 0.00938 | 0.01667 | 0.00652 | 0.02210 | 1.730e-6 | 2.285e-6 | 6.304e-7 |
| **Total** | **~0.336 kg** | — | — | — | — | — | — |

> The **total estimated arm mass is ~336 g**, consistent with a lightweight 3D-printed PLA prototype. Link_1 (Shoulder) is the heaviest link at 100 g.

---

## 🤏 Gripper Mechanism

The end-effector is a **gear-actuated parallel gripper** using a spur gear pair to synchronize the opening and closing of two finger assemblies:

### Gear Pair Specification

| Parameter | Gear_1 | Gear_2 |
|---|---|---|
| Parent Link | Link_6 | Link_6 |
| Origin offset | (-0.0139, 0.0015, 0.0225) | (0.013, -0.0025, 0.0225) |
| Rotation axis | Y | Y |
| Motion range | ±0.523 rad (±30°) | ±0.523 rad (±30°) |
| Mass | 9.33 g | 9.38 g |

### Gripper Sub-Assembly Components

The `EnsamblePinza.SLDASM` sub-assembly contains:
- `Gripper base mano3` — palm/base plate connecting to Link_6
- `Gripper 1` (×2) — finger bodies
- `grip link 1` (×4) — four-bar linkage elements connecting gears to fingers
- `gear1` + `gear2` — mating spur gear pair

The four-bar linkage mechanism converts the ±30° gear rotation into a symmetric lateral motion of both fingers, enabling **parallel jaw grasping**.

---

## 🖥 ROS & Gazebo Simulation

### Software Requirements

| Package | Version | Purpose |
|---|---|---|
| Ubuntu | 20.04 LTS | Operating system |
| ROS | Noetic (ROS 1) | Robot middleware and launch system |
| Gazebo | 11 | Physics simulation |
| RViz | 1.14+ | 3D visualization |
| `robot_state_publisher` | — | Broadcasts joint transforms (TF) |
| `joint_state_publisher_gui` | — | Interactive joint sliders in RViz |
| `sw_urdf_exporter` | 1.6.0 | SolidWorks → URDF export plugin |

> ⚠️ **Note:** The launch files use **ROS 1 (Noetic)** syntax (`roslaunch`, `<node pkg="...">`, `$(find ...)`). Despite references to ROS2 in the project documentation, the actual launch infrastructure is ROS 1.

### ROS Package Setup

```bash
# Create and build the workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
# Copy the Ensamblaje_urdf package here
cp -r /path/to/Industria_Robotics_Arm_Project/1.Software/Ensamblaje_urdf .

cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Launch — RViz Visualization

Visualize the arm with interactive joint sliders:

```bash
roslaunch Ensamblaje_urdf display.launch
```

This launches:
- `joint_state_publisher_gui` — GUI sliders for manually commanding each joint angle
- `robot_state_publisher` — Computes and broadcasts all link TF transforms from joint states
- `rviz` — 3D visualization with the robot model loaded

### Launch — Gazebo Physics Simulation

Spawn the arm in an empty Gazebo world:

```bash
roslaunch Ensamblaje_urdf gazebo.launch
```

This launches:
- `gazebo` (empty world)
- `spawn_model` — loads the URDF into Gazebo via `gazebo_ros`
- `tf_footprint_base` — static TF between `base_link` and `base_footprint`
- `fake_joint_calibration` — publishes `/calibrated` to unblock joint controllers

### Joint Names Configuration

The controller joint list is defined in `config/joint_names_Ensamblaje_urdf.yaml`:

```yaml
controller_joint_names: [
  '',           # placeholder
  'Joint_1',    # Shoulder rotation
  'Joint_2',    # Upper arm elevation
  'Joint_3',    # Elbow flexion
  'Joint_4',    # Wrist roll
  'Joint_5',    # Wrist pitch
  'Joint_6',    # End-effector rotation
  'joint_gear1', # Gripper left
  'Joint_gear2'  # Gripper right
]
```

### URDF Validation

Before launching, validate the URDF for kinematic and XML errors:

```bash
# Check URDF structure
check_urdf 1.Software/Ensamblaje_urdf/urdf/Ensamblaje_urdf.urdf

# Visualize link tree
urdf_to_graphiz 1.Software/Ensamblaje_urdf/urdf/Ensamblaje_urdf.urdf
evince Ensamblaje_urdf.pdf
```

---

## 🎮 Arduino Control System

### Architecture

```
PC / Host
   │
   │  Serial (USB, 9600 baud)
   ▼
Arduino UNO
   │
   │  I2C (SDA/SCL — pins A4/A5)
   ▼
PCA9685 (16-ch PWM Driver @ 0x40)
   │
   ├── Ch 0 — MG996R — Base rotation
   ├── Ch 1 — MG996R — Shoulder elevation
   ├── Ch 2 — MG996R — Elbow flexion
   ├── Ch 3 — SG90  — Wrist pitch
   └── Ch 4 — SG90  — Gripper
```

### Core Control Code — `Manual_Control.ino`

```cpp
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm;

// PWM pulse width range for servo control (in 12-bit counts @ 60 Hz)
const int SERVOMIN = 150;  // ~0.7 ms pulse  → 0°
const int SERVOMAX = 600;  // ~2.4 ms pulse  → 180°
```

### PWM Frequency & Pulse Width

The PCA9685 is configured at **60 Hz**, which is the standard PWM frequency for analog servomotors. The `SERVOMIN` and `SERVOMAX` counts correspond to:

```
Pulse width (µs) = count × (1 / (60 Hz × 4096)) × 10⁶
SERVOMIN (150) → 150 / (60 × 4096) × 1e6 ≈ 610 µs
SERVOMAX (600) → 600 / (60 × 4096) × 1e6 ≈ 2441 µs
```

The angle-to-pulse mapping:

```cpp
int pulseLength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
pwm.setPWM(servo, 0, pulseLength);
```

---

## 🔧 PCA9685 PWM Servo Driver

The **PCA9685** is a 16-channel, 12-bit PWM driver from NXP/Adafruit that communicates over I2C. It offloads all servo timing from the Arduino, allowing deterministic PWM signals on up to 16 channels simultaneously.

| Parameter | Value |
|---|---|
| Communication | I2C (default address `0x40`) |
| Channels | 16 |
| Resolution | 12-bit (4096 steps per cycle) |
| PWM Frequency | Configurable 40–1000 Hz (set to 60 Hz) |
| Arduino Pins | SDA → A4, SCL → A5 |
| Library | `Adafruit_PWMServoDriver` |
| Operating Voltage | 3.3V or 5V logic; servo power separate (V+) |

> ⚠️ **Important:** The PCA9685 V+ rail must be connected to an **external 5–6V DC power supply**, not the Arduino 5V pin. The MG996R servos draw up to 2.5A stall current each — powering them from the Arduino will cause brownouts and undefined behavior.

### I2C Wiring

```
Arduino UNO     PCA9685
-----------     -------
A4 (SDA)   ──── SDA
A5 (SCL)   ──── SCL
5V         ──── VCC (logic)
GND        ──── GND
           ──── V+  ←── External 5V supply (+)
           ──── GND ←── External 5V supply (-)
```

---

## 🔄 Control Sequence & Automation

The firmware implements a **serial-triggered state machine**. The user sends single-character commands via the Serial Monitor:

| Command | Action |
|---|---|
| `Y` or `y` | Start pick-and-place automation sequence |
| `S` or `s` | Stop and hold current position |

### Predefined Motion Sequence

The `executeSequence()` function defines three joint-space waypoints:

```
POSITION 1 — Home (Initial)
   Servo 0: 90°  (base centered)
   Servo 1:  0°  (shoulder down)
   Servo 2:  0°  (elbow straight)
   Servo 3:  2°  (wrist neutral)
   Servo 4: 80°  (gripper open)

POSITION 2 — Object Pickup
   Servo 0: 90°  (base centered)
   Servo 1: 45°  (shoulder mid-elevation)
   Servo 2: 45°  (elbow bent)
   Servo 3: 45°  (wrist angled)
   Servo 4:  0°  (gripper closed → grips object)

POSITION 3 — Destination
   Servo 0: 150° (base rotated to drop zone)
   Servo 1: 20°  (shoulder lowered)
   Servo 2: 20°  (elbow extended)
   Servo 3: 20°  (wrist adjusted)
   Servo 4: 80°  (gripper open → releases object)

→ Returns to POSITION 1 (Home)
```

Each position is held for **1000 ms** before transitioning to the next waypoint.

### Per-Joint Angle Limits (Firmware)

These software limits are enforced independently of the hardware/URDF limits:

| Channel | Joint | Min (°) | Max (°) | Notes |
|---|---|---|---|---|
| 0 | Base | 0 | 180 | Full servo range |
| 1 | Shoulder | 0 | 85 | Mechanically constrained |
| 2 | Elbow | 0 | 90 | Mechanically constrained |
| 3 | Wrist pitch | 2 | 90 | Offset to avoid singularity at 0° |
| 4 | Gripper | 0 | 80 | 0° = closed, 80° = open |

---

## 🧮 Inverse Kinematics

The `1.Software/Invers_Kinematics/` directory is reserved for inverse kinematics computation. The analytical IK problem for a 6-DoF spherical-wrist manipulator can be decoupled into:

1. **Positional IK** (Joints 1–3): Determines the shoulder, elbow, and base angles to position the wrist center at the desired 3D location
2. **Orientational IK** (Joints 4–6): Determines the wrist Euler angles to achieve the desired end-effector orientation

### Analytical Approach (Spherical Wrist Decoupling)

For a robot with a spherical wrist (Joints 4, 5, 6 intersecting at a single point), the IK decouples as:

```
Step 1 — Compute wrist center position:
   Pw = Pd - d₆ · R_desired · [0, 0, 1]ᵀ

Step 2 — Solve for θ₁, θ₂, θ₃ geometrically:
   θ₁ = atan2(Pw_y, Pw_x)
   Use law of cosines for θ₂, θ₃ given link lengths a₂, a₃

Step 3 — Solve for θ₄, θ₅, θ₆:
   R₃₆ = (R₀₃)ᵀ · R_desired
   Extract Euler angles from R₃₆
```

> 📌 **Note:** The `Invers_Kinematics/Sample.txt` placeholder indicates this module is planned for future implementation. Contributions implementing the analytical IK are welcome — see [Contributing](#-contributing).

---

## 🧰 Hardware Components

| Component | Model | Qty | Role |
|---|---|---|---|
| Microcontroller | Arduino UNO | 1 | Main controller, serial interface |
| PWM Driver | PCA9685 (Adafruit) | 1 | 16-channel I2C servo controller |
| High-torque servo | MG996R | 3 | Joints 0–2 (base, shoulder, elbow) |
| Micro servo | SG90 (9g) | 2 | Joints 3–4 (wrist, gripper) |
| DC Power Supply | 5–6V, 3A+ | 1 | Servo power (external) |
| PLA Filament | 1.75mm PLA | ~250g | 3D printing all structural parts |
| Fasteners | M2/M3 screws+nuts | Assorted | Part assembly |
| Jumper wires | M-M, M-F | ~20 | Electronic connections |
| Protoboard | Full-size | 1 | Component layout |

### Servo Comparison

| Spec | MG996R | SG90 |
|---|---|---|
| Stall Torque | 9.4 kg·cm (6V) | 1.8 kg·cm (5V) |
| Operating Voltage | 4.8–7.2V | 4.8–6V |
| Speed | 0.19 s/60° | 0.1 s/60° |
| Weight | 55 g | 9 g |
| Gear Type | Metal | Plastic/nylon |
| Use in project | Primary joints (high load) | Wrist + gripper (low load) |

---

## 📁 Repository Structure

```
Industria_Robotics_Arm_Project/
│
├── 0.Model/
│   ├── 0.0. Individual Parts/          # SolidWorks part files (.SLDPRT)
│   │   ├── BaseCentral.SLDPRT          # Base link
│   │   ├── Hombro.SLDPRT              # Shoulder (Link_1)
│   │   ├── Brazo.SLDPRT               # Upper arm (Link_2)
│   │   ├── Codo.SLDPRT                # Elbow (Link_3)
│   │   ├── Mano1.SLDPRT               # Wrist roll (Link_4)
│   │   ├── Mano2.SLDPRT               # Wrist pitch (Link_5)
│   │   ├── Gripper base mano3.SLDPRT  # Gripper mount (Link_6)
│   │   ├── Gripper 1.SLDPRT           # Finger body
│   │   ├── grip link 1.SLDPRT         # Finger linkage
│   │   ├── gear1.SLDPRT               # Drive gear
│   │   ├── gear2.SLDPRT               # Driven gear
│   │   ├── Servo Motor MG996R.SLDPRT  # High-torque servo (model)
│   │   └── Servo Motor Micro 9g.SLDPRT # Micro servo (model)
│   │
│   ├── 0.1. Assembly/
│   │   ├── Ensamble.SLDASM             # Main arm assembly
│   │   ├── Ensamble2.SLDASM            # Full assembly with gripper
│   │   └── EnsamblePinza.SLDASM        # Gripper sub-assembly
│   │
│   └── 0.2. Sketch/
│       ├── PDF Drawings/               # Engineering drawings (PDF, 13 sheets)
│       └── SolidWorks Drawings/        # Parametric drawings (.SLDDRW)
│
├── 1.Software/
│   ├── Ensamblaje_urdf/                # ROS URDF package
│   │   ├── urdf/
│   │   │   ├── Ensamblaje_urdf.urdf    # Full robot URDF (9 links, 8 joints)
│   │   │   └── Ensamblaje_urdf.csv     # DH/inertial data export from SolidWorks
│   │   ├── meshes/                     # STL collision/visual meshes (9 files)
│   │   ├── launch/
│   │   │   ├── display.launch          # RViz + joint_state_publisher_gui
│   │   │   └── gazebo.launch           # Gazebo physics simulation
│   │   ├── config/
│   │   │   └── joint_names_Ensamblaje_urdf.yaml  # Controller joint list
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── Manual_Control/
│   │   └── Manual_Control.ino          # Arduino firmware (PCA9685 + serial)
│   │
│   └── Invers_Kinematics/              # IK solver (placeholder — future work)
│
├── 2. Documents/
│   └── Proyecto Robótica.docx          # Full research document
│
└── README.md
```

---

## ⚙️ Setup & Installation

### Prerequisites

| Tool | Version | Installation |
|---|---|---|
| Ubuntu | 20.04 LTS | — |
| ROS Noetic | 1.15+ | [ros.org/install](http://wiki.ros.org/noetic/Installation/Ubuntu) |
| Gazebo | 11 | Included with `ros-noetic-desktop-full` |
| Arduino IDE | 1.8+ | [arduino.cc/downloads](https://www.arduino.cc/en/software) |
| SolidWorks | 2022+ | Commercial license required |

### ROS Environment Setup

```bash
# 1. Install ROS Noetic (full desktop)
sudo apt update
sudo apt install ros-noetic-desktop-full

# 2. Install additional packages
sudo apt install ros-noetic-joint-state-publisher-gui \
                 ros-noetic-robot-state-publisher \
                 ros-noetic-gazebo-ros-pkgs \
                 ros-noetic-xacro

# 3. Create catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 4. Clone the repository
git clone https://github.com/FryFr/Industria_Robotics_Arm_Project.git

# 5. Copy ROS package to src
cp -r Industria_Robotics_Arm_Project/1.Software/Ensamblaje_urdf .

# 6. Build
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Arduino Setup

```bash
# Install Adafruit PWM Servo Driver library
# In Arduino IDE: Tools → Manage Libraries → Search "Adafruit PWM Servo Driver"
# OR via CLI:
arduino-cli lib install "Adafruit PWM Servo Driver Library"
```

Open `1.Software/Manual_Control/Manual_Control.ino`, select **Arduino UNO** as the board, select the correct COM port, and upload. Use the **Serial Monitor at 9600 baud** to send `Y`/`S` commands.

### Running the Simulation

```bash
# Terminal 1 — Start ROS master
roscore

# Terminal 2 — RViz visualization (interactive joint sliders)
roslaunch Ensamblaje_urdf display.launch

# OR Terminal 2 — Gazebo physics simulation
roslaunch Ensamblaje_urdf gazebo.launch
```

---

## 🗺 Roadmap

- [ ] **Inverse kinematics solver** — Implement analytical IK using spherical wrist decoupling in the `Invers_Kinematics/` module
- [ ] **MoveIt! integration** — Configure MoveIt! for motion planning, collision avoidance, and trajectory execution
- [ ] **ROS2 migration** — Port launch files and controllers to ROS2 (Humble or Iron)
- [ ] **Closed-loop control** — Add joint encoders or potentiometers for position feedback
- [ ] **Vision-guided grasping** — Integrate an RGB-D camera (RealSense or ZED) with OpenCV/PCL for object detection
- [ ] **URDF → SDF conversion** — Improve Gazebo simulation with proper SDF dynamics
- [ ] **Trajectory interpolation** — Replace discrete waypoints with smooth cubic spline joint trajectories
- [ ] **Web dashboard** — Real-time joint monitoring and commanding via ROS Bridge + WebSocket

---

## 🤝 Contributing

Contributions are welcome, especially for:

- Implementing the analytical **Inverse Kinematics** module (Python or C++)
- Migrating the launch system to **ROS2**
- Adding **MoveIt!** configuration files
- Improving servo **calibration** or adding encoder feedback

To contribute:

1. **Fork** this repository
2. Create your feature branch:
   ```bash
   git checkout -b feature/inverse-kinematics
   ```
3. Commit with descriptive messages:
   ```bash
   git commit -m "feat(ik): implement analytical inverse kinematics with spherical wrist decoupling"
   ```
4. Push and open a **Pull Request**

### Commit Convention

| Prefix | Scope | Usage |
|---|---|---|
| `feat` | `(ik)`, `(urdf)`, `(ctrl)` | New functionality |
| `fix` | — | Bug fix |
| `sim` | `(gazebo)`, `(rviz)` | Simulation changes |
| `hw` | `(servo)`, `(arduino)` | Hardware/firmware changes |
| `docs` | — | Documentation only |
| `cad` | `(solidworks)` | CAD model changes |

---

## 👨‍💻 Author

**Juan Silva Medina**
- GitHub: [@FryFr](https://github.com/FryFr)
- LinkedIn: [linkedin.com/in/jsilva-medina](https://www.linkedin.com/in/jsilva-medina/?skipRedirect=true)
- YouTube: [@juansilva4256](https://www.youtube.com/@juansilva4256)

**Academic Supervisor:** Prof. Nikolay Prieto, Ph.D. — Universidad EAN

---

## 📄 License

Distributed under the **MIT License**. See [`LICENSE`](./LICENSE) for more details.

```
MIT License — Copyright (c) 2024 Juan Silva Medina
```

---

## 📚 References

- Denavit, J. & Hartenberg, R.S. (1955). *A kinematic notation for lower-pair mechanisms based on matrices.* Journal of Applied Mechanics, 22(2), 215–221.
- Craig, J.J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson Prentice Hall.
- Siciliano, B. et al. (2009). *Robotics: Modelling, Planning and Control.* Springer.
- ROS Wiki — [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- Adafruit PCA9685 — [Product Guide](https://learn.adafruit.com/16-channel-pwm-servo-driver)
- Brawner, S. — [SolidWorks to URDF Exporter](http://wiki.ros.org/sw_urdf_exporter)

---

<div align="center">
  <sub>Built with ❤️, SolidWorks, ROS and a lot of PLA · Found it useful? Give it a ⭐</sub>
</div>
