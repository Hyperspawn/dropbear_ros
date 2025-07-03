# Dropbear ROS Repo

Experimental repo for ROS-2 packages for simulating and controlling Dropbear. I've tried to make a complete guide to use the simulation environment with advanced control interfaces, and mocap motion capabilities. 

## 📋 Overview

Dropbear is a bipedal robot. We're on a journey to make advanced humanoid robots at the edge.

This repo has:
- **Modular Design**: Individual URDF components for torso, arms, legs, head, and battery
- **Advanced Control**: Dual-rate control system (60Hz effort + 1Hz trajectory)
- **Motion Capabilities**: Control each and every motor on the robot and make ros nodes that publish joint position, speed and torque values to simulate walking, running, etc.
- **Multi-Simulator Support**: Gazebo, RViz, and other ROS 2 compatible simulators (Gazebo is recommended)

## 🚀 Setup & Initialization

### Prerequisites
- ROS 2 (Humble or later)
- Gazebo Classic
- RViz2
- Python 3.8+
- Conda environment with ROS 2

### Installation & Launch

```bash
# Activate ROS environment
conda activate ros

# Clone and setup
git clone dropbear_ros
cd dropbear_ros
colcon build
source install/setup.sh

# Launch Detailed Gazebo simulation:

ros2 launch dropbear dropbear_detailed_gazebo.launch.py

# Optional: RViz visualization
ros2 launch dropbear dropbear_detailed_display.launch.py

# Optional: Gazebo with SDF spawn
ros2 launch dropbear dropbear_sdf.launch.py
```

**Important**: When using Gazebo, wait for the model to fully load, then click the **Play ▶️** button in the lower control panel.

### Launch Controllers (Separate Terminal)

```bash
# In a new terminal window, same directory
conda activate ros  # Optional if already activated
source install/setup.sh
ros2 launch dropbear dropbear_controllers.launch.py
```

<img src="https://github.com/user-attachments/assets/a3fb5b5f-da51-4242-9967-6fe8e98c914b" width="200"/>


## 🎮 Control Interfaces

### 1. Hand Trajectory Control

Control individual hand movements with precise joint control:

```bash
ros2 run dropbear hand_trajectory_publisher.py
```

**Usage Example:**
```
[INFO] Enter 'right' or 'left' for the hand to control:
right
[INFO] Hand: Right Hand
[INFO] Controlled Joints: RH_yaw, RH_pitch, RH_roll, RH_wrist_roll

Enter the target position for RH_yaw: 0
Enter the target position for RH_pitch: 1
Enter the target position for RH_roll: 0
Enter the target position for RH_wrist_roll: 0
```
<img src="https://github.com/user-attachments/assets/b042486e-04a4-422b-ad84-9c705fe14b0e" width="200"/>


**Joint Descriptions:**
- **RH_yaw / LH_yaw**: Shoulder rotation control
  - `-1`: Shoulder rotates anticlockwise (45° with full torque, slow speed)
  - `0`: Shoulder faces down (home position, engages hardware locks)
  - `+1`: Shoulder rotates clockwise (45° with full torque, slow speed)

- **RH_pitch / LH_pitch**: Shoulder pitch (outward/inward)
  - Back shoulder actuators
  - From front: clockwise out (-), anticlockwise in (+)

- **RH_roll / LH_roll**: Elbow arm inward/outward movement

- **RH_wrist_roll / LH_wrist_roll**: Wrist rotation control

**Advanced Features** (Experimental):
- Torque control for each joint
- Speed control for each joint

### 2. Leg Trajectory Control

Control leg movements and positioning:

```bash
ros2 run dropbear leg_trajectory_publisher.py
```

**Usage Example:**
```
[INFO] Enter 'right' or 'left' for the leg to control:
right
[INFO] Leg: Right Leg
[INFO] Controlled Joints: RL_hip_joint
Enter the target position for RL_hip_joint: 1

[INFO] Trajectory Sent!
```

<img src="https://github.com/user-attachments/assets/724fdc90-eddd-4790-be66-5fa44ef0acab" width="200"/>

**Features:**
- Choose `right` or `left` leg
- Controls hip joint positioning
- Provides trajectory confirmation

### 3. Pelvic Girdle Control

Stabilize the robot with high-torque base leg control:

```bash
ros2 run dropbear pelvic_girdle_joints_trajectory_publisher.py
```

**Usage Example:**
```
Enter the target position for PG_left_leg_roll: 1
Enter the target position for PG_left_leg_pitch: 2
Enter the target position for PG_right_leg_roll: 1
Enter the target position for PG_right_leg_pitch: 2

[INFO] Trajectory Sent!
```
<img src="https://github.com/user-attachments/assets/eaba530b-ed9f-4b65-b132-df2c5db44ac8" width="200"/>

**Purpose:**
- High-torque back actuators for leg spreading
- Maintains base position and stability
- Controls leg orientation and stance

**Supported Joints:**
- `PG_left_leg_roll`: Left leg roll control
- `PG_left_leg_pitch`: Left leg pitch control
- `PG_right_leg_roll`: Right leg roll control
- `PG_right_leg_pitch`: Right leg pitch control

### 4. Walking Node

Execute pre-trained walking patterns using RL policies:

```bash
python3 dropbear_control/dropbear_control/walking_node.py
```

**Output:**
```
[INFO] Current phase: left_swing
[INFO] Current phase: right_swing
[INFO] Current phase: left_swing
[INFO] Current phase: right_swing
```

<img src="https://github.com/user-attachments/assets/47ed55a6-664a-4ead-a9ca-c79eb1c001f7" width="200"/>

**Features:**
- Walking values generated by trained RL policies
- Alternating left/right leg swings
- Coordinated arm movements for balance
- Phase-based locomotion control

## 🔧 Technical Details

### Control Architecture

#### Dual-Rate Control System
- **High Frequency (60Hz)**: Effort controllers for responsive joint control
- **Low Frequency (1Hz)**: Trajectory controllers for smooth motion planning

#### Controller Types
- **Joint Trajectory Controllers**: Position-based control for arms and legs
- **Effort Controllers**: Torque-based control for knees and elbows
- **Group Controllers**: Coordinated multi-joint movements

### Robot Structure

#### Main Components
- **Torso**: Central body with battery and electronics
- **Head**: Stewart platform with 6-DOF motion
- **Arms**: 5-DOF manipulators (shoulder, bicep, elbow, wrist, etc.)
- **Legs**: 4-DOF bipedal legs (hips, glutes, knee, ankle)
- **Pelvic Girdle**: Base stabilization system with high-torque actuators

#### Joint Configuration
```
Right Arm: RH_yaw, RH_pitch, RH_roll, RH_wrist_roll, RH_elbow_joint
Left Arm:  LH_yaw, LH_pitch, LH_roll, LH_wrist_roll, LH_elbow_joint
Right Leg: RL_hip_joint, RL_knee_actuator_joint, RL_Revolute67, RL_Revolute87
Left Leg:  LL_hip_joint, LL_knee_actuator_joint, LL_Revolute67, LL_Revolute87
Pelvic:    PG_left_leg_roll, PG_left_leg_pitch, PG_right_leg_roll, PG_right_leg_pitch
Head:      Stewart platform with 6 sliders
```

### Motion Capabilities

#### Walking Algorithm
- **Phase-based**: Alternating left/right swing phases
- **Balance Control**: Coordinated arm movements
- **Trajectory Planning**: Smooth joint transitions
- **Stability**: Pelvic girdle stabilization
- **RL-trained**: Values generated by reinforcement learning policies

#### Animation System
- **Motion Capture**: Pre-recorded sequences from Blender
- **YAML Configuration**: Joint value definitions
- **Real-time Playback**: 60Hz animation execution
- **Multiple Behaviors**: Idle, walk, run, jump

## 🎯 Use Cases

### Research & Development
- **Control Algorithm Testing**: Validate new control strategies
- **Motion Planning**: Test path planning algorithms
- **Simulation Studies**: Performance analysis and optimization
- **RL Training**: Test reinforcement learning policies

### Education & Training
- **ROS 2 Learning**: Complete humanoid robot example
- **Control Theory**: Practical implementation of control systems
- **Robotics Education**: Hands-on experience with complex robots

### Prototyping
- **Hardware Testing**: Validate designs before physical construction
- **Software Development**: Test control software in simulation
- **Integration Testing**: Verify system components work together

## 🔬 Experimental Features

### Advanced Control Options
- **Torque Control**: Direct torque commands for joints
- **Speed Control**: Velocity-based joint control
- **Hardware Lock Integration**: Automatic engagement of safety locks

### Custom Animations
Create custom animations by:
1. Defining joint values in `config/joint_values.yaml`
2. Creating animation nodes following the `base_anim_node.py` pattern
3. Implementing the animation logic

## 🛠️ Development

### Adding New Controllers
1. Define controller in `config/controllers.yaml`
2. Add joint mappings in control nodes
3. Implement publisher logic
4. Test with simulation

### Extending Animations
1. Export motion data from Blender
2. Convert to YAML format
3. Create animation node
4. Add to launch files

### Custom URDF Modifications
- Modify XACRO files in `urdf/gazebo/`
- Update joint configurations
- Test with visualization tools

## 🏗️ Architecture

### Core Packages

#### `dropbear/` - Main Robot Description
- **URDF/XACRO**: Modular robot model with detailed meshes
- **Launch Files**: Simulation and visualization launchers
- **Trajectory Publishers**: Individual joint control interfaces
- **Controllers**: ROS 2 Control configuration

#### `dropbear_control/` - Control & Animation
- **Joint Controls**: Advanced joint manipulation nodes
- **Subassemblies Controls**: Control individual subassemblies
- **Walking Algorithm**: Bipedal locomotion controller
- **Animation System**: Idle, walk, run, jump animations


## 📁 Project Structure

```
dropbear_ros/
├── dropbear/                    # Main robot package
│   ├── config/                  # Controller configurations
│   ├── dropbear/               # Trajectory publishers
│   ├── launch/                 # Launch files
│   ├── meshes/                 # 3D model files
│   ├── urdf/                   # Robot description files
│   └── rviz/                   # RViz configurations
├── dropbear_control/           # Control and animation package
│   ├── joint_controls/         # Advanced control nodes
│   ├── mocap_motions/          # Animation system
│   └── config/                 # Animation configurations
└── install/                    # Built packages
```

## 🤝 Contributing

This project is part of the Hyperspawn Robotics ecosystem. Contributions are welcome!

### Development Guidelines
- Follow ROS 2 best practices
- Use proper Python/XML formatting
- Test changes in simulation
- Update documentation

## 🙏 Acknowledgments

- [@robit-man](https://github.com/robit-man)
- [@Priyanshupareek](https://github.com/Priyanshupareek)

---

**Ready to explore humanoid robotics? Start with Dropbear! Find more repos [here](https://github.com/Hyperspawn/repositories) 🚀**