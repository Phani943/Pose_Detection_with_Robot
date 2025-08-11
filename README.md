# Harro Robot - Pose Detection with ROS2 Simulation

A comprehensive robotics project featuring a custom-designed robot with real-time pose detection capabilities, built from scratch using Blender, ROS2, Gazebo simulation, and YOLOv8 computer vision.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)
![Python](https://img.shields.io/badge/Python-3.8+-green.svg)
![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange.svg)
![YOLOv8](https://img.shields.io/badge/YOLOv8-Ultralytics-red.svg)
![License](https://img.shields.io/badge/License-MIT-yellow.svg)

## 🎥 Demo Videos
- **[Short Demo](https://github.com/user-attachments/assets/91003354-3e35-4f93-8cc2-c6a074bf917f)** - Quick project overview
- **[Full Demo - "Harro Robo"](https://github.com/user-attachments/assets/5b935d54-5257-49ef-b2e1-b21cbe44728d)** - Complete demonstration with audio

## Robot Preview
<img src="render_img.png" alt="Robot Preview"/>

## 🚀 Project Overview

Harro Robot is a fully integrated robotics system that combines:
- **Custom 3D Robot Design** created in Blender
- **ROS2-based Control System** for robot movement and sensor integration
- **Real-time Pose Detection** using YOLOv8 neural network
- **Gazebo Simulation Environment** with interactive human models
- **Teleop Control Interface** for manual robot operation

## 🏗️ System Architecture

```
+---------------+      +---------------+      +---------------+
| Blender 3D    | ---> | ROS2 Package  | ---> | Gazebo        |
| Robot Design  |      | Integration   |      | Simulation    |
+---------------+      +---------------+      +---------------+
       |
       v
+---------------+      +---------------+      +---------------+
| Camera Feed   | ---> | YOLOv8 Pose   | ---> | Pose Annotated|
| Subscriber    |      | Detection     |      | Publisher     |
+---------------+      +---------------+      +---------------+
```

## 🛠️ Technology Stack

- **3D Modeling**: Blender with Phobos plugin for URDF generation
- **Robotics Framework**: ROS2 Humble
- **Simulation**: Gazebo Classic
- **Computer Vision**: YOLOv8 (Ultralytics)
- **Programming**: Python 3.8+
- **Build System**: ament_python
- **Visualization**: RViz2, rqt_image_view

## 📁 Project Structure

```
my_harro_robot/
├── package.xml                # ROS2 package configuration
├── setup.py                   # Python package setup
├── setup.cfg                  # Setup configuration
├── launch/                    # Launch files
│   ├── display.launch.py      # RViz2 visualization
│   ├── gazebo.launch.py       # Gazebo simulation
│   └── spawn_models.launch.py # Environmental models
├── meshes/                    # Robot 3D components
│   ├── Body.dae
│   ├── Camera.dae
│   └── Wheel-[1-4].dae
├── models/                    # Human pose models
│   ├── boy_girl.dae
│   ├── boy_icecream.dae
│   ├── boy_on_table.dae
│   ├── girl_with_bag.dae
│   ├── man_sofa_sitting.dae
│   └── man_standing.dae
├── my_harro_robot/            # Python package
│   ├── move_me.py             # Teleop control
│   ├── pose_detection.py      # YOLOv8 pose detection
│   ├── yolov8n-pose.pt        # Pre-trained pose model
│   └── __init__.py
├── textures/                  # Human model textures
├── trees/                     # Tree models for environment
├── tree_textures/             # Tree textures
└── urdf/                      # Robot description
    └── harro.xacro            # Main robot URDF
```

## ⚡ Features

### 🤖 Robot Capabilities
- **Custom Design**: Built from scratch in Blender with realistic physics
- **Differential Drive**: 4-wheel mobile robot with camera sensor
- **Real-time Control**: Teleop interface with arrow key navigation
- **Camera Integration**: Live video feed processing

### 🧠 AI & Computer Vision
- **YOLOv8 Pose Detection**: Real-time human pose estimation
- **Live Processing**: Processes camera feed at video rate
- **Pose Visualization**: Annotated skeleton overlays
- **ROS2 Integration**: Seamless topic-based communication

### 🌍 Simulation Environment
- **Realistic Physics**: Gazebo-based simulation with proper dynamics
- **Interactive Humans**: Multiple human models in various poses
- **Environmental Assets**: Trees and textures for realistic scenes
- **Modular Loading**: Configurable model spawning system

## 🚀 Installation

### Prerequisites
ROS2 Humble installation required.

```bash
sudo apt update
sudo apt install ros-humble-desktop
```

Additional dependencies:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-cv-bridge
pip install ultralytics opencv-python
```

### Build Instructions

```bash
# Create workspace
mkdir -p ~/harro_ws/src
cd ~/harro_ws/src

# Clone repository
git clone https://github.com/Phani943/Pose_Detection_with_Robot.git
mv Pose_Detection_with_Robot my_harro_robot

# Build package
cd ~/harro_ws
colcon build --packages-select my_harro_robot

# Source workspace
source install/setup.bash
```

## 🎮 Usage

### 1. Launch Gazebo Simulation
In Terminal 1:
```bash
ros2 launch my_harro_robot gazebo.launch.py
```

In Terminal 2: spawn environmental models
```bash
ros2 launch my_harro_robot spawn_models.launch.py
```

### 2. Start Pose Detection
In Terminal 3:
```bash
ros2 run my_harro_robot pose_detection_node
```

### 3. Control Robot
In Terminal 4:
```bash
ros2 run my_harro_robot move_me
```

Use arrow keys to control:
- ↑ : Forward
- ↓ : Backward
- ← : Turn left
- → : Turn right
- Space: Stop
- R: Rotate continuously

### 4. Visualize Results
In Terminal 5:
```bash
ros2 run rqt_image_view rqt_image_view /pose_img
```

Or launch RViz2 for robot visualization:
```bash
ros2 launch my_harro_robot display.launch.py
```

## 📊 ROS2 Topics

| Topic                         | Type                      | Description                 |
|------------------------------:|---------------------------|-----------------------------|
| `/cmd_vel`                    | `geometry_msgs/Twist`     | Robot movement commands     |
| `/harro_camera_feed/image_raw`| `sensor_msgs/Image`       | Raw camera input            |
| `/pose_img`                   | `sensor_msgs/Image`       | Pose-annotated output       |
| `/odom`                       | `nav_msgs/Odometry`       | Robot odometry data         |

## 🎯 Key Nodes

### `pose_detection_node`
- **Subscribes**: Camera feed from robot
- **Processes**: YOLOv8 pose detection inference
- **Publishes**: Annotated pose visualization
- **Features**: Error handling, configurable topics

### `move_me`
- **Function**: Manual robot control
- **Input**: Keyboard arrow keys
- **Output**: Twist commands to `/cmd_vel`
- **Features**: Real-time control, emergency stop

## 🔧 Configuration

### Robot Parameters
```yaml
image_topic: '/harro_camera_feed/image_raw'
pose_output_topic: '/pose_img'
yolo_model: 'yolov8n-pose.pt'
```

### Movement parameters
```yaml
linear_scale: 3.0   # Forward/backward speed
angular_scale: 1.0  # Rotation speed
```

## 🎨 3D Assets

### Robot Components
- **Body**: Main chassis with mounting points
- **Wheels**: 4 independent wheels with realistic physics
- **Camera**: Sensor with proper field of view

### Environmental Models
- **Human Poses**: 6 different human activities
- **Trees**: 2 varieties with realistic textures
- **Textures**: High-quality materials for all models

## 🔬 Technical Achievements

1. **End-to-End Pipeline**: From 3D design to AI-powered robotics
2. **Real-time Processing**: Live pose detection at video frame rates
3. **Modular Architecture**: Cleanly separated components and interfaces
4. **Professional Structure**: Proper ROS2 package organization
5. **Simulation Fidelity**: Realistic physics and environmental modeling

## 🚦 Running the Complete System

Launch everything in sequence:
```bash
# Start Gazebo with robot
ros2 launch my_harro_robot gazebo.launch.py

# Add environmental models
ros2 launch my_harro_robot spawn_models.launch.py

# Start pose detection
ros2 run my_harro_robot pose_detection_node

# Enable robot control
ros2 run my_harro_robot move_me

# View pose detection results
ros2 run rqt_image_view rqt_image_view /pose_img
```

## 🤝 Contributing

Contributions welcome! Areas for enhancement:
- Additional pose detection models
- Autonomous navigation features
- Advanced environmental interactions
- Performance optimizations

## 📈 Future Enhancements

- [ ] Autonomous person-following behavior
- [ ] Multi-robot coordination
- [ ] Advanced pose analysis and gestures
- [ ] Real hardware deployment
- [ ] Web-based control interface
- [ ] Machine learning for behavior prediction

## 📧 Contact

**Phani Chaitanya** - [phanichaitanya63@gmail.com](mailto:phanichaitanya63@gmail.com)

**Portfolio**: https://phani943.github.io/Profile/  
**Repository**: https://github.com/Phani943/Pose_Detection_with_Robot

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🏆 Acknowledgments

- **Blender Foundation** for the excellent 3D modeling software
- **Open Source Robotics Foundation** for ROS2
- **Ultralytics** for YOLOv8 pose detection model
- **Gazebo** simulation community

---

⭐ **If you found this project interesting, please star the repository!**

🎬 **Watch the demo videos to see Harro Robot in action!**
