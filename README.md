# 🚁🍅 Autonomous Drone Tomato Farm Vision System

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![PX4](https://img.shields.io/badge/PX4-v1.14-green.svg)](https://px4.io/)

**Autonomous drone system for real-time tomato detection, classification, and tracking in custom greenhouse environments using PX4-ROS2-Gazebo simulation with custom YOLO11 models.**

> **Built upon**: [PX4-ROS2-Gazebo-YOLOv8](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8) by monemati  
> **Tomato Farm Models**: Generated using [aoc_tomato_farm](https://github.com/LCAS/aoc_tomato_farm) by LCAS

## 🎯 Project Overview

This project extends the [PX4-ROS2-Gazebo-YOLOv8](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8) framework with custom tomato farm environments and specialized YOLO11 detection models. The system combines autonomous drone flight control with state-of-the-art computer vision to create a comprehensive tomato farm monitoring solution.

The custom greenhouse environment was generated using the [aoc_tomato_farm](https://github.com/LCAS/aoc_tomato_farm) package, creating realistic 8×8 tomato plant layouts with proper lighting and structure for optimal computer vision performance.

### ✨ Key Features

- 🚁 **Autonomous Flight**: PX4-based drone control with keyboard and programmatic interfaces
- 🍅 **Advanced Detection**: Custom-trained YOLO11 model with 90.1% mAP for tomato segmentation
- 🎯 **Multi-Class Classification**: Detects green, half-ripened, and fully-ripened tomatoes
- 🏠 **Custom Greenhouse Environment**: Realistic tomato farm generated with aoc_tomato_farm package
- 📊 **Real-time Visualization**: Live detection results in RQT and OpenCV windows
- 🔄 **ROS2 Integration**: Complete ROS2 ecosystem with topic-based communication
- 📈 **Analytics**: Tomato counting, tracking, and ripeness statistics
- 🌱 **Scalable Farm Layout**: 8×8 grid layout with 64+ tomato plants and greenhouse structure

## 🏗️ System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   PX4 SITL      │    │   Gazebo Sim     │    │  YOLO11 Model   │
│  (Flight Control)│◄──►│ (Greenhouse Env) │◄──►│ (Tomato Vision) │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         ▲                        ▲                       ▲
         │                        │                       │
         ▼                        ▼                       ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  ROS2 Bridge    │    │   Camera Feed    │    │  Detection UI   │
│  (Communication)│    │ (Image Stream)   │    │ (RQT/OpenCV)    │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## 🚀 Installation & Setup

### Prerequisites

- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **Python**: 3.10+
- **GPU**: NVIDIA GPU recommended (RTX 4060+ optimal)
- **RAM**: 16GB+ recommended
- **Storage**: 10GB+ free space

### Step 1: Clone This Repository

```bash
# Clone this repository to your home directory
cd ~
git clone https://github.com/YOUR_USERNAME/drone-farm-vision-system.git drone_farm_vision_ws
cd drone_farm_vision_ws
```

### Step 2: Set Up Python Virtual Environment

```bash
# Create virtual environment
python3 -m venv dfv_env

# Activate virtual environment
source dfv_env/bin/activate

# Install Python dependencies
pip install -r requirements.txt
```

### Step 3: Install PX4-Autopilot

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl
```

### Step 4: Install ROS2 Humble (if not already installed)

```bash
cd ~
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Step 5: Setup Micro XRCE-DDS Agent & Client

```bash
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### Step 6: Build ROS2 Workspace

```bash
cd ~/drone_farm_vision_ws

# Build the main workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Install additional ROS2 packages
sudo apt install ros-humble-ros-gzgarden ros-humble-cv-bridge ros-humble-image-transport
```

### Step 7: Configure Environment

```bash
# Copy models to Gazebo directory
mkdir -p ~/.gz/models
cp -r models/* ~/.gz/models/

# Copy world files to PX4
cp worlds/*.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/

# Add environment variables to bashrc
echo "export GZ_SIM_RESOURCE_PATH=~/.gz/models" >> ~/.bashrc
```

### Step 8: Verify Installation

```bash
# Source the environment setup
cd ~/drone_farm_vision_ws
source setup_env.sh

# You should see:
# ✅ Main workspace sourced successfully
# ✅ Environment configured for drone_farm_vision_ws
# Virtual environment: /path/to/dfv_env/bin/python
# ROS2 packages: 2 px4 packages available
```

## 🎮 Running the System

You need **5 separate terminals** for the complete system. Follow this exact sequence:

### Terminal 1: XRCE-DDS Agent
```bash
cd ~/Micro-XRCE-DDS-Agent
MicroXRCEAgent udp4 -p 8888
```

### Terminal 2: PX4 Simulation with Custom Tomato Farm
```bash
cd ~/PX4-Autopilot
~/drone_farm_vision_ws/scripts/launch_tomato_two_step.sh
```

### Terminal 3: Camera Bridge
```bash
source /opt/ros/humble/setup.bash
ros2 run ros_gz_image image_bridge /camera --ros-args -p use_sim_time:=true
```

### Terminal 4: Tomato Detection System
```bash
source ~/drone_farm_vision_ws/setup_env.sh
cd ~/drone_farm_vision_ws/scripts
python final_tomato_detection.py
```

### Terminal 5: Drone Control Interface
```bash
source ~/drone_farm_vision_ws/setup_env.sh
cd ~/drone_farm_vision_ws/scripts
python keyboard-mavsdk-test.py
```

> **Note**: Make sure to wait for each terminal to fully initialize before starting the next one. Terminal 2 should show "LAUNCH COMPLETE!" before proceeding.

## 🎮 Flight Controls

| Key | Action |
|-----|--------|
| `R` | Arm drone |
| `W` | Move forward |
| `S` | Move backward |
| `A` | Move left |
| `D` | Move right |
| `↑` | Increase altitude |
| `↓` | Decrease altitude |
| `←` `→` | Rotate left/right |
| `L` | Land drone |

## 🍅 Tomato Detection Model

This system uses a custom-trained YOLO11 model for tomato segmentation:

- **Model**: YOLO11x-seg
- **Dataset**: Laboro Tomato Dataset
- **Performance**: 90.1% mAP@0.5
- **Classes**: Green, Half-ripened, Fully-ripened
- **Repository**: [yolo11-tomato-segmentation](https://github.com/Fonyuy45/yolo11-tomato-segmentation)

### Detection Capabilities

- 🟢 **Green Tomatoes**: Unripe fruits for growth monitoring
- 🟡 **Half-ripened**: Transition stage tracking
- 🔴 **Fully-ripened**: Ready-to-harvest identification
- 📊 **Instance Segmentation**: Precise boundary detection
- 🔢 **Counting**: Real-time tomato quantity tracking

## 🏠 Custom Greenhouse Environment

The simulation features a custom greenhouse environment generated using the [aoc_tomato_farm](https://github.com/LCAS/aoc_tomato_farm) package:

- **Dimensions**: 14m × 7m greenhouse structure
- **Layout**: 8×8 grid of tomato plants (64 plants total)
- **Components**: Individual tomato plants, flowerpots, soil beds, overhead lighting
- **Structure**: Glass walls and metal framework for realistic greenhouse appearance
- **Lighting**: Optimized lighting conditions for computer vision detection
- **Generation**: Parametrically generated using Jupyter notebooks from aoc_tomato_farm
- **Compatibility**: Both Gazebo Garden/Harmonic compatible models

## 📁 Project Structure

This repository follows **standard ROS2 workspace conventions**:

```
drone_farm_vision_ws/
├── src/                            # ROS2 packages (standard location)
│   ├── px4_msgs/                   # PX4 message definitions
│   └── px4_ros_com/                # PX4-ROS2 communication bridge
├── scripts/                        # Standalone Python scripts
│   ├── final_tomato_detection.py   # Main detection script
│   ├── keyboard-mavsdk-test.py     # Flight control interface
│   ├── launch_tomato_two_step.sh   # System launcher
│   ├── KeyPressModule.py           # Keyboard input handler
│   └── uav_camera_det.py           # Camera detection utilities
├── worlds/                         # Gazebo world files
│   ├── default.sdf                 # Default world
│   ├── default_docker.sdf          # Docker-compatible world
│   └── tomato_farm_px4_complete.sdf # Custom tomato farm
├── models/                         # Gazebo models and AI models
│   ├── tomato_detection/           # YOLO models
│   │   ├── best.pt                 # Main YOLO11 model
│   │   └── yolo11x-seg-tomato.pt   # Segmentation model
│   ├── tomato_103118/              # Gazebo tomato models
│   ├── flowerpot_103118/           # Gazebo flowerpot models
│   └── ... (200+ custom models)    # Generated farm components
├── launch/                         # ROS2 launch files
├── config/                         # Configuration files
├── dfv_env/                        # Python virtual environment
├── build/                          # Build artifacts (auto-generated)
├── install/                        # Installed packages (auto-generated)
├── log/                            # Build logs (auto-generated)
├── README.md                       # This file
├── requirements.txt                # Python dependencies
└── setup_env.sh                   # Environment configuration script
```

## 🔧 Configuration

### Detection Parameters

Edit `scripts/final_tomato_detection.py` to customize:

```python
# Detection confidence threshold
conf_threshold = 0.4

# IoU threshold for NMS
iou_threshold = 0.5

# Model selection
model_path = "models/tomato_detection/yolo11x-seg-tomato.pt"

# Detection classes
class_names = {
    0: 'fully_ripened',
    1: 'green', 
    2: 'half_ripened'
}
```

### Flight Parameters

Modify `scripts/keyboard-mavsdk-test.py` for flight behavior:

```python
# Flight altitude (meters)
flight_altitude = 3.0

# Movement speed
movement_speed = 1.0

# Takeoff altitude
takeoff_altitude = 2.5
```

## 📊 Performance Metrics

### Detection Performance
- **mAP@0.5**: 90.1%
- **Inference Speed**: ~28ms per frame
- **Detection Rate**: 30+ FPS
- **Segmentation Accuracy**: 89.8%

### System Performance
- **Flight Stability**: ±0.1m position accuracy
- **Real-time Processing**: 30 Hz camera feed
- **Memory Usage**: ~2GB GPU, ~4GB RAM
- **CPU Load**: ~40% (Intel i7/AMD Ryzen 5+)

## 🛠️ Development

### Adding New Detection Classes

1. Retrain YOLO model with new classes
2. Update `class_names` in detection script
3. Modify visualization colors
4. Update statistics calculation

### Custom Flight Missions

```python
# Example autonomous mission
async def survey_mission():
    # Takeoff
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(3.0)
    await drone.action.takeoff()
    
    # Survey pattern
    waypoints = generate_survey_pattern()
    for waypoint in waypoints:
        await drone.action.goto_location(*waypoint)
        # Capture and analyze
        detections = await capture_and_detect()
        log_detections(detections)
```

## 📈 Analytics and Monitoring

The system provides real-time analytics:

- **Detection Counts**: Live count of each tomato class
- **Spatial Distribution**: Tomato locations in the greenhouse
- **Ripeness Progression**: Temporal tracking of fruit development
- **Coverage Statistics**: Area surveyed and detection density

## 🤝 Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📞 Contact

- **Author**: Dieudonne Fonyuy Y.
- **Email**: dieudonne.yufonyuy@gmail.com
- **LinkedIn**: [Dieudonne Yufonyuy](https://www.linkedin.com/in/dieudonne-yufonyuy)
- **Project Link**: [https://github.com/YOUR_USERNAME/drone-farm-vision-system](https://github.com/YOUR_USERNAME/drone-farm-vision-system)

---

⭐ **Star this repository if you found it helpful!**

🚁🍅 *Autonomous tomato farming with precision agriculture and computer vision*

> **Built with**: PX4 Autopilot + ROS2 Humble + Gazebo + YOLO11 + Custom Tomato Farm Models

## 🌟 Related Projects & Acknowledgments

This project builds upon and integrates several excellent open-source projects:

### Base Framework
- **[PX4-ROS2-Gazebo-YOLOv8](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8)** by monemati  
  *The foundational framework for PX4-ROS2-Gazebo integration with YOLO object detection*

### Tomato Farm Environment
- **[aoc_tomato_farm](https://github.com/LCAS/aoc_tomato_farm)** by LCAS  
  *Tomato farm/glasshouse generator compatible with both Gazebo and Unity*  
  *Citation*: Flores, J.P.E., Yilmaz, A., Avendaño, L.A.S., Cielniak, G. (2024). Comparative Analysis of Unity and Gazebo Simulators for Digital Twins of Robotic Tomato Harvesting Scenarios. TAROS 2024.

### Computer Vision Model
- **[YOLO11 Tomato Segmentation](https://github.com/Fonyuy45/yolo11-tomato-segmentation)**  
  *Custom-trained YOLO11 model achieving 90.1% mAP for tomato detection and segmentation*

## 🙏 Acknowledgments

- **[PX4 Development Team](https://px4.io/)** for the excellent autopilot system
- **[ROS2 Community](https://ros.org/)** for the robotic middleware framework  
- **[Ultralytics](https://ultralytics.com/)** for the YOLO11 framework
- **[Gazebo Team](https://gazebosim.org/)** for the simulation environment
- **[LCAS Research Group](https://github.com/LCAS)** for the tomato farm generation tools
- **[Roboflow](https://roboflow.com/)** for the Laboro Tomato dataset used in model training
- **monemati** for the original PX4-ROS2-Gazebo-YOLOv8 integration