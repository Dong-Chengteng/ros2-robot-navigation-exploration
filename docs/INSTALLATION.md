# Installation Guide

This guide will help you install and configure the ROS2 Robot Navigation and Exploration System on Ubuntu 22.04.

## System Requirements

### Hardware Requirements
- **CPU**: Intel i5 or AMD Ryzen 5 or higher
- **Memory**: 8GB RAM (16GB recommended)
- **Storage**: At least 20GB available space
- **Graphics**: OpenGL 3.3 compatible graphics card (for Gazebo simulation)

### Software Requirements
- **Operating System**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **Python**: 3.8 or higher
- **Gazebo**: Garden or newer

## Installation Steps

### 1. System Update

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install necessary system tools
sudo apt install -y curl wget git vim build-essential
```

### 2. Install ROS2 Humble

#### 2.1 Set Up Software Sources

```bash
# Add ROS2 software sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

# Add GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 software sources
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### 2.2 Install ROS2

```bash
# Update package list
sudo apt update

# Install ROS2 Humble
sudo apt install ros-humble-desktop -y

# Install development tools
sudo apt install python3-rosdep python3-colcon-common-extensions -y
```

#### 2.3 Initialize rosdep

```bash
# Initialize rosdep
sudo rosdep init
rosdep update
```

### 3. Install Navigation Related Packages

```bash
# Install Navigation2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y

# Install SLAM tools
sudo apt install ros-humble-slam-toolbox -y

# Install Gazebo
sudo apt install ros-humble-gazebo-ros-pkgs -y

# Install other necessary packages
sudo apt install ros-humble-joint-state-publisher ros-humble-robot-state-publisher -y
sudo apt install ros-humble-xacro -y
sudo apt install ros-humble-rviz2 -y
```

### 4. Install Python Dependencies

```bash
# Install Python package manager
sudo apt install python3-pip -y

# Install Python dependencies
pip3 install numpy scipy opencv-python transforms3d matplotlib
pip3 install scikit-learn pandas plotly
```

### 5. Create Workspace

```bash
# Create ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Set environment variables
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 6. Clone and Build Project

```bash
# Enter src directory
cd ~/ros2_ws/src

# Clone project (replace with your repository address)
git clone https://github.com/yourusername/ros2-smart-exploration.git

# Return to workspace root directory
cd ~/ros2_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install

# Set environment
source install/setup.bash
```

## Verify Installation

### 1. Check ROS2 Environment

```bash
# Check ROS2 version
ros2 --version

# Check installed packages
ros2 pkg list | grep first_pkg
```

### 2. Test Gazebo

```bash
# Launch Gazebo
gazebo --version

# Test Gazebo ROS2 integration
ros2 launch gazebo_ros gazebo.launch.py
```

### 3. Test Project Build

```bash
# Check if package compiled correctly
colcon test --packages-select first_pkg

# View test results
colcon test-result --all
```

## Troubleshooting

### Issue 1: ROS2 Environment Not Properly Set

**Symptoms**: "command not found" when running `ros2` command

**Solution**:
```bash
# Manually set environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Permanently set environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Issue 2: Gazebo Launch Failure

**Symptoms**: Gazebo cannot launch or displays black screen

**Solution**:
```bash
# Install graphics drivers
sudo ubuntu-drivers autoinstall

# Restart system
sudo reboot

# Check OpenGL support
glxinfo | grep "OpenGL version"
```

### Issue 3: Python Package Import Error

**Symptoms**: "ModuleNotFoundError" when running

**Solution**:
```bash
# Check Python version
python3 --version

# Reinstall dependencies
pip3 install --upgrade pip
pip3 install -r requirements.txt

# Check package installation
pip3 list | grep numpy
```

### Issue 4: Build Error

**Symptoms**: `colcon build` fails

**Solution**:
```bash
# Clean build directory
rm -rf build/ install/ log/

# Reinstall dependencies
rosdep install --from-paths src --ignore-src -r -y

# Rebuild
colcon build --symlink-install
```

### Issue 5: Permission Issues

**Symptoms**: Cannot access certain files or directories

**Solution**:
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Set workspace permissions
sudo chown -R $USER:$USER ~/ros2_ws

# Re-login or restart
```

## Development Environment Setup

### 1. Install Development Tools

```bash
# Install code editor
sudo apt install code -y  # VS Code
# or
sudo apt install vim -y   # Vim

# Install Git
sudo apt install git -y

# Configure Git
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

### 2. Install Code Quality Tools

```bash
# Install Python code quality tools
pip3 install flake8 black isort mypy

# Install testing framework
pip3 install pytest pytest-cov
```

### 3. Configure VS Code (Optional)

```bash
# Install ROS2 extensions
code --install-extension ms-vscode.cpptools
code --install-extension ms-python.python
code --install-extension ms-vscode.cmake-tools
```

## Performance Optimization

### 1. System Optimization

```bash
# Increase file descriptor limits
echo "* soft nofile 65536" | sudo tee -a /etc/security/limits.conf
echo "* hard nofile 65536" | sudo tee -a /etc/security/limits.conf

# Optimize memory usage
echo "vm.swappiness=10" | sudo tee -a /etc/sysctl.conf
```

### 2. Gazebo Optimization

```bash
# Set Gazebo environment variables
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/first_pkg/models" >> ~/.bashrc
echo "export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/ros2_ws/src/first_pkg/map" >> ~/.bashrc
```

## Uninstall Guide

If you need to completely uninstall the system:

```bash
# Uninstall ROS2
sudo apt remove ros-humble-* -y
sudo apt autoremove -y

# Remove software sources
sudo rm /etc/apt/sources.list.d/ros2.list
sudo rm /usr/share/keyrings/ros-archive-keyring.gpg

# Remove workspace
rm -rf ~/ros2_ws

# Clean environment variables
sed -i '/ros2/d' ~/.bashrc
sed -i '/ros2_ws/d' ~/.bashrc
```

## Getting Help

If you encounter installation problems:

1. Check the [Troubleshooting](#troubleshooting) section
2. Check [GitHub Issues](https://github.com/yourusername/ros2-smart-exploration/issues)
3. Ask questions in [Discussions](https://github.com/yourusername/ros2-smart-exploration/discussions)
4. Contact maintainers

---

**Note**: This installation guide is based on Ubuntu 22.04 and ROS2 Humble. Other operating systems or ROS2 versions may require different installation steps.
