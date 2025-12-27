# Appendix B: Software Installation

This appendix provides step-by-step installation procedures for essential software tools and frameworks used in Physical AI and humanoid robotics development.

## B.1 Ubuntu 22.04 LTS Installation

Ubuntu 22.04 LTS (Jammy Jellyfish) serves as the recommended operating system for ROS 2 Humble development, offering long-term support until April 2027.

### B.1.1 Download and Verification

**Obtaining the ISO Image:**

1. Visit the official Ubuntu website:
   - URL: https://ubuntu.com/download/desktop
   - Select Ubuntu 22.04.3 LTS (or latest point release)
   - Choose appropriate architecture (amd64 for standard PCs)

2. Download locations:
   - Primary: ubuntu.com (official)
   - Mirrors: Select geographically close mirror for faster downloads
   - Torrent: Available for faster downloads and verification

**Verifying ISO Integrity:**

```bash
# Download SHA256 checksum file
wget https://releases.ubuntu.com/22.04/SHA256SUMS

# Verify downloaded ISO
sha256sum ubuntu-22.04.3-desktop-amd64.iso

# Compare output with SHA256SUMS file
grep ubuntu-22.04.3-desktop-amd64.iso SHA256SUMS
```

Expected checksum format:
```
a4acfda10b18da50e2ec50ccaf860d7f20b389df8765611142305c0e911d16fd *ubuntu-22.04.3-desktop-amd64.iso
```

**Verifying GPG Signature (Optional):**

```bash
# Download signature and signing key
wget https://releases.ubuntu.com/22.04/SHA256SUMS.gpg
gpg --keyid-format long --verify SHA256SUMS.gpg SHA256SUMS

# Import Ubuntu signing keys if needed
gpg --keyserver hkp://keyserver.ubuntu.com --recv-keys 0x46181433FBB75451 0xD94AA3F0EFE21092
```

### B.1.2 Bootable USB Creation

**Using Linux:**

```bash
# Identify USB device
lsblk
# Look for your USB drive (e.g., /dev/sdb)

# Unmount if automatically mounted
sudo umount /dev/sdb*

# Write ISO to USB drive
sudo dd bs=4M if=ubuntu-22.04.3-desktop-amd64.iso of=/dev/sdb status=progress oflag=sync

# Verify write
sudo sync
```

**Using macOS:**

```bash
# Convert ISO to DMG
hdiutil convert ubuntu-22.04.3-desktop-amd64.iso -format UDRW -o ubuntu.dmg

# Identify disk
diskutil list

# Unmount disk
diskutil unmountDisk /dev/diskN

# Write image
sudo dd if=ubuntu.dmg of=/dev/rdiskN bs=1m

# Eject
diskutil eject /dev/diskN
```

**Using Windows:**

Recommended tools:
- **Rufus** (rufus.ie): Most reliable, many options
- **balenaEtcher** (balena.io/etcher): Simple, cross-platform
- **Ventoy** (ventoy.net): Multi-ISO support

Rufus settings:
- Partition scheme: GPT
- Target system: UEFI (non CSM)
- File system: FAT32
- Cluster size: 4096 bytes (default)

### B.1.3 Installation Process

**Boot Configuration:**

1. Insert USB drive and restart computer
2. Enter boot menu (typically F12, F2, F10, or Del key)
3. Select USB drive from boot options
4. Choose "Try or Install Ubuntu" from GRUB menu

**Installation Steps:**

**Step 1: Language and Keyboard Selection**
- Select installation language (English recommended for technical work)
- Choose keyboard layout (test in provided text box)

**Step 2: Installation Type**
- **Normal installation**: Includes web browser, utilities, office software, media players
- **Minimal installation**: Basic desktop, web browser, essential utilities only
- Recommended: Normal installation for development workstations

**Step 3: Updates and Additional Software**
- Check "Download updates while installing Ubuntu"
- Check "Install third-party software for graphics and Wi-Fi hardware"
  - Includes NVIDIA drivers, Wi-Fi firmware, media codecs
  - Required for optimal performance

**Step 4: Disk Partitioning**

Option A: Erase disk and install Ubuntu (simplest)
- Automatic partitioning
- Uses entire drive
- Suitable for dedicated robotics workstation

Option B: Manual partitioning (recommended for advanced users)

Recommended partition scheme:

| Mount Point | Size | Type | Description |
|-------------|------|------|-------------|
| `/boot/efi` | 512 MB | EFI System Partition | Boot loader (UEFI) |
| `/` | 100 GB | ext4 | Root filesystem |
| `/home` | 500+ GB | ext4 | User data, code, datasets |
| `swap` | 16-32 GB | swap | Swap space (equal to RAM) |
| `/data` | Remaining | ext4 | Datasets, models, logs |

Manual partition creation:
```
1. Select "Something else"
2. Create new partition table (if needed): GPT for UEFI
3. Click "+" to add partition
4. Configure size, type, and mount point
5. Repeat for all partitions
6. Select boot loader device (usually /dev/sda for UEFI)
```

Option C: Install alongside existing OS (dual boot)
- Installer detects existing operating systems
- Automatically configures GRUB bootloader
- Allocate at least 100 GB for Ubuntu

**Step 5: User Account Creation**
- Your name: Display name
- Computer name: Hostname (use descriptive name, e.g., robotics-workstation)
- Username: Login name (lowercase, no spaces)
- Password: Strong password (recommended: 16+ characters)
- Option: Require password to log in (recommended)

**Step 6: Installation**
- Review configuration summary
- Click "Install Now" to begin
- Confirm disk changes
- Installation takes 10-30 minutes
- Computer will prompt for restart when complete

**Step 7: First Boot**
- Remove USB drive when prompted
- System will boot into GRUB (if dual-boot) or directly to Ubuntu
- Login with credentials created during installation

### B.1.4 Post-Installation Configuration

**System Updates:**

```bash
# Update package repositories
sudo apt update

# Upgrade installed packages
sudo apt upgrade -y

# Optionally upgrade to new package versions
sudo apt full-upgrade -y

# Remove unnecessary packages
sudo apt autoremove -y

# Reboot to apply kernel updates
sudo reboot
```

**Essential Tools Installation:**

```bash
# Development tools
sudo apt install build-essential git curl wget vim nano htop -y

# Compilation tools
sudo apt install cmake gcc g++ gdb make -y

# Version control and utilities
sudo apt install git-lfs tree tmux screen -y

# Network tools
sudo apt install net-tools openssh-server nmap -y

# Python development
sudo apt install python3-pip python3-venv python-is-python3 -y
```

**NVIDIA Driver Installation (for NVIDIA GPUs):**

```bash
# Detect recommended drivers
ubuntu-drivers devices

# Install recommended driver automatically
sudo ubuntu-drivers autoinstall

# OR install specific version
sudo apt install nvidia-driver-535 -y

# Reboot to load driver
sudo reboot

# Verify installation
nvidia-smi
```

Expected output:
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 535.129.03   Driver Version: 535.129.03   CUDA Version: 12.2   |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  NVIDIA GeForce ...  Off  | 00000000:01:00.0  On |                  N/A |
| 30%   45C    P8    25W / 320W |    512MiB / 12288MiB |      2%      Default |
+-------------------------------+----------------------+----------------------+
```

**System Settings Configuration:**

```bash
# Disable automatic suspend (useful for long-running processes)
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-timeout 0

# Increase inotify watchers (needed for large projects)
echo "fs.inotify.max_user_watches=524288" | sudo tee -a /etc/sysctl.conf
sudo sysctl -p

# Configure swappiness (reduce swap usage for performance)
echo "vm.swappiness=10" | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

**Terminal Customization:**

```bash
# Install zsh and oh-my-zsh (optional but recommended)
sudo apt install zsh -y
sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

# Install useful aliases
cat << 'EOF' >> ~/.bashrc
# Custom aliases for robotics development
alias src='source ~/.bashrc'
alias ros2ws='cd ~/ros2_ws && source install/setup.bash'
alias cb='cd ~/ros2_ws && colcon build --symlink-install'
alias ct='cd ~/ros2_ws && colcon test'
EOF

source ~/.bashrc
```

---

## B.2 ROS 2 Humble/Iron Installation

ROS 2 (Robot Operating System 2) is the primary middleware for robotics development. Humble Hawksbill is the LTS release (supported until 2027), while Iron Irwini offers newer features.

### B.2.1 Adding ROS 2 Repositories

**Set Locale:**

```bash
# Ensure UTF-8 locale
locale

# If not set, configure UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

**Add ROS 2 Repository:**

```bash
# Enable Ubuntu Universe repository
sudo apt install software-properties-common -y
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package cache
sudo apt update
```

### B.2.2 Installing ROS 2 Packages

**ROS 2 Humble (LTS - Recommended):**

```bash
# Desktop Install (Recommended): ROS, RViz, demos, tutorials
sudo apt install ros-humble-desktop -y

# ROS-Base Install (Minimal): Communication libraries, no GUI tools
# sudo apt install ros-humble-ros-base -y

# Development tools
sudo apt install ros-dev-tools -y

# Additional useful packages
sudo apt install ros-humble-rviz2 ros-humble-rqt* -y
sudo apt install ros-humble-gazebo-ros-pkgs -y
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y
sudo apt install ros-humble-slam-toolbox -y
```

**ROS 2 Iron (Latest Features):**

```bash
# Change 'humble' to 'iron' in repository
sudo sed -i 's/humble/iron/g' /etc/apt/sources.list.d/ros2.list
sudo apt update

# Install ROS 2 Iron
sudo apt install ros-iron-desktop -y
sudo apt install ros-dev-tools -y
```

**Package Size Reference:**
- ros-humble-ros-base: ~200 MB
- ros-humble-desktop: ~1.5 GB
- Additional packages: Variable (50 MB - 500 MB each)

### B.2.3 Environment Setup

**Automatic Sourcing:**

```bash
# Add ROS 2 to bashrc for automatic sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --version
# Expected: ros2 cli version: ros2 doctor --version X.Y.Z
```

**Workspace Setup:**

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build workspace (empty for now)
colcon build

# Source workspace overlay
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Environment Variables Configuration:**

```bash
# Add useful ROS 2 environment variables to bashrc
cat << 'EOF' >> ~/.bashrc
# ROS 2 Configuration
export ROS_DOMAIN_ID=0  # Change if running multiple robots
export ROS_LOCALHOST_ONLY=0  # Set to 1 to restrict to localhost
export RCUTILS_COLORIZED_OUTPUT=1  # Colored log output
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
EOF

source ~/.bashrc
```

**Domain ID Guidelines:**

| Domain ID | Use Case |
|-----------|----------|
| 0 | Default, single robot/developer |
| 1-99 | Individual robots in multi-robot system |
| 100-232 | Reserved for specific applications |

### B.2.4 Verifying Installation

**Test 1: Check ROS 2 Installation:**

```bash
# List available packages
ros2 pkg list

# Should show hundreds of packages including:
# action_msgs
# ament_cmake
# geometry_msgs
# rclpy
# sensor_msgs
# std_msgs
```

**Test 2: Run Demo Nodes:**

Terminal 1:
```bash
ros2 run demo_nodes_cpp talker
```

Terminal 2:
```bash
ros2 run demo_nodes_py listener
```

Expected output in Terminal 2:
```
[INFO] [1703123456.789]: I heard: [Hello World: 1]
[INFO] [1703123457.789]: I heard: [Hello World: 2]
```

**Test 3: ROS 2 Doctor:**

```bash
# Comprehensive system check
ros2 doctor

# Should report no warnings or errors
# Check network configuration, middleware, etc.
```

**Test 4: Launch RViz2:**

```bash
rviz2
```

Should open RViz2 GUI without errors.

**Common Installation Issues:**

| Issue | Solution |
|-------|----------|
| `ros2: command not found` | Source setup.bash: `source /opt/ros/humble/setup.bash` |
| Package not found | Update apt cache: `sudo apt update` |
| Permission denied | Add user to dialout group: `sudo usermod -aG dialout $USER` |
| Slow discovery | Check firewall: `sudo ufw allow 7400:7500/udp` |

---

## B.3 Gazebo and Unity Setup

Simulation environments enable safe development and testing before deploying to physical robots.

### B.3.1 Gazebo Installation Options

**Gazebo Classic (Gazebo 11) - Legacy:**

```bash
# Install Gazebo 11 (included with ROS 2 Humble)
sudo apt install gazebo ros-humble-gazebo-ros-pkgs -y

# Verify installation
gazebo --version
# Expected: Gazebo multi-robot simulator, version 11.10.2
```

**Gazebo (New Generation) - Recommended:**

Gazebo (formerly Ignition Gazebo) is the modern replacement for Gazebo Classic.

```bash
# Install Gazebo Garden (compatible with ROS 2 Humble)
sudo apt install ros-humble-ros-gz -y

# Or install full Gazebo Garden
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
sudo apt install gz-garden -y

# Verify installation
gz sim --version
```

**Testing Gazebo:**

```bash
# Launch empty world (Gazebo Classic)
gazebo

# Launch example world (New Gazebo)
gz sim shapes.sdf
```

**Gazebo Version Comparison:**

| Feature | Gazebo Classic 11 | Gazebo Garden |
|---------|-------------------|---------------|
| Physics engines | ODE, Bullet, Simbody, DART | DART, TPE |
| Rendering | OGRE 1.x | OGRE 2.x |
| Sensors | Basic | Advanced (GPU-accelerated) |
| Performance | Moderate | High |
| Plugin system | Legacy | Modern, modular |
| ROS 2 support | ros_gz_bridge | Native integration |

### B.3.2 Unity Hub and Unity Editor

Unity provides photorealistic simulation and synthetic data generation capabilities.

**Unity Hub Installation:**

```bash
# Download Unity Hub
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage

# Make executable
chmod +x UnityHub.AppImage

# Run Unity Hub
./UnityHub.AppImage
```

**Alternative: Manual Installation:**

```bash
# Install dependencies
sudo apt install libgconf-2-4 libglu1-mesa libcanberra-gtk-module -y

# Download and install
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHubSetup.AppImage
chmod +x UnityHubSetup.AppImage
./UnityHubSetup.AppImage
```

**Unity Editor Installation via Hub:**

1. Sign in or create Unity account
2. Activate license:
   - Personal (free): For learning and small projects
   - Plus/Pro: For larger projects (subscription)
3. Install Unity Editor:
   - Recommended version: 2022.3 LTS
   - Add modules: Linux Build Support, Documentation
4. Installation location: `~/Unity/Hub/Editor/2022.3.X/`

**Unity Editor System Requirements:**

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| OS | Ubuntu 20.04+ | Ubuntu 22.04 |
| CPU | X64 with SSE2 | 8+ cores |
| RAM | 8 GB | 16 GB |
| GPU | OpenGL 3.2+ | Vulkan support |
| Storage | 10 GB | 20 GB |

### B.3.3 Unity Robotics Hub Packages

Unity Robotics Hub enables ROS 2 integration with Unity for simulation and visualization.

**Installation Steps:**

1. **Create New Unity Project:**
   - Open Unity Hub
   - Click "New Project"
   - Select "3D (URP)" template for better performance
   - Name project (e.g., "RoboticsSimulation")
   - Create project

2. **Install Unity Robotics Hub Packages:**

   Window > Package Manager > Add package from git URL

   Add the following packages:
   ```
   https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
   https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
   ```

3. **Configure ROS-TCP Endpoint:**

   Robotics > ROS Settings
   - Protocol: ROS 2
   - ROS IP Address: 127.0.0.1 (or workstation IP)
   - ROS Port: 10000
   - Show HUD: Enabled (for debugging)

4. **Install ROS 2 TCP Endpoint (on Ubuntu):**

   ```bash
   # In ROS 2 workspace
   cd ~/ros2_ws/src
   git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint
   cd ~/ros2_ws
   colcon build --packages-select ros_tcp_endpoint
   source install/setup.bash

   # Launch TCP endpoint
   ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
   ```

**Testing Unity-ROS 2 Connection:**

Create simple publisher in Unity (C# script):
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class RosPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/unity_test";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topicName);
    }

    void Update()
    {
        StringMsg message = new StringMsg("Hello from Unity!");
        ros.Publish(topicName, message);
    }
}
```

Subscribe in ROS 2:
```bash
ros2 topic echo /unity_test
```

### B.3.4 Testing Simulation Environments

**Test 1: Gazebo with ROS 2:**

```bash
# Launch example robot in Gazebo
ros2 launch gazebo_ros gazebo.launch.py

# In another terminal, spawn a simple model
ros2 run gazebo_ros spawn_entity.py -entity my_robot -database turtlebot3_waffle
```

**Test 2: Multi-Robot Simulation:**

```bash
# Launch Gazebo with custom world
gz sim -r multi_robot_world.sdf

# Verify robots are spawned
gz topic -l
```

**Test 3: Unity URDF Import:**

1. In Unity: Robotics > Import URDF
2. Select URDF file (e.g., robot.urdf)
3. Configure import settings:
   - Axis: Y-up (Unity convention)
   - Mesh scale: 1.0
   - Generate colliders: Enabled
4. Click Import
5. Verify robot appears in scene hierarchy

**Performance Benchmarking:**

```bash
# Gazebo Classic performance test
gazebo --verbose worlds/shapes.world

# Check FPS and RTF (Real-Time Factor)
# RTF = 1.0: Real-time simulation
# RTF > 1.0: Faster than real-time
# RTF < 1.0: Slower than real-time (computational bottleneck)
```

Monitor with:
```bash
gz stats
```

---

## B.4 NVIDIA Isaac Installation

NVIDIA Isaac provides GPU-accelerated robotics simulation and AI tools.

### B.4.1 System Requirements

**Hardware Requirements:**

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| GPU | RTX 2070 | RTX 4070 Ti or higher |
| VRAM | 8 GB | 12 GB+ |
| CPU | 4-core | 8-core+ |
| RAM | 16 GB | 32 GB |
| Storage | 50 GB | 100 GB SSD |
| OS | Ubuntu 20.04/22.04 | Ubuntu 22.04 |

**Software Prerequisites:**

```bash
# Install dependencies
sudo apt update
sudo apt install build-essential git git-lfs curl wget -y

# Verify NVIDIA driver
nvidia-smi

# Driver version 525+ required for Isaac Sim 2023+
```

### B.4.2 Omniverse Launcher Installation

**Download and Install:**

```bash
# Download Omniverse Launcher AppImage
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Make executable
chmod +x omniverse-launcher-linux.AppImage

# Run launcher
./omniverse-launcher-linux.AppImage
```

**First-Time Setup:**

1. Sign in with NVIDIA account (create if needed)
2. Accept End User License Agreement
3. Launcher will download initial components (~500 MB)
4. Configure installation directory (default: `~/.local/share/ov`)

**Library Configuration:**

Navigate to Library tab and install:
- Nucleus (local server for asset management)
- Cache (improves loading times)
- USD Composer (optional, for scene creation)

### B.4.3 Isaac Sim Installation

**Installation via Omniverse Launcher:**

1. Go to "Exchange" tab
2. Search for "Isaac Sim"
3. Click "Install" on Isaac Sim 2023.1.1 (or latest)
4. Select installation path (requires ~30 GB)
5. Wait for installation (15-45 minutes depending on connection)

**Command-Line Installation (Alternative):**

```bash
# Set installation directory
ISAAC_SIM_PATH="${HOME}/.local/share/ov/pkg/isaac_sim-2023.1.1"

# Download via Omniverse CLI
omni repo add isaac-sim
omni repo update isaac-sim
omni install isaac-sim
```

**Post-Installation Configuration:**

```bash
# Add Isaac Sim to PATH
echo "export ISAAC_SIM_PATH=\"${HOME}/.local/share/ov/pkg/isaac_sim-2023.1.1\"" >> ~/.bashrc
echo "export PATH=\"\${ISAAC_SIM_PATH}:\${PATH}\"" >> ~/.bashrc
source ~/.bashrc

# Run setup script
cd ${ISAAC_SIM_PATH}
./setup_conda_env.sh

# Verify installation
./isaac-sim.sh --help
```

**First Launch:**

```bash
# Launch Isaac Sim GUI
./isaac-sim.sh

# Or headless mode for server
./isaac-sim.sh --headless
```

First launch will:
- Compile shaders (~5-10 minutes)
- Download default assets
- Initialize Nucleus local server

**Testing Isaac Sim:**

```bash
# Run example scene
cd ${ISAAC_SIM_PATH}
./python.sh standalone_examples/api/omni.isaac.core/add_cubes.py
```

### B.4.4 Isaac ROS Installation

Isaac ROS provides GPU-accelerated ROS 2 packages for perception and navigation.

**Prerequisites:**

```bash
# Install NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt update
sudo apt install nvidia-docker2 -y
sudo systemctl restart docker

# Verify Docker GPU access
sudo docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

**Isaac ROS Common Setup:**

```bash
# Create workspace
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src

# Clone Isaac ROS Common
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# Configure workspace
cd ~/isaac_ros_ws
./src/isaac_ros_common/scripts/run_dev.sh
```

**Installing Isaac ROS Packages:**

```bash
# Inside Docker container (after run_dev.sh)
cd /workspaces/isaac_ros_ws/src

# Example: Install DNN inference package
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference.git

# Install dependencies
cd /workspaces/isaac_ros_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash
```

**Available Isaac ROS Packages:**

| Package | Description | Use Case |
|---------|-------------|----------|
| isaac_ros_dnn_inference | TensorRT inference | Object detection, segmentation |
| isaac_ros_image_pipeline | Image processing | Rectification, debayering |
| isaac_ros_nvblox | 3D reconstruction | Mapping, navigation |
| isaac_ros_visual_slam | Visual odometry | Localization |
| isaac_ros_apriltag | AprilTag detection | Fiducial tracking |
| isaac_ros_depth_segmentation | Depth segmentation | Scene understanding |

**Testing Isaac ROS:**

```bash
# Example: Run AprilTag detection
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py
```

---

## B.5 Development Tools and Libraries

Essential development tools enhance productivity and debugging capabilities.

### B.5.1 VS Code Setup

**Installation:**

```bash
# Download and install VS Code
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /usr/share/keyrings/
sudo sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'

sudo apt update
sudo apt install code -y
```

**Essential Extensions:**

Install via Extensions marketplace (Ctrl+Shift+X) or command line:

```bash
# ROS 2 development
code --install-extension ms-iot.vscode-ros
code --install-extension ms-python.python
code --install-extension ms-vscode.cpptools

# Git integration
code --install-extension eamodio.gitlens

# Remote development
code --install-extension ms-vscode-remote.remote-ssh

# Additional useful extensions
code --install-extension ms-vscode.cmake-tools
code --install-extension twxs.cmake
code --install-extension ms-python.pylint
code --install-extension ms-toolsai.jupyter
```

**VS Code Configuration for ROS 2:**

Create `.vscode/settings.json` in workspace:

```json
{
  "ros.distro": "humble",
  "python.autoComplete.extraPaths": [
    "/opt/ros/humble/lib/python3.10/site-packages"
  ],
  "python.analysis.extraPaths": [
    "/opt/ros/humble/lib/python3.10/site-packages"
  ],
  "C_Cpp.default.includePath": [
    "/opt/ros/humble/include/**"
  ],
  "cmake.configureOnOpen": false
}
```

### B.5.2 Python Environment Management

**Virtual Environments:**

```bash
# Create virtual environment for robot projects
python3 -m venv ~/robot_env

# Activate environment
source ~/robot_env/bin/activate

# Install common packages
pip install numpy scipy matplotlib opencv-python
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install transformers accelerate

# Deactivate when done
deactivate
```

**Conda Alternative:**

```bash
# Install Miniconda
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh

# Create environment
conda create -n robotics python=3.10
conda activate robotics

# Install packages
conda install numpy scipy matplotlib
pip install opencv-python
```

**Requirements File Management:**

```bash
# Generate requirements.txt
pip freeze > requirements.txt

# Install from requirements
pip install -r requirements.txt
```

### B.5.3 RViz2 and Foxglove

**RViz2 (already installed with ROS 2 Desktop):**

```bash
# Launch RViz2
rviz2

# Launch with configuration
rviz2 -d config.rviz
```

**Foxglove Studio Installation:**

```bash
# Download Foxglove Studio
wget https://github.com/foxglove/studio/releases/download/v1.87.0/foxglove-studio-1.87.0-linux-amd64.deb

# Install
sudo apt install ./foxglove-studio-1.87.0-linux-amd64.deb

# Launch
foxglove-studio
```

**Foxglove Bridge for ROS 2:**

```bash
# Install Foxglove Bridge
sudo apt install ros-humble-foxglove-bridge -y

# Launch bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

# Connect Foxglove Studio to ws://localhost:8765
```

**Foxglove vs RViz2:**

| Feature | RViz2 | Foxglove Studio |
|---------|-------|-----------------|
| Platform | Linux (primary) | Cross-platform |
| Performance | Native | Web-based |
| 3D rendering | OGRE | Three.js |
| Data sources | ROS 2 only | ROS 1/2, MCAP, more |
| Extensibility | Plugins (C++) | Extensions (TypeScript) |
| Recording | ros2 bag | Built-in, MCAP format |

### B.5.4 Git Configuration

**Basic Configuration:**

```bash
# Set identity
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# Set default editor
git config --global core.editor "vim"

# Set default branch name
git config --global init.defaultBranch main

# Enable color output
git config --global color.ui auto

# Configure line endings
git config --global core.autocrlf input
```

**Git LFS for Large Files:**

```bash
# Install Git LFS
sudo apt install git-lfs -y

# Initialize
git lfs install

# Track large files (in repository)
git lfs track "*.bag"
git lfs track "*.pth"
git lfs track "*.onnx"
git lfs track "*.urdf"

# Commit .gitattributes
git add .gitattributes
git commit -m "Configure Git LFS"
```

**Useful Git Aliases:**

```bash
# Add to ~/.gitconfig
git config --global alias.st status
git config --global alias.co checkout
git config --global alias.br branch
git config --global alias.ci commit
git config --global alias.unstage 'reset HEAD --'
git config --global alias.last 'log -1 HEAD'
git config --global alias.visual 'log --graph --oneline --all'
```

**SSH Key Setup for GitHub:**

```bash
# Generate SSH key
ssh-keygen -t ed25519 -C "your.email@example.com"

# Start ssh-agent
eval "$(ssh-agent -s)"

# Add key
ssh-add ~/.ssh/id_ed25519

# Copy public key to clipboard
cat ~/.ssh/id_ed25519.pub

# Add to GitHub: Settings > SSH and GPG keys > New SSH key
```

---

## Summary

This appendix covered comprehensive software installation procedures for Physical AI development:

- **Ubuntu 22.04 LTS**: Complete installation process, verification, and post-installation configuration
- **ROS 2 Humble/Iron**: Repository setup, package installation, environment configuration, and verification
- **Simulation Environments**: Gazebo Classic, new Gazebo, and Unity with Robotics Hub integration
- **NVIDIA Isaac**: Isaac Sim via Omniverse and Isaac ROS GPU-accelerated packages
- **Development Tools**: VS Code with ROS extensions, Python environment management, visualization tools (RViz2, Foxglove), and Git configuration

Proper installation of these tools provides a complete development environment for robotics research and education. Always verify each installation step before proceeding to ensure a stable foundation.
