# Appendix A: Hardware Setup Guides

This appendix provides detailed hardware setup procedures for Physical AI and humanoid robotics development. Follow these guides to configure your development workstation, embedded systems, sensors, and robot platforms.

## A.1 Digital Twin Workstation Configuration

### A.1.1 Recommended System Specifications

A capable workstation is essential for running simulation environments, training models, and developing robot software. The following specifications represent recommended configurations for different use cases.

**Minimum Configuration (Development Only)**
- CPU: Intel Core i5-12400 or AMD Ryzen 5 5600X (6 cores)
- RAM: 16 GB DDR4-3200
- GPU: NVIDIA RTX 3060 (12 GB VRAM)
- Storage: 512 GB NVMe SSD
- OS: Ubuntu 22.04 LTS

**Recommended Configuration (Development + Light Simulation)**
- CPU: Intel Core i7-13700K or AMD Ryzen 7 7700X (8-16 cores)
- RAM: 32 GB DDR4-3600 or DDR5-5600
- GPU: NVIDIA RTX 4070 Ti (12 GB VRAM)
- Storage: 1 TB NVMe SSD (Gen 4)
- OS: Ubuntu 22.04 LTS

**High-Performance Configuration (Heavy Simulation + Training)**
- CPU: Intel Core i9-13900K or AMD Ryzen 9 7950X (16-24 cores)
- RAM: 64 GB DDR5-6000
- GPU: NVIDIA RTX 4090 (24 GB VRAM) or RTX 6000 Ada
- Storage: 2 TB NVMe SSD (Gen 4) + 2 TB secondary storage
- OS: Ubuntu 22.04 LTS

### A.1.2 GPU Requirements and Selection

GPU capabilities directly impact simulation performance and model training times. The following table compares suitable NVIDIA GPUs for Physical AI development.

| GPU Model | VRAM | CUDA Cores | Tensor Cores | Use Case |
|-----------|------|------------|--------------|----------|
| RTX 3060 | 12 GB | 3584 | Gen 3 | Basic development, small scenes |
| RTX 4060 Ti | 16 GB | 4352 | Gen 4 | Development, moderate simulation |
| RTX 4070 Ti | 12 GB | 7680 | Gen 4 | Recommended baseline |
| RTX 4080 | 16 GB | 9728 | Gen 4 | Heavy simulation workloads |
| RTX 4090 | 24 GB | 16384 | Gen 4 | Professional development |
| RTX 6000 Ada | 48 GB | 18176 | Gen 4 | Multi-robot simulation |

**Key Considerations:**
- VRAM capacity limits scene complexity and batch sizes
- Tensor Cores accelerate deep learning operations
- RTX 4070 Ti or better recommended for Isaac Sim
- Multiple GPU support beneficial for distributed training

### A.1.3 CPU, RAM, and Storage Guidelines

**CPU Selection:**
- Minimum 6 cores for basic ROS 2 development
- 8-16 cores recommended for parallel simulation
- High single-thread performance benefits physics engines
- Support for AVX2 instructions required by many ML frameworks

**RAM Requirements:**
- 16 GB minimum for ROS 2 development
- 32 GB recommended for Gazebo or Isaac Sim
- 64 GB for large-scale simulations with multiple robots
- ECC memory optional but beneficial for long training runs

**Storage Configuration:**
- NVMe SSD required for OS and development tools
- Separate partition or drive for datasets (1-2 TB)
- SSD recommended for faster dataset loading
- Network-attached storage acceptable for archived data

**Recommended Partition Scheme:**
- `/` (root): 100 GB
- `/home`: 400+ GB (code, models, local datasets)
- `swap`: 16-32 GB (equal to RAM)
- `/data`: Remaining space (datasets, logs)

### A.1.4 Ubuntu 22.04 LTS Installation Overview

Ubuntu 22.04 LTS (Jammy Jellyfish) is the recommended Linux distribution for ROS 2 Humble development. Long-term support extends until April 2027.

**Pre-Installation Checklist:**
1. Backup all existing data
2. Download Ubuntu 22.04.3 LTS Desktop ISO from ubuntu.com
3. Verify ISO checksum (SHA256)
4. Create bootable USB drive (8 GB minimum)
5. Review BIOS/UEFI settings

**Installation Media Creation:**

Using Linux:
```bash
# Identify USB device (e.g., /dev/sdb)
lsblk

# Write ISO to USB drive (replace /dev/sdX with your device)
sudo dd bs=4M if=ubuntu-22.04.3-desktop-amd64.iso of=/dev/sdX status=progress oflag=sync
```

Using Windows:
- Use Rufus or balenaEtcher
- Select ISO file and target USB drive
- Choose GPT partition scheme for UEFI
- Click "Start" to create bootable drive

**Installation Steps:**
1. Boot from USB drive (F12, F2, or Del key during startup)
2. Select "Try or Install Ubuntu"
3. Choose installation language
4. Select keyboard layout
5. Choose "Normal installation" with updates and third-party software
6. Configure disk partitioning (see section A.1.5)
7. Set timezone and create user account
8. Complete installation and restart

**Post-Installation Configuration:**
```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install essential build tools
sudo apt install build-essential git curl wget vim -y

# Install NVIDIA drivers (if not already installed)
ubuntu-drivers devices
sudo ubuntu-drivers autoinstall

# Reboot to load new drivers
sudo reboot
```

### A.1.5 BIOS/UEFI Settings for Dual Boot

Configuring BIOS/UEFI correctly ensures smooth dual-boot operation with Windows and optimal hardware performance.

**Accessing BIOS/UEFI:**
- Restart computer and press Del, F2, F10, or F12 (varies by manufacturer)
- Common keys: Del (ASUS, MSI), F2 (Dell, Lenovo), F10 (HP)

**Recommended BIOS/UEFI Settings:**

**Boot Configuration:**
- Boot Mode: UEFI (not Legacy/CSM)
- Secure Boot: Disabled (or configure for Ubuntu)
- Fast Boot: Disabled
- Boot Order: USB first for installation, SSD first after installation

**Hardware Settings:**
- Virtualization Technology (VT-x/AMD-V): Enabled
- VT-d/IOMMU: Enabled (for GPU passthrough if needed)
- Above 4G Decoding: Enabled (for large GPU memory)
- Resizable BAR: Enabled (improves GPU performance)

**Power Management:**
- CPU C-States: Auto or Enabled
- Power Limit: Maximum (for workstations)
- Fan Control: Performance mode

**Storage Configuration:**
- SATA Mode: AHCI
- NVMe Configuration: Enabled
- Disable Windows Fast Startup to prevent filesystem issues

**Dual Boot Considerations:**
- Install Windows first, then Ubuntu
- Use separate drives if possible
- Ubuntu installer will detect Windows and configure GRUB
- Default GRUB timeout: 10 seconds (configurable in `/etc/default/grub`)

---

## A.2 NVIDIA Jetson Orin Setup

The NVIDIA Jetson Orin family provides embedded AI computing for robot deployment. These single-board computers enable onboard perception, control, and decision-making.

### A.2.1 Jetson Family Comparison

| Feature | Jetson Orin Nano | Jetson Orin NX | Jetson AGX Orin |
|---------|------------------|----------------|-----------------|
| GPU | 1024 CUDA cores | 1024-1792 cores | 1792-2048 cores |
| Tensor Cores | 32 | 56 | 64 |
| CPU | 6-core Arm Cortex-A78AE | 8-core Arm Cortex-A78AE | 12-core Arm Cortex-A78AE |
| AI Performance | 40 TOPS | 100 TOPS | 275 TOPS |
| Memory | 4-8 GB | 8-16 GB | 32-64 GB |
| Storage | microSD + NVMe | microSD + NVMe | NVMe |
| Power | 7-15W | 10-25W | 15-60W |
| Form Factor | SODIMM module | SODIMM module | Full module |
| Typical Use | Small mobile robots | Medium humanoids | Research platforms |
| Price Range | $199-$299 | $399-$599 | $999-$1,999 |

**Selection Guidelines:**
- Orin Nano: Quadruped scouts, lightweight arms, basic vision
- Orin NX: Mid-size humanoids, manipulation tasks, multi-sensor fusion
- AGX Orin: Full humanoid platforms, complex perception, real-time learning

### A.2.2 Flashing JetPack OS

JetPack SDK includes Ubuntu Linux, CUDA, cuDNN, TensorRT, and other essential libraries. The SDK Manager provides the recommended installation method.

**Prerequisites:**
- Host computer running Ubuntu 18.04 or 20.04
- USB cable (USB-C for Orin, Micro-USB for older modules)
- Jetson module and carrier board
- Internet connection for downloading packages

**Installation Steps:**

1. **Download NVIDIA SDK Manager:**
```bash
# Download from developer.nvidia.com/sdk-manager
# Or use command line:
wget https://developer.nvidia.com/downloads/sdkmanager_[version]_amd64.deb
sudo dpkg -i sdkmanager_[version]_amd64.deb
```

2. **Launch SDK Manager:**
```bash
sdkmanager
```

3. **Configure Installation:**
   - Login with NVIDIA Developer account
   - Select target hardware (e.g., Jetson AGX Orin 64GB)
   - Choose JetPack version (5.1.2 or later recommended)
   - Select components: Jetson OS, CUDA, cuDNN, TensorRT, VPI, Isaac ROS

4. **Put Jetson in Recovery Mode:**
   - Power off the Jetson
   - Connect USB cable to host computer
   - Press and hold RECOVERY button
   - Press and release POWER button
   - Hold RECOVERY for 2 more seconds
   - Release RECOVERY button

5. **Flash the System:**
   - SDK Manager will detect Jetson in recovery mode
   - Click "Flash" to begin installation
   - Enter sudo password when prompted
   - Wait for OS flash (10-20 minutes)

6. **Complete Setup on Jetson:**
   - After flashing, Jetson will reboot
   - Complete on-screen setup (language, user account)
   - SDK Manager will continue installing SDK components
   - Total time: 30-60 minutes depending on connection speed

**Alternative: SD Card Image Method (Orin Nano only):**
```bash
# Download image from developer.nvidia.com
# Flash to microSD card (64 GB minimum recommended)
sudo dd bs=4M if=jetson-orin-nano-sd-card-image.img of=/dev/sdX status=progress
```

### A.2.3 Initial Configuration Steps

After flashing JetPack, perform these configuration steps to prepare the Jetson for robotics development.

**System Update:**
```bash
# Update package lists and installed packages
sudo apt update && sudo apt upgrade -y

# Install essential development tools
sudo apt install build-essential git cmake python3-pip -y

# Verify CUDA installation
nvcc --version

# Test GPU functionality
/usr/local/cuda/samples/1_Utilities/deviceQuery/deviceQuery
```

**Performance Mode Configuration:**
```bash
# Check available power modes
sudo nvpmodel -q

# Set to maximum performance (varies by model)
# Orin Nano: mode 0 (15W)
# Orin NX: mode 0 (25W)
# AGX Orin: mode 0 (50W or 60W)
sudo nvpmodel -m 0

# Set CPU clocks to maximum
sudo jetson_clocks

# Make performance mode persistent
sudo systemctl enable jetson_clocks
```

**Storage Expansion (NVMe SSD):**
```bash
# Check NVMe detection
lsblk

# Format NVMe drive (if new)
sudo parted /dev/nvme0n1 mklabel gpt
sudo parted /dev/nvme0n1 mkpart primary ext4 0% 100%
sudo mkfs.ext4 /dev/nvme0n1p1

# Create mount point
sudo mkdir -p /mnt/nvme

# Mount automatically on boot
echo "/dev/nvme0n1p1 /mnt/nvme ext4 defaults 0 2" | sudo tee -a /etc/fstab
sudo mount -a
```

**Network Configuration:**
```bash
# Configure static IP (optional but recommended for robot platforms)
# Edit netplan configuration
sudo nano /etc/netplan/01-network-manager-all.yaml

# Example static IP configuration:
# network:
#   version: 2
#   renderer: NetworkManager
#   ethernets:
#     eth0:
#       dhcp4: no
#       addresses: [192.168.1.100/24]
#       gateway4: 192.168.1.1
#       nameservers:
#         addresses: [8.8.8.8, 8.8.4.4]

# Apply configuration
sudo netplan apply
```

### A.2.4 Power Supply Requirements

Proper power delivery is critical for stable Jetson operation under load.

**Power Specifications by Model:**

| Model | Input Voltage | Minimum Current | Recommended PSU | Connector Type |
|-------|---------------|-----------------|-----------------|----------------|
| Orin Nano Dev Kit | 5V or 9-20V | 3A (5V) / 2A (9-20V) | 15W (5V/3A) | USB-C or barrel |
| Orin NX Dev Kit | 9-20V | 2.5A | 30W (19V/2.5A) | Barrel jack |
| AGX Orin Dev Kit | 9-20V | 5A | 65W (19V/5A) | Barrel jack |

**Power Considerations:**
- Use power supply with 20% headroom above minimum
- Avoid USB power banks for continuous operation
- Provide dedicated power when using peripherals (cameras, motors)
- Monitor power consumption: `tegrastats` command
- Battery operation requires voltage regulation circuit

**Carrier Board Power Options:**
- Devkit carrier boards: Barrel jack (standard)
- Custom carrier boards: May use PoE, battery input, or other sources
- Mobile robots: Integrate DC-DC converter from main battery (12V-48V to 19V)

### A.2.5 Cooling Solutions

Adequate cooling maintains performance and prevents thermal throttling.

**Passive Cooling:**
- Stock heatsink suitable for \&lt;10Wlt;10W operation
- Add thermal pads for better contact with case
- Ensure airflow around heatsink fins
- Maximum ambient temperature: 35°C

**Active Cooling:**
- Required for sustained high-performance workloads
- Devkit includes PWM fan (5V)
- Fan automatically controlled by thermal management
- Manual fan control:
```bash
# Set fan to 100%
sudo sh -c 'echo 255 > /sys/devices/pwm-fan/target_pwm'

# Set fan to 50%
sudo sh -c 'echo 128 > /sys/devices/pwm-fan/target_pwm'
```

**Thermal Monitoring:**
```bash
# Real-time thermal status
tegrastats

# Monitor specific temperatures
cat /sys/devices/virtual/thermal/thermal_zone*/temp
```

**Cooling Recommendations by Deployment:**
- Desktop development: Stock fan, adequate
- Mobile robot (indoor): 30mm fan, PWM controlled
- Mobile robot (outdoor): Consider ambient temperature, may need larger heatsink
- Enclosed robot: Active ventilation required

---

## A.3 Sensor Integration

Sensors provide robots with environmental awareness. This section covers integration of common perception and proprioceptive sensors.

### A.3.1 Intel RealSense D435i Setup

The Intel RealSense D435i provides RGB, depth, and IMU data in a single package, making it ideal for robot perception.

**Hardware Specifications:**
- Depth technology: Stereo vision
- Depth range: 0.3m to 3m
- Depth resolution: Up to 1280x720
- RGB resolution: 1920x1080
- Frame rate: Up to 90 fps (depth), 30 fps (RGB)
- Field of view: 87° x 58° (depth), 69° x 42° (RGB)
- IMU: 6-axis (accel + gyro)
- Interface: USB 3.0 (Type-C)
- Power consumption: &lt;1.5W

**Driver Installation (Ubuntu 22.04):**
```bash
# Register Intel RealSense repository
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list

# Update and install
sudo apt update
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev -y

# Verify installation
realsense-viewer
```

**ROS 2 Integration:**
```bash
# Install RealSense ROS 2 wrapper
sudo apt install ros-humble-realsense2-camera -y

# Launch camera node
ros2 run realsense2_camera realsense2_camera_node

# Or use launch file with parameters
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=true \
  enable_color:=true \
  enable_infra:=true \
  enable_gyro:=true \
  enable_accel:=true
```

**Common Configuration Parameters:**
- `depth_module.profile`: Resolution and frame rate (e.g., 640x480x30)
- `align_depth.enable`: Align depth to color frame
- `spatial_filter.enable`: Reduce depth noise
- `temporal_filter.enable`: Reduce temporal noise
- `decimation_filter.enable`: Reduce resolution for performance

**Mounting Considerations:**
- Mount rigidly to reduce motion blur
- Avoid IR interference from other depth cameras
- USB 3.0 cable length: &lt;3m recommended
- Shield from direct sunlight for outdoor use

### A.3.2 IMU Calibration Procedures

Inertial Measurement Units (IMUs) require calibration to account for sensor biases and misalignments. Most robots include built-in IMUs (e.g., in RealSense D435i, on Jetson carrier boards).

**Types of Calibration:**

1. **Accelerometer Calibration (6-point tumble test):**
   - Place IMU in 6 orientations (+X, -X, +Y, -Y, +Z, -Z)
   - Record steady-state readings in each orientation
   - Expected: 1g on one axis, 0g on others
   - Calculate bias and scale factors

2. **Gyroscope Calibration (static bias):**
   - Place IMU on stable surface
   - Record gyroscope outputs for 30-60 seconds
   - Calculate mean bias for each axis
   - Subtract bias from future readings

3. **Magnetometer Calibration (if present):**
   - Rotate IMU through full 3D rotations
   - Record min/max values for each axis
   - Calculate hard-iron and soft-iron offsets

**Using RealSense IMU Calibration:**
```bash
# Install IMU calibration tool
python3 -m pip install pyrealsense2

# Run calibration (follow on-screen instructions)
rs-imu-calibration
```

**Manual Calibration Example (Python):**
```python
import pyrealsense2 as rs
import numpy as np
import time

# Configure pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.accel)
config.enable_stream(rs.stream.gyro)

# Start streaming
pipeline.start(config)

# Collect gyro bias data
gyro_samples = []
for _ in range(300):  # 10 seconds at 30 Hz
    frames = pipeline.wait_for_frames()
    gyro_frame = frames.first_or_default(rs.stream.gyro)
    gyro_data = gyro_frame.as_motion_frame().get_motion_data()
    gyro_samples.append([gyro_data.x, gyro_data.y, gyro_data.z])
    time.sleep(0.033)

# Calculate bias
gyro_bias = np.mean(gyro_samples, axis=0)
print(f"Gyro bias: {gyro_bias}")

pipeline.stop()
```

**Calibration Best Practices:**
- Perform calibration at operating temperature
- Recalibrate after mechanical shocks
- Store calibration parameters in configuration files
- Validate calibration periodically

### A.3.3 USB Microphone Configuration

Audio input enables speech recognition, sound localization, and acoustic event detection.

**Recommended USB Microphones:**
- ReSpeaker Mic Array v2.0 (4-mic circular array)
- Matrix Voice (8-mic circular array)
- PlayStation Eye (4-mic array, budget option)
- Generic USB microphones (mono/stereo)

**Device Detection:**
```bash
# List audio devices
arecord -l

# Example output:
# card 2: ArrayUAC10 [ReSpeaker 4 Mic Array (UAC1.0)], device 0: USB Audio [USB Audio]
#   Subdevices: 1/1
#   Subdevice #0: subdevice #0

# Test recording
arecord -D hw:2,0 -f S16_LE -r 16000 -c 4 test.wav
```

**ALSA Configuration:**
```bash
# Edit ALSA configuration for default device
nano ~/.asoundrc

# Add content:
pcm.!default {
    type hw
    card 2  # Your microphone card number
    device 0
}

ctl.!default {
    type hw
    card 2
}
```

**ROS 2 Audio Capture:**
```bash
# Install audio_common package
sudo apt install ros-humble-audio-common -y

# Launch audio capture node
ros2 run audio_capture audio_capture_node --ros-args \
  -p format:=wave \
  -p channels:=4 \
  -p sample_rate:=16000 \
  -p device:=hw:2,0
```

**Microphone Array Processing:**
- Use beamforming for directional audio capture
- Apply noise suppression for better speech recognition
- Consider acoustic echo cancellation if robot has speakers
- Libraries: ODAS (Open embeddeD Audition System), SpeexDSP

### A.3.4 Multiple Sensor Synchronization

Fusing data from multiple sensors requires temporal alignment.

**Synchronization Strategies:**

1. **Hardware Synchronization:**
   - External trigger signal to all sensors
   - Shared clock line (PTP, gPTP)
   - Most accurate but requires hardware support

2. **Software Synchronization:**
   - Timestamp messages using common clock
   - Use ROS 2 time synchronization
   - Apply message filters for approximate sync

3. **Post-Processing Alignment:**
   - Interpolate between samples
   - Account for different sensor rates
   - Use Kalman filtering for fusion

**ROS 2 Time Synchronization:**
```python
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image, Imu

class SensorSync(Node):
    def __init__(self):
        super().__init__('sensor_sync')

        # Create subscribers
        image_sub = Subscriber(self, Image, '/camera/image')
        imu_sub = Subscriber(self, Imu, '/imu/data')

        # Synchronize with 100ms tolerance
        ts = ApproximateTimeSynchronizer(
            [image_sub, imu_sub],
            queue_size=10,
            slop=0.1  # 100ms
        )
        ts.registerCallback(self.sync_callback)

    def sync_callback(self, image_msg, imu_msg):
        # Process synchronized messages
        time_diff = abs(
            image_msg.header.stamp.sec - imu_msg.header.stamp.sec +
            (image_msg.header.stamp.nanosec - imu_msg.header.stamp.nanosec) * 1e-9
        )
        self.get_logger().info(f'Time difference: {time_diff*1000:.2f} ms')
```

**Network Time Protocol (NTP) Setup:**
```bash
# Install chrony (modern NTP implementation)
sudo apt install chrony -y

# Configure as NTP client (robot)
sudo nano /etc/chrony/chrony.conf
# Add: server <workstation-ip> iburst

# Restart chrony
sudo systemctl restart chrony

# Verify synchronization
chronyc tracking
```

---

## A.4 Robot Platform Setup

This section covers setup procedures for common research and educational robot platforms.

### A.4.1 Unitree G1/Go2 Overview

Unitree Robotics produces commercial quadruped and humanoid robots for research and development.

**Unitree Go2 Quadruped:**
- DOF: 12 (3 per leg)
- Weight: 15 kg
- Payload: 5 kg
- Battery: 15,000 mAh (90 minutes runtime)
- Compute: Jetson Orin (education/research editions)
- Sensors: Foot force sensors, IMU, optional cameras
- Price: $2,700 - $3,500

**Unitree G1 Humanoid:**
- DOF: 23-43 (depending on configuration)
- Height: 1.3m
- Weight: 35 kg
- Battery: Built-in lithium battery
- Compute: Integrated control computer
- Sensors: IMU, joint encoders, optional vision
- Price: Contact manufacturer

**Software SDK:**
```bash
# Clone Unitree SDK
git clone https://github.com/unitreerobotics/unitree_ros2.git
cd unitree_ros2

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build
source install/setup.bash
```

**Network Setup:**
- Default robot IP: 192.168.123.15
- Connect via Ethernet or WiFi
- SDK communicates over UDP
- ROS 2 topics available through bridge

**Safety Features:**
- Emergency stop button (physical)
- Software emergency stop command
- Automatic shutdown on low battery
- Fall detection and recovery

### A.4.2 Robotis OP3 Configuration

ROBOTIS OP3 is an open-source humanoid robot platform popular in education and research.

**Hardware Specifications:**
- DOF: 20 (Dynamixel servos)
- Height: 510 mm
- Weight: 3.5 kg
- Compute: Intel NUC or compatible
- Sensors: Logitech C920 camera, IMU, FSR foot sensors
- Power: 11.1V LiPo battery

**Initial Setup:**
```bash
# OP3 uses ROS 1 (Noetic) by default
# Official packages available for Ubuntu 20.04

# Install OP3 packages
sudo apt install ros-noetic-op3-* -y

# Configure Dynamixel USB permissions
sudo usermod -aG dialout $USER
sudo reboot

# Test Dynamixel connection
sudo apt install ros-noetic-dynamixel-workbench -y
ros2 run dynamixel_workbench_controllers find_dynamixel /dev/ttyUSB0
```

**Servo Configuration:**
- Uses Dynamixel XM and XH series servos
- Communication: TTL (3-wire)
- Baud rate: 1,000,000 bps
- IDs: 1-20 (predefined in OP3 framework)

**Walking Gait Tuning:**
- Uses online walking engine with preview control
- Parameters: step length, period, hip pitch offset
- Tune in GUI or via configuration files
- Start with conservative parameters and gradually increase

### A.4.3 Network Setup and Communication

Proper network configuration enables remote control, monitoring, and multi-robot coordination.

**Direct Ethernet Connection:**
```bash
# On development workstation, configure static IP
sudo ip addr add 192.168.123.10/24 dev eth0

# On robot, verify IP
ip addr show

# Test connectivity
ping 192.168.123.15  # Robot IP
```

**WiFi Access Point (Robot as AP):**
```bash
# Install hostapd and dnsmasq
sudo apt install hostapd dnsmasq -y

# Configure hostapd
sudo nano /etc/hostapd/hostapd.conf
# interface=wlan0
# ssid=RobotAP
# channel=6
# hw_mode=g
# wpa=2
# wpa_passphrase=yourpassword
# wpa_key_mgmt=WPA-PSK

# Enable IP forwarding and NAT (if needed)
sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
```

**ROS 2 Domain ID Configuration:**
```bash
# Set ROS_DOMAIN_ID to isolate robots on same network
export ROS_DOMAIN_ID=42

# Add to ~/.bashrc for persistence
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
```

**Firewall Configuration:**
```bash
# Allow ROS 2 DDS discovery and communication
sudo ufw allow 7400:7500/udp
sudo ufw allow 7400:7500/tcp

# Allow SSH
sudo ufw allow 22/tcp

# Enable firewall
sudo ufw enable
```

### A.4.4 Safety Precautions

Working with physical robots requires adherence to safety protocols to prevent injury and equipment damage.

**General Safety Guidelines:**

1. **Workspace Preparation:**
   - Clear at least 2m radius around robot
   - Use padded floor mats for legged robots
   - Remove tripping hazards from area
   - Ensure adequate lighting

2. **Personal Protective Equipment:**
   - Safety glasses when working near end effectors
   - Closed-toe shoes in robot lab
   - Avoid loose clothing near moving parts

3. **Electrical Safety:**
   - Disconnect battery when performing maintenance
   - Use proper fuses and circuit protection
   - Check for frayed cables and connectors
   - Never override battery protection circuits

4. **Software Safety:**
   - Implement software emergency stop
   - Enforce joint limits in software
   - Use watchdog timers for critical systems
   - Test in simulation before deployment

5. **Operating Procedures:**
   - Always have emergency stop within reach
   - Start with low-power/slow-speed modes
   - Announce robot activation to nearby people
   - Never leave powered robot unattended

**Emergency Stop Implementation:**
```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class EmergencyStop(Node):
    def __init__(self):
        super().__init__('emergency_stop')
        self.estop_service = self.create_service(
            Trigger, 'emergency_stop', self.estop_callback
        )
        self.estop_active = False

    def estop_callback(self, request, response):
        self.estop_active = True
        # Stop all motors immediately
        self.stop_all_motors()
        response.success = True
        response.message = 'Emergency stop activated'
        self.get_logger().error('EMERGENCY STOP ACTIVATED')
        return response

    def stop_all_motors(self):
        # Send zero velocity commands to all actuators
        # Engage brakes if available
        # Cut power to motors if possible
        pass
```

**Pre-Operation Checklist:**
- [ ] Workspace clear of obstacles and people
- [ ] Emergency stop accessible and tested
- [ ] Battery charged and properly connected
- [ ] All sensors functioning correctly
- [ ] Software safety limits configured
- [ ] Communication link established and stable
- [ ] Cooling systems operational (if applicable)
- [ ] Backup operator/spotter present (for large robots)

**Maintenance Safety:**
- Use lockout/tagout procedures for maintenance
- Discharge high-voltage capacitors before servicing
- Support robot mechanically during maintenance
- Document any modifications or repairs

---

## Summary

This appendix provided comprehensive hardware setup guidance for Physical AI development:

- **Workstation Configuration**: Detailed specifications for development computers, including GPU requirements, BIOS settings, and Ubuntu installation
- **Jetson Orin Setup**: Comparison of Jetson models, JetPack flashing procedures, and performance configuration
- **Sensor Integration**: Setup procedures for RealSense cameras, IMU calibration, microphones, and multi-sensor synchronization
- **Robot Platforms**: Configuration guides for Unitree and ROBOTIS platforms, network setup, and critical safety precautions

Proper hardware configuration establishes the foundation for successful robotics development. Refer to manufacturer documentation for platform-specific details and updates.
