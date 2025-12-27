# Appendix C: Reference Materials

This appendix provides quick-reference materials for common APIs, formats, and troubleshooting procedures in Physical AI development.

## C.1 ROS 2 API Reference

### C.1.1 rclpy Common Classes and Methods

**Node Class**

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
```

**Common Node Methods:**

| Method | Description | Example |
|--------|-------------|---------|
| `create_publisher(msg_type, topic, qos)` | Create a publisher | `self.pub = self.create_publisher(String, '/topic', 10)` |
| `create_subscription(msg_type, topic, callback, qos)` | Create a subscriber | `self.sub = self.create_subscription(String, '/topic', self.callback, 10)` |
| `create_timer(period, callback)` | Create a timer | `self.timer = self.create_timer(0.5, self.timer_callback)` |
| `create_service(srv_type, name, callback)` | Create a service | `self.srv = self.create_service(AddTwoInts, 'add', self.callback)` |
| `create_client(srv_type, name)` | Create a service client | `self.cli = self.create_client(AddTwoInts, 'add')` |
| `get_logger()` | Get logger instance | `self.get_logger().info('message')` |
| `get_clock()` | Get clock instance | `now = self.get_clock().now()` |
| `declare_parameter(name, value)` | Declare a parameter | `self.declare_parameter('my_param', 42)` |
| `get_parameter(name)` | Get parameter value | `val = self.get_parameter('my_param').value` |
| `destroy_node()` | Clean up node | `self.destroy_node()` |

**Publisher Methods:**

| Method | Description | Example |
|--------|-------------|---------|
| `publish(msg)` | Publish a message | `self.publisher.publish(msg)` |
| `get_subscription_count()` | Get number of subscribers | `count = self.publisher.get_subscription_count()` |

**Subscription Callback Signature:**

```python
def callback(self, msg):
    # Process message
    self.get_logger().info(f'Received: {msg.data}')
```

**Service Callback Signature:**

```python
def service_callback(self, request, response):
    # Process request and populate response
    response.sum = request.a + request.b
    return response
```

**Timer Callback Signature:**

```python
def timer_callback(self):
    # Periodic execution
    self.get_logger().info('Timer triggered')
```

**Quality of Service (QoS) Profiles:**

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Predefined profiles
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_services_default
from rclpy.qos import qos_profile_parameters

# Custom QoS
custom_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

**Common QoS Settings:**

| Profile | Reliability | Durability | History | Use Case |
|---------|-------------|------------|---------|----------|
| `sensor_data` | Best Effort | Volatile | Keep Last (5) | Sensor streams |
| `system_default` | Reliable | Volatile | Keep Last (10) | General topics |
| `services_default` | Reliable | Volatile | Keep Last (10) | Services |
| `parameters` | Reliable | Volatile | Keep Last (1000) | Parameter events |

**Logging Levels:**

```python
self.get_logger().debug('Debug message')
self.get_logger().info('Info message')
self.get_logger().warn('Warning message')
self.get_logger().error('Error message')
self.get_logger().fatal('Fatal message')
```

**Time Handling:**

```python
from rclpy.time import Time, Duration

# Get current time
now = self.get_clock().now()

# Create timestamp
stamp = Time(seconds=123, nanoseconds=456)

# Duration
duration = Duration(seconds=1.5)

# Time arithmetic
future_time = now + duration
time_diff = future_time - now
```

**Parameter Declaration and Access:**

```python
# Declare with default
self.declare_parameter('robot_name', 'robot1')
self.declare_parameter('max_speed', 1.0)
self.declare_parameter('enable_debug', False)

# Get parameter values
name = self.get_parameter('robot_name').get_parameter_value().string_value
speed = self.get_parameter('max_speed').get_parameter_value().double_value
debug = self.get_parameter('enable_debug').get_parameter_value().bool_value

# Or use typed helper
name = self.get_parameter('robot_name').value
```

### C.1.2 Common Message Types

**std_msgs:**

| Message Type | Fields | Use Case |
|--------------|--------|----------|
| `Bool` | `bool data` | Binary signals |
| `Int32` | `int32 data` | Integer values |
| `Float64` | `float64 data` | Floating-point values |
| `String` | `string data` | Text messages |
| `Header` | `stamp, frame_id` | Message headers |
| `ColorRGBA` | `r, g, b, a` | Color values |

**geometry_msgs:**

| Message Type | Fields | Use Case |
|--------------|--------|----------|
| `Point` | `x, y, z` | 3D positions |
| `Vector3` | `x, y, z` | 3D vectors |
| `Quaternion` | `x, y, z, w` | Rotations |
| `Pose` | `position, orientation` | 6D pose |
| `PoseStamped` | `header, pose` | Timestamped pose |
| `Transform` | `translation, rotation` | Transforms |
| `TransformStamped` | `header, child_frame_id, transform` | TF transforms |
| `Twist` | `linear, angular` | Velocity commands |
| `TwistStamped` | `header, twist` | Timestamped velocities |
| `Wrench` | `force, torque` | Forces |

**sensor_msgs:**

| Message Type | Key Fields | Use Case |
|--------------|------------|----------|
| `Image` | `header, height, width, encoding, data` | Camera images |
| `CameraInfo` | `header, height, width, K, D, R, P` | Camera calibration |
| `LaserScan` | `header, angle_min, angle_max, ranges` | 2D lidar |
| `PointCloud2` | `header, fields, data` | 3D point clouds |
| `Imu` | `header, orientation, angular_velocity, linear_acceleration` | IMU data |
| `JointState` | `header, name, position, velocity, effort` | Joint states |
| `NavSatFix` | `header, latitude, longitude, altitude` | GPS data |
| `Range` | `header, radiation_type, field_of_view, min_range, max_range, range` | Ultrasonic/IR sensors |

**nav_msgs:**

| Message Type | Key Fields | Use Case |
|--------------|------------|----------|
| `Odometry` | `header, child_frame_id, pose, twist` | Robot odometry |
| `Path` | `header, poses[]` | Planned paths |
| `OccupancyGrid` | `header, info, data` | 2D maps |
| `MapMetaData` | `resolution, width, height, origin` | Map information |

**tf2_msgs:**

| Message Type | Key Fields | Use Case |
|--------------|------------|----------|
| `TFMessage` | `transforms[]` | Transform broadcasts |

**action_msgs:**

| Message Type | Key Fields | Use Case |
|--------------|------------|----------|
| `GoalStatus` | `goal_id, status` | Action goal status |

### C.1.3 Service and Action Interfaces

**Common Service Types:**

**std_srvs:**

```python
# Empty service (trigger with no data)
from std_srvs.srv import Empty

# SetBool service
from std_srvs.srv import SetBool
# Request: bool data
# Response: bool success, string message

# Trigger service
from std_srvs.srv import Trigger
# Request: (empty)
# Response: bool success, string message
```

**example_interfaces:**

```python
# AddTwoInts
from example_interfaces.srv import AddTwoInts
# Request: int64 a, int64 b
# Response: int64 sum
```

**Custom Service Definition:**

```
# my_package/srv/GetTransform.srv
string source_frame
string target_frame
---
geometry_msgs/TransformStamped transform
bool success
string message
```

**Action Types:**

**Common Action Structure:**

```
# Goal
target_pose geometry_msgs/PoseStamped
---
# Result
final_pose geometry_msgs/PoseStamped
bool success
---
# Feedback
current_pose geometry_msgs/PoseStamped
distance_remaining float64
```

**Using Actions (Client):**

```python
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class NavClient(Node):
    def __init__(self):
        super().__init__('nav_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance: {feedback.distance_remaining}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}')
```

### C.1.4 Launch File Syntax Reference

**Python Launch File Structure:**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='Robot namespace'
    )

    arg_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation'
    )

    # Get launch configuration values
    namespace = LaunchConfiguration('namespace')
    use_sim = LaunchConfiguration('use_sim')

    # Define nodes
    node_controller = Node(
        package='my_package',
        executable='controller',
        name='controller',
        namespace=namespace,
        parameters=[
            {'param1': 'value1'},
            PathJoinSubstitution([
                FindPackageShare('my_package'),
                'config',
                'params.yaml'
            ])
        ],
        remappings=[
            ('/cmd_vel', '/robot/cmd_vel')
        ],
        condition=IfCondition(use_sim)
    )

    # Include another launch file
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('other_package'),
                'launch',
                'other.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': namespace,
        }.items()
    )

    return LaunchDescription([
        arg_namespace,
        arg_use_sim,
        node_controller,
        included_launch
    ])
```

**Common Launch Actions:**

| Action | Purpose | Example |
|--------|---------|---------|
| `Node` | Launch ROS 2 node | See above |
| `ExecuteProcess` | Run external command | `ExecuteProcess(cmd=['gazebo', 'world.sdf'])` |
| `IncludeLaunchDescription` | Include another launch file | See above |
| `DeclareLaunchArgument` | Define launch argument | See above |
| `SetParameter` | Set global parameter | `SetParameter(name='use_sim_time', value=True)` |
| `GroupAction` | Group actions with common config | Namespace, condition |
| `TimerAction` | Delay action execution | `TimerAction(period=5.0, actions=[node])` |

**Launch File Best Practices:**

```python
# Use descriptive names
DeclareLaunchArgument('robot_model', default_value='g1', description='Robot model to use')

# Path construction
config_file = PathJoinSubstitution([
    FindPackageShare('my_package'),
    'config',
    'robot.yaml'
])

# Conditional execution
Node(
    ...,
    condition=IfCondition(LaunchConfiguration('enable_camera'))
)

# Output configuration
Node(
    ...,
    output='screen',  # or 'log', 'both'
    emulate_tty=True  # For colored output
)
```

---

## C.2 URDF/SDF Format Specifications

### C.2.1 URDF Element Reference

**Robot Element (Root):**

```xml
<robot name="robot_name">
  <!-- Links and joints -->
</robot>
```

**Link Element:**

```xml
<link name="link_name">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
      <!-- or <cylinder radius="0.5" length="1"/> -->
      <!-- or <sphere radius="0.5"/> -->
      <!-- or <mesh filename="package://pkg/meshes/model.dae" scale="1 1 1"/> -->
    </geometry>
    <material name="material_name">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>

  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </collision>

  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="1.0"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0"
             iyy="1.0" iyz="0.0"
             izz="1.0"/>
  </inertial>
</link>
```

**Joint Element:**

```xml
<joint name="joint_name" type="revolute">
  <!-- type: fixed, revolute, continuous, prismatic, floating, planar -->

  <parent link="parent_link"/>
  <child link="child_link"/>

  <origin xyz="0 0 0" rpy="0 0 0"/>

  <axis xyz="0 0 1"/>

  <!-- For revolute and prismatic joints -->
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>

  <!-- Dynamics (optional) -->
  <dynamics damping="0.7" friction="0.0"/>
</joint>
```

**Joint Types:**

| Type | DOF | Parameters | Use Case |
|------|-----|------------|----------|
| `fixed` | 0 | None | Rigid attachment |
| `revolute` | 1 | `axis`, `limit` | Rotating joint with limits |
| `continuous` | 1 | `axis` | Rotating joint without limits |
| `prismatic` | 1 | `axis`, `limit` | Sliding joint |
| `floating` | 6 | None | Free-floating (rarely used) |
| `planar` | 2 | Normal direction | Motion in plane |

**Calculating Inertia Tensors:**

**Box (dimensions: x, y, z):**
```
ixx = (1/12) * m * (y² + z²)
iyy = (1/12) * m * (x² + z²)
izz = (1/12) * m * (x² + y²)
ixy = ixz = iyz = 0
```

**Cylinder (radius: r, height: h, axis along z):**
```
ixx = iyy = (1/12) * m * (3*r² + h²)
izz = (1/2) * m * r²
ixy = ixz = iyz = 0
```

**Sphere (radius: r):**
```
ixx = iyy = izz = (2/5) * m * r²
ixy = ixz = iyz = 0
```

**Gazebo-Specific Extensions:**

```xml
<gazebo reference="link_name">
  <material>Gazebo/Red</material>
  <mu1>0.2</mu1>  <!-- Friction coefficient 1 -->
  <mu2>0.2</mu2>  <!-- Friction coefficient 2 -->
  <kp>1000000.0</kp>  <!-- Contact stiffness -->
  <kd>100.0</kd>  <!-- Contact damping -->
  <selfCollide>true</selfCollide>
</gazebo>

<gazebo>
  <plugin name="plugin_name" filename="libplugin.so">
    <parameter>value</parameter>
  </plugin>
</gazebo>
```

### C.2.2 SDF Element Reference

**SDF Root (Simulation Description Format):**

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="world_name">
    <!-- World contents -->
  </world>

  <!-- or -->

  <model name="model_name">
    <!-- Model contents -->
  </model>
</sdf>
```

**Model Element:**

```xml
<model name="robot">
  <pose>0 0 0.5 0 0 0</pose>  <!-- x y z roll pitch yaw -->
  <static>false</static>

  <link name="base_link">
    <pose relative_to="__model__">0 0 0 0 0 0</pose>

    <inertial>
      <mass>1.0</mass>
      <inertia>
        <ixx>0.1</ixx>
        <iyy>0.1</iyy>
        <izz>0.1</izz>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyz>0</iyz>
      </inertia>
    </inertial>

    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.5</mu>
            <mu2>0.5</mu2>
          </ode>
        </friction>
        <contact>
          <ode>
            <kp>1e6</kp>
            <kd>100</kd>
          </ode>
        </contact>
      </surface>
    </collision>

    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <material>
        <ambient>1 0 0 1</ambient>
        <diffuse>1 0 0 1</diffuse>
        <specular>0.1 0.1 0.1 1</specular>
      </material>
    </visual>

    <sensor name="camera" type="camera">
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>1920</width>
          <height>1080</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <update_rate>30</update_rate>
    </sensor>
  </link>

  <joint name="joint" type="revolute">
    <parent>base_link</parent>
    <child>child_link</child>
    <axis>
      <xyz>0 0 1</xyz>
      <limit>
        <lower>-1.57</lower>
        <upper>1.57</upper>
        <effort>100</effort>
        <velocity>1</velocity>
      </limit>
      <dynamics>
        <damping>0.1</damping>
        <friction>0.0</friction>
      </dynamics>
    </axis>
  </joint>

  <plugin name="plugin_name" filename="libplugin.so">
    <param>value</param>
  </plugin>
</model>
```

**World Element:**

```xml
<world name="default">
  <physics name="default_physics" type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
  </physics>

  <scene>
    <ambient>0.4 0.4 0.4 1</ambient>
    <background>0.7 0.7 0.7 1</background>
    <shadows>true</shadows>
  </scene>

  <light name="sun" type="directional">
    <pose>0 0 10 0 0 0</pose>
    <diffuse>1 1 1 1</diffuse>
    <specular>0.5 0.5 0.5 1</specular>
    <direction>-0.5 0.1 -0.9</direction>
  </light>

  <include>
    <uri>model://ground_plane</uri>
  </include>

  <model name="my_robot">
    <!-- Model definition or include -->
  </model>
</world>
```

### C.2.3 Common Attributes and Values

**Geometry Types:**

| Type | Parameters | Example |
|------|------------|---------|
| Box | `size="x y z"` | `<box><size>1 1 1</size></box>` |
| Cylinder | `radius`, `length` | `<cylinder><radius>0.5</radius><length>1</length></cylinder>` |
| Sphere | `radius` | `<sphere><radius>0.5</radius></sphere>` |
| Mesh | `filename`, `scale` | `<mesh><uri>model.dae</uri><scale>1 1 1</scale></mesh>` |
| Plane | `normal`, `size` | `<plane><normal>0 0 1</normal><size>10 10</size></plane>` |

**Material Colors (Gazebo):**

| Name | Color |
|------|-------|
| `Gazebo/White` | White |
| `Gazebo/Black` | Black |
| `Gazebo/Red` | Red |
| `Gazebo/Green` | Green |
| `Gazebo/Blue` | Blue |
| `Gazebo/Yellow` | Yellow |
| `Gazebo/Purple` | Purple |
| `Gazebo/Orange` | Orange |
| `Gazebo/Grey` | Grey |

**Sensor Types (SDF):**

| Type | Use Case | Key Parameters |
|------|----------|----------------|
| `camera` | RGB camera | `horizontal_fov`, `image` |
| `depth_camera` | Depth camera | Same as camera + depth |
| `gpu_lidar` | 3D lidar | `scan`, `range` |
| `imu` | IMU sensor | `angular_velocity`, `linear_acceleration` |
| `contact` | Contact sensor | `collision` |
| `force_torque` | Force/torque | `frame`, `measure_direction` |
| `gps` | GPS sensor | `position_sensing` |
| `magnetometer` | Magnetometer | `noise` |

---

## C.3 Isaac ROS Package Reference

### C.3.1 Isaac ROS GEMs Overview

**Isaac ROS GEMs (GPU-Accelerated Modules):**

| Package | Functionality | Acceleration |
|---------|---------------|--------------|
| `isaac_ros_dnn_inference` | Deep learning inference | TensorRT |
| `isaac_ros_image_pipeline` | Image preprocessing | CUDA |
| `isaac_ros_stereo_image_proc` | Stereo processing | CUDA |
| `isaac_ros_apriltag` | AprilTag detection | CUDA |
| `isaac_ros_visual_slam` | Visual SLAM | CUDA + TensorRT |
| `isaac_ros_nvblox` | 3D reconstruction | CUDA |
| `isaac_ros_depth_segmentation` | Depth segmentation | CUDA |
| `isaac_ros_object_detection` | Object detection | TensorRT |
| `isaac_ros_pose_estimation` | Pose estimation | TensorRT |

### C.3.2 Common Parameters and Topics

**isaac_ros_dnn_inference:**

Parameters:
```yaml
model_file_path: "/path/to/model.onnx"
engine_file_path: "/path/to/engine.plan"
input_tensor_names: ["input"]
output_tensor_names: ["output"]
input_binding_names: ["input"]
output_binding_names: ["output"]
```

Topics:
- Input: `/tensor_pub` (isaac_ros_tensor_list_interfaces/TensorList)
- Output: `/tensor_sub` (isaac_ros_tensor_list_interfaces/TensorList)

**isaac_ros_apriltag:**

Parameters:
```yaml
size: 0.162  # Tag size in meters
max_tags: 64
family: "tag36h11"  # Tag family
```

Topics:
- Input: `/image` (sensor_msgs/Image)
- Input: `/camera_info` (sensor_msgs/CameraInfo)
- Output: `/detections` (isaac_ros_apriltag_interfaces/AprilTagDetectionArray)

**isaac_ros_visual_slam:**

Parameters:
```yaml
enable_imu: false
enable_rectified_pose: true
enable_slam_visualization: true
enable_observations_view: false
enable_landmarks_view: false
map_frame: "map"
odom_frame: "odom"
base_frame: "base_link"
```

Topics:
- Input: `/stereo_camera/left/image` (sensor_msgs/Image)
- Input: `/stereo_camera/right/image` (sensor_msgs/Image)
- Input: `/stereo_camera/left/camera_info` (sensor_msgs/CameraInfo)
- Input: `/stereo_camera/right/camera_info` (sensor_msgs/CameraInfo)
- Output: `/visual_slam/tracking/odometry` (nav_msgs/Odometry)
- Output: `/visual_slam/tracking/vo_pose` (geometry_msgs/PoseStamped)

**isaac_ros_nvblox:**

Parameters:
```yaml
voxel_size: 0.05  # Meters
esdf_update_rate: 10.0  # Hz
mesh_update_rate: 10.0  # Hz
```

Topics:
- Input: `/depth/image` (sensor_msgs/Image)
- Input: `/depth/camera_info` (sensor_msgs/CameraInfo)
- Input: `/pose` (geometry_msgs/PoseStamped)
- Output: `/mesh` (nvblox_msgs/Mesh)
- Output: `/map_slice` (nvblox_msgs/DistanceMapSlice)

### C.3.3 Performance Tuning Guidelines

**General Optimization:**

1. **Use Correct Data Types:**
   - RGB8 for color images
   - MONO8 for grayscale
   - Avoid unnecessary conversions

2. **Adjust Queue Sizes:**
   ```python
   # Small queue for real-time processing
   qos_profile = QoSProfile(depth=1)

   # Larger queue for recording/playback
   qos_profile = QoSProfile(depth=10)
   ```

3. **Enable Zero-Copy:**
   ```python
   # Use intra-process communication
   rclpy.init()
   executor = rclpy.executors.MultiThreadedExecutor()
   ```

4. **Batch Processing:**
   - Process multiple images per inference when latency permits
   - Increases throughput at cost of latency

**TensorRT Optimization:**

| Setting | Impact | Trade-off |
|---------|--------|-----------|
| FP16 precision | 2x faster | Slight accuracy loss |
| INT8 precision | 4x faster | Requires calibration, accuracy loss |
| Batch size | Higher throughput | Higher latency |
| Workspace size | Better optimization | More GPU memory |

**Memory Management:**

```bash
# Monitor GPU memory
nvidia-smi -l 1

# Set TensorRT workspace size
export ISAAC_ROS_TENSORRT_WORKSPACE_SIZE=2147483648  # 2 GB
```

**CPU/GPU Affinity:**

```bash
# Pin processes to specific CPU cores
taskset -c 0-3 ros2 run package node

# Set GPU device
export CUDA_VISIBLE_DEVICES=0
```

---

## C.4 Common Troubleshooting Guide

### C.4.1 ROS 2 Common Errors

**Error: "ros2: command not found"**

Solution:
```bash
# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Add to bashrc for persistence
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

**Error: "Package 'X' not found"**

Solution:
```bash
# Update package database
sudo apt update

# Install package
sudo apt install ros-humble-package-name

# Or install from workspace
cd ~/ros2_ws
colcon build --packages-select package_name
source install/setup.bash
```

**Error: "DDS discovery timeout"**

Solution:
```bash
# Check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# Set consistent domain ID
export ROS_DOMAIN_ID=0

# Check firewall
sudo ufw allow 7400:7500/udp
sudo ufw allow 7400:7500/tcp

# Test with multicast
ros2 multicast receive
# In another terminal:
ros2 multicast send
```

**Error: "Failed to create node"**

Solution:
```python
# Check node name is valid (no spaces, special chars)
# Ensure no duplicate node names in same namespace

# Add unique suffix if needed
import random
node_name = f'my_node_{random.randint(1000, 9999)}'
```

**Error: "QoS mismatch"**

Solution:
```python
# Match QoS between publisher and subscriber
# Check with:
ros2 topic info /topic_name --verbose

# Use compatible QoS
from rclpy.qos import qos_profile_sensor_data
subscriber = node.create_subscription(
    Image,
    '/camera/image',
    callback,
    qos_profile_sensor_data
)
```

### C.4.2 Sensor Driver Issues

**RealSense Camera Not Detected:**

```bash
# Check USB connection
lsusb | grep Intel

# Verify driver installation
realsense-viewer

# Check permissions
sudo adduser $USER video
sudo reboot

# Reinstall udev rules
sudo cp /usr/local/etc/udev/rules.d/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**IMU Calibration Drift:**

```bash
# Recalibrate IMU
# Place on stable surface for 30 seconds
# Record bias values
# Update configuration file

# For RealSense:
rs-imu-calibration
```

**Camera Image Distortion:**

```bash
# Verify camera_info is published
ros2 topic echo /camera/camera_info

# Check calibration parameters (K, D matrices)
# Recalibrate if necessary using:
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.108 \
  image:=/camera/image_raw
```

### C.4.3 Simulation Crashes

**Gazebo Crashes on Launch:**

```bash
# Check graphics drivers
nvidia-smi

# Update drivers if needed
sudo ubuntu-drivers autoinstall

# Try software rendering (slow, for testing)
export LIBGL_ALWAYS_SOFTWARE=1
gazebo

# Check for model errors
gz model --check model.sdf
```

**Unity Simulation Freezes:**

Solutions:
1. Reduce scene complexity
2. Lower graphics quality settings
3. Check for infinite loops in scripts
4. Monitor with Process Explorer:
   ```bash
   top -p $(pgrep -d',' Unity)
   ```

**Physics Instability:**

```xml
<!-- Increase solver iterations in SDF -->
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <max_contacts>20</max_contacts>
  <solver>
    <type>quick</type>
    <iters>50</iters>  <!-- Increase from default 20 -->
    <sor>1.3</sor>
  </solver>
</physics>
```

### C.4.4 Network and Communication Problems

**ROS 2 Nodes Can't Communicate:**

```bash
# Check same ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# Check localhost restriction
echo $ROS_LOCALHOST_ONLY

# For multi-machine:
export ROS_LOCALHOST_ONLY=0

# Verify network connectivity
ping <other-machine-ip>

# Check DDS discovery
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

**High Latency in Message Passing:**

```bash
# Measure latency
ros2 topic hz /topic_name
ros2 topic bw /topic_name

# Solutions:
# 1. Reduce message size
# 2. Increase QoS depth
# 3. Use intra-process communication
# 4. Switch DDS implementation (FastDDS, CycloneDDS)
```

**Jetson Network Issues:**

```bash
# Check network interface
ifconfig

# Set static IP
sudo nano /etc/netplan/01-network-manager-all.yaml
sudo netplan apply

# Test bandwidth
iperf3 -s  # On one machine
iperf3 -c <server-ip>  # On other machine
```

---

## C.5 Glossary of Terms

### C.5.1 Robotics Terminology

| Term | Definition |
|------|------------|
| **Actuator** | Device that produces motion (motor, servo, pneumatic cylinder) |
| **DOF (Degrees of Freedom)** | Number of independent ways a robot can move |
| **End Effector** | Tool at the end of a robot arm (gripper, tool, sensor) |
| **Forward Kinematics** | Computing end-effector pose from joint angles |
| **Inverse Kinematics** | Computing joint angles to achieve desired end-effector pose |
| **Jacobian** | Matrix relating joint velocities to end-effector velocities |
| **Joint** | Connection between two links allowing relative motion |
| **Link** | Rigid body in a robot kinematic chain |
| **Odometry** | Estimating position from motion sensors (wheels, IMU) |
| **Payload** | Maximum load a robot can carry |
| **Singularity** | Configuration where robot loses one or more DOF |
| **Workspace** | Volume in which robot end-effector can reach |

### C.5.2 AI/ML Terms

| Term | Definition |
|------|------------|
| **Batch Size** | Number of samples processed before model update |
| **Epoch** | One complete pass through training dataset |
| **Feature Extraction** | Identifying relevant patterns in input data |
| **Fine-tuning** | Adapting pre-trained model to specific task |
| **Hyperparameter** | Configuration setting external to model (learning rate, etc.) |
| **Inference** | Using trained model to make predictions |
| **Learning Rate** | Step size for gradient descent optimization |
| **Loss Function** | Measure of prediction error during training |
| **Overfitting** | Model performs well on training but poorly on new data |
| **Pre-training** | Initial training on large dataset before task-specific training |
| **Transfer Learning** | Reusing knowledge from one task for another |
| **Validation Set** | Data used to tune hyperparameters, separate from test set |

### C.5.3 Hardware and Sensor Terms

| Term | Definition |
|------|------------|
| **Depth Camera** | Camera that provides distance to each pixel |
| **FOV (Field of View)** | Angular extent of observable area |
| **IMU** | Inertial Measurement Unit (accelerometer + gyroscope) |
| **Lidar** | Laser-based distance measurement sensor |
| **Odometry Sensor** | Sensor measuring motion (wheel encoders, visual odometry) |
| **Point Cloud** | Set of 3D points representing scanned surface |
| **RGB-D Camera** | Camera providing both color and depth |
| **Servo** | Motor with built-in position control |
| **Stepper Motor** | Motor moving in discrete steps |
| **Strain Gauge** | Sensor measuring force/torque via deformation |
| **Torque Sensor** | Sensor measuring rotational force |
| **VRAM** | Video RAM on GPU for graphics and compute |

**ROS 2-Specific Terms:**

| Term | Definition |
|------|------------|
| **Action** | Long-running task with goal, feedback, and result |
| **Bag File** | Recording of ROS 2 messages for playback |
| **Component** | Loadable node in shared library (composition) |
| **DDS** | Data Distribution Service (ROS 2 middleware) |
| **Launch File** | Script to start multiple nodes with configuration |
| **Message** | Data structure sent between nodes |
| **Node** | Executable process in ROS graph |
| **Package** | Unit of organization containing code and resources |
| **QoS** | Quality of Service settings for communication |
| **Service** | Synchronous request-response communication |
| **Topic** | Named bus for asynchronous message passing |
| **URDF** | Unified Robot Description Format (XML) |
| **Workspace** | Directory containing ROS 2 packages |

---

## Summary

This appendix provided comprehensive reference materials for Physical AI development:

- **ROS 2 API Reference**: Core rclpy classes, message types, services, actions, and launch file syntax
- **URDF/SDF Specifications**: Complete element reference for robot descriptions in both formats
- **Isaac ROS Reference**: Overview of GPU-accelerated packages with parameters and optimization guidelines
- **Troubleshooting Guide**: Solutions for common ROS 2, sensor, simulation, and network issues
- **Glossary**: Definitions of key terms in robotics, AI/ML, and hardware

Keep this appendix accessible during development for quick reference to APIs, formats, and solutions to common problems.
