# Chapter 4: Building with ROS 2

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand ROS 2 package structure and organization principles
- Explain how publishers and subscribers work conceptually
- Analyze when to use topics, services, and actions for different scenarios
- Describe the launch system architecture and its role in system orchestration
- Configure and manage parameters for flexible system configuration
- Understand the build system (colcon) and dependency management
- Design multi-node systems with appropriate communication patterns

## Introduction

Understanding ROS 2's architecture is essential, but building real systems requires mastering its development tools and patterns. How do you structure code into packages? How do publishers and subscribers actually communicate under the hood? When should you use a topic versus a service versus an action? How do you start ten nodes with the right parameters in the right order?

This chapter answers these questions by exploring the practical aspects of building with ROS 2. We examine package organization, communication pattern implementations, launch systems, parameter management, and build processes. By understanding these tools and patterns, you will be able to design and implement complex multi-node systems for humanoid robotics.

## ROS 2 Package Structure and Organization

### What is a Package?

A package is the fundamental unit of organization in ROS 2. It contains:

**Nodes:** Executable programs that perform computations.

**Launch Files:** Scripts that start multiple nodes with configuration.

**Configuration Files:** Parameters, URDFs, calibration data.

**Message Definitions:** Custom data structures for communication.

**Dependencies:** Declaration of required libraries and other packages.

**Build Instructions:** How to compile the package.

**Documentation:** READMEs, API documentation.

Packages encapsulate related functionality. A camera driver package contains the driver node, camera message definitions, calibration tools, and documentation—everything needed to use that camera.

### Package Organization Principles

Well-designed packages follow these principles:

**Single Responsibility:** Each package has one clear purpose. A "camera driver" package drives cameras, not processing images.

**Minimal Dependencies:** Packages depend only on what they need. Excessive dependencies create fragile systems.

**Reusability:** Packages should work in different contexts. A LiDAR driver shouldn't assume a specific robot configuration.

**Discoverability:** Clear naming and documentation help others find and use packages.

**Consistency:** Follow ROS 2 naming conventions and directory structure.

### Standard Package Structure

A typical ROS 2 package follows this directory layout:

```
my_robot_package/
├── CMakeLists.txt          (C++ build instructions)
├── package.xml             (package metadata and dependencies)
├── setup.py                (Python setup, if applicable)
├── setup.cfg               (Python configuration)
│
├── src/                    (C++ source files)
│   ├── my_node.cpp
│   └── utils.cpp
│
├── include/                (C++ header files)
│   └── my_robot_package/
│       └── my_node.hpp
│
├── my_robot_package/       (Python package directory)
│   ├── __init__.py
│   └── my_node.py
│
├── launch/                 (launch files)
│   ├── my_robot.launch.py
│   └── simulation.launch.xml
│
├── config/                 (configuration files)
│   ├── params.yaml
│   └── calibration.yaml
│
├── urdf/                   (robot descriptions)
│   └── my_robot.urdf
│
├── msg/                    (custom message definitions)
│   └── MyMessage.msg
│
├── srv/                    (custom service definitions)
│   └── MyService.srv
│
├── action/                 (custom action definitions)
│   └── MyAction.action
│
├── test/                   (unit tests)
│   └── test_my_node.py
│
└── README.md              (package documentation)
```

This structure separates concerns: source code in `src/`, configuration in `config/`, launch files in `launch/`. Tools like build systems and package managers understand this structure.

### Package Metadata (package.xml)

The `package.xml` file describes the package:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_robot_package</name>
  <version>1.0.0</version>
  <description>Description of package functionality</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <!-- Build tool -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Dependencies needed to build -->
  <build_depend>rclcpp</build_depend>
  <build_depend>sensor_msgs</build_depend>

  <!-- Dependencies needed to run -->
  <exec_depend>rclcpp</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>

  <!-- Test dependencies -->
  <test_depend>ament_lint_auto</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

This declares that the package depends on `rclcpp` (ROS 2 C++ client library) and `sensor_msgs` (standard sensor message types). Build tools use this to ensure dependencies are available.

### Workspace Organization

Packages exist within workspaces—directories containing multiple packages:

```
ros2_workspace/
├── src/                    (source space - your packages)
│   ├── package_a/
│   ├── package_b/
│   └── package_c/
│
├── build/                  (build artifacts)
├── install/                (installed packages)
└── log/                    (build logs)
```

You edit code in `src/`, build into `build/` and `install/`, then source `install/setup.bash` to use the packages.

### Package Naming Conventions

ROS 2 follows naming conventions for clarity:

**Packages:** lowercase with underscores: `my_robot_bringup`, `camera_driver`

**Nodes:** lowercase with underscores: `robot_state_publisher`, `joint_controller`

**Topics:** lowercase with slashes for namespaces: `/camera/image_raw`, `/joint_states`

**Messages/Services/Actions:** CamelCase: `SetBool`, `NavigateToPose`, `JointState`

**Parameters:** lowercase with underscores or dots: `max_velocity`, `camera.frame_rate`

Consistent naming prevents confusion and improves readability.

## Publishers and Subscribers: How They Work

### The Publish-Subscribe Pattern

Publishers and subscribers implement asynchronous, many-to-many communication:

**Decoupling:** Publishers don't know about subscribers. Subscribers don't know about publishers. This separation allows changing either side independently.

**Flexibility:** Adding a new subscriber doesn't require modifying publishers. Multiple subscribers receive the same data.

**Scalability:** Can handle hundreds of concurrent data streams.

### Publisher Lifecycle

When you create a publisher, several things happen:

**Topic Registration:** The publisher announces it will publish on a specific topic with a specific message type.

**Discovery:** Through DDS, the publisher advertises itself to all nodes in the system.

**Buffer Allocation:** Memory is allocated for outgoing messages based on QoS history settings.

**Connection Establishment:** When subscribers appear, DDS establishes communication paths.

**Publishing:** When you call `publish()`, the message is serialized and sent via DDS to all subscribers.

**Conceptual Flow:**
```
[Node creates Publisher]
       ↓
[Register topic + type with DDS]
       ↓
[DDS advertises to network]
       ↓
[Subscriber discovers publisher] ← (DDS discovery)
       ↓
[DDS establishes connection]
       ↓
[publish() called]
       ↓
[Message serialized]
       ↓
[DDS distributes to all subscribers]
```

### Subscriber Lifecycle

Subscribers mirror publishers:

**Topic Registration:** The subscriber announces interest in a specific topic and message type.

**Discovery:** DDS discovers compatible publishers.

**Buffer Allocation:** Memory for incoming messages based on QoS.

**Connection Establishment:** DDS connects to publishers.

**Callback Registration:** The subscriber registers a callback function to invoke when messages arrive.

**Message Reception:** When messages arrive, DDS deserializes them and invokes the callback with the message data.

**Conceptual Flow:**
```
[Node creates Subscriber]
       ↓
[Register topic + type + callback]
       ↓
[DDS advertises subscription]
       ↓
[DDS discovers publishers]
       ↓
[DDS establishes connections]
       ↓
[Message arrives from publisher]
       ↓
[DDS deserializes message]
       ↓
[Callback invoked with message]
```

### Message Serialization and Deserialization

ROS 2 messages are language-agnostic data structures. To send over network:

**Serialization (Publisher):** Convert message from in-memory structure to byte stream. DDS handles this using Common Data Representation (CDR) format.

**Transmission:** Byte stream sent via network (UDP, shared memory, etc.).

**Deserialization (Subscriber):** Convert byte stream back to in-memory structure.

This serialization is transparent—you work with native data structures in your programming language.

### Zero-Copy Communication

For efficiency, ROS 2 supports zero-copy transport when publisher and subscriber are on the same machine:

**Shared Memory:** Instead of copying message data, both publisher and subscriber access the same memory region.

**Pointer Passing:** Only pointers are passed, not data.

**Performance:** Eliminates copy overhead for large messages (images, point clouds).

**Limitations:** Works only on same machine, requires compatible QoS settings.

This is particularly important for humanoid robots processing high-resolution images and point clouds.

### Topic Namespaces and Remapping

Topics can be namespaced to organize communication:

**Global Topics:** Begin with `/`: `/camera/image`

**Relative Topics:** Don't begin with `/`: `image` (resolved relative to node namespace)

**Private Topics:** Begin with `~`: `~/debug` (private to node)

**Remapping:** Topics can be remapped at runtime:
```
ros2 run my_package my_node --ros-args -r chatter:=/my_topic
```

This remaps topic `chatter` to `/my_topic`, allowing reusing nodes in different contexts without code changes.

## Services vs Topics vs Actions: When to Use Each

### Decision Framework

Choosing the right communication pattern is critical for system design:

**Topics:** Continuous data streams, latest value matters, one-way communication.

**Services:** Request-response, confirmation needed, infrequent transactions.

**Actions:** Long-running tasks, progress updates needed, cancellation required.

### Topic Use Cases

Use topics when:

**Sensor Data Streaming:** Camera images, LiDAR scans, IMU readings. Data flows continuously, subscribers want latest values.

**State Publishing:** Joint positions, robot pose, odometry. Multiple nodes need current state.

**Command Streams:** Velocity commands to motors. Commands stream continuously at control rate.

**Event Notifications:** Collision detected, button pressed. Events broadcast to interested parties.

**Examples:**
- `/camera/image_raw`: Camera driver publishes, detector/visualizer subscribe
- `/joint_states`: Joint controllers publish, state estimator/visualizer subscribe
- `/cmd_vel`: Planner publishes, motor controller subscribes
- `/diagnostics`: All nodes publish health, monitor subscribes

### Service Use Cases

Use services when:

**Configuration Changes:** Setting parameters, triggering calibration. Need confirmation of change.

**Computation Requests:** Path planning, inverse kinematics. Send input, get computed result.

**State Queries:** Get map, retrieve configuration. Query current state.

**Control Commands:** Start/stop processes, reset state. Need acknowledgment.

**Examples:**
- `/set_parameters`: Change node configuration
- `/compute_ik`: Request inverse kinematics solution
- `/get_planning_scene`: Retrieve current environment model
- `/trigger_calibration`: Start calibration routine

### Action Use Cases

Use actions when:

**Navigation:** Moving to goal takes seconds. Want progress updates, ability to cancel.

**Manipulation:** Grasping object takes time. Need feedback on approach progress.

**Long Computations:** Planning complex path takes seconds. Want to cancel if taking too long.

**Sequences:** Executing multi-step behaviors. Want per-step feedback.

**Examples:**
- `/navigate_to_pose`: Navigate to target with progress updates
- `/follow_joint_trajectory`: Execute trajectory with feedback
- `/grasp_object`: Approach, grasp, lift with feedback
- `/dock`: Autonomous docking with progress updates

### Comparison Table

| Aspect | Topic | Service | Action |
|--------|-------|---------|--------|
| Pattern | Pub-Sub | Request-Response | Goal-Feedback-Result |
| Timing | Asynchronous | Synchronous | Asynchronous |
| Duration | Continuous | Instant | Long-running |
| Feedback | No | No | Yes |
| Cancellation | N/A | N/A | Yes |
| Confirmation | No | Yes | Yes |
| Multiple Consumers | Yes | No | No |
| Use Case | Streaming data | Quick transactions | Tasks with progress |

### Hybrid Patterns

Real systems often combine patterns:

**Command + Status:** Publish commands on topic, subscribe to status topic for feedback. Simpler than actions for continuous control.

**Service + Topic:** Call service to start process, subscribe to topic for results. Avoids blocking on long computations.

**Action + Topic:** Use action for high-level goal, publish intermediate results on topics for other nodes.

**Example Humanoid Navigation:**
- Action `/navigate_to_pose`: High-level navigation goal
- Topic `/cmd_vel`: Low-level velocity commands to motors
- Topic `/odometry`: Position feedback
- Service `/get_map`: Retrieve map for planning

## Launch System Architecture

### Why Launch Systems?

Starting a humanoid robot manually is impractical:

```
Terminal 1: ros2 run camera_driver camera_node
Terminal 2: ros2 run depth_processing depth_node
Terminal 3: ros2 run object_detection detector_node --ros-args -p threshold:=0.8
Terminal 4: ros2 run localization slam_node
Terminal 5: ros2 run planning planner_node
Terminal 6: ros2 run control controller_node
... (20+ more nodes)
```

Launch systems automate this, starting all nodes with correct parameters, namespaces, and remappings in a single command.

### Launch File Anatomy

Launch files describe how to start a system. They can:

- Start nodes with arguments
- Set parameters from YAML files
- Remap topics/services
- Set namespaces
- Include other launch files
- Execute actions conditionally
- Set environment variables
- Start processes in order

### Launch File Types

ROS 2 supports multiple launch file formats:

**Python Launch Files (.launch.py):** Most flexible. Full Python scripting available.

**XML Launch Files (.launch.xml):** Declarative, easier for simple cases.

**YAML Launch Files (.launch.yaml):** Simplest, limited functionality.

Python launch files are most common for complex systems.

### Python Launch File Structure

A basic Python launch file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_driver',
            executable='camera_node',
            name='camera',
            parameters=[{
                'frame_rate': 30,
                'resolution': '1920x1080'
            }]
        ),
        Node(
            package='object_detection',
            executable='detector_node',
            name='detector',
            parameters=[{'confidence_threshold': 0.8}],
            remappings=[
                ('/image', '/camera/image_raw')
            ]
        )
    ])
```

This starts a camera node and detector node with specified parameters and topic remapping.

### Launch File Composition

Complex systems use composition—including other launch files:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('sensors.launch.py')
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('perception.launch.py')
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('planning.launch.py')
        )
    ])
```

This modular approach separates concerns: `sensors.launch.py` starts sensors, `perception.launch.py` starts perception nodes, etc. You can test each layer independently.

### Conditional Launching

Launch files support conditional logic:

```python
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim'),

        Node(
            package='gazebo_ros',
            executable='spawn_entity',
            condition=IfCondition(use_sim),
            # Only launch in simulation
        ),
        Node(
            package='hardware_interface',
            executable='robot_hw_node',
            condition=UnlessCondition(use_sim),
            # Only launch on real hardware
        )
    ])
```

This allows one launch file to work in simulation and on real hardware.

### Launch File Best Practices

**Parameterize:** Use launch arguments rather than hardcoding values.

**Compose:** Break large launch files into smaller, reusable pieces.

**Document:** Add comments explaining what each section does.

**Test Incrementally:** Start with one node, verify, then add more.

**Use Namespaces:** Organize nodes into namespaces for clarity.

**Handle Errors:** Consider what happens if nodes fail to start.

## Parameter Management

### What are Parameters?

Parameters configure node behavior without recompiling:

**Runtime Configuration:** Change settings without code changes.

**Environment Adaptation:** Different parameters for simulation vs. hardware.

**Tuning:** Adjust control gains, thresholds, timeouts during development.

**Multi-Instance:** Same node with different parameters for different sensors.

### Parameter Types

ROS 2 supports several parameter types:

**Boolean:** true/false flags
**Integer:** Whole numbers
**Double:** Floating-point numbers
**String:** Text values
**Arrays:** Lists of values (integer, double, string arrays)
**Byte Arrays:** Binary data

### Parameter Declaration

Nodes must declare parameters before using them:

**Conceptual Process:**
1. Node declares parameter with name, type, and optional default
2. ROS 2 checks if parameter was set externally (launch file, command line)
3. If set externally, use that value; otherwise use default
4. Node accesses parameter value during execution

**Static vs. Dynamic:**
- **Static Parameters:** Cannot change after node starts. Set once at initialization.
- **Dynamic Parameters:** Can be modified while node runs. Node must handle changes.

### Parameter Sources

Parameters can come from multiple sources (in priority order):

1. **Command Line:** Highest priority
   ```
   ros2 run pkg node --ros-args -p param:=value
   ```

2. **Launch Files:** Python, XML, or YAML launch files
   ```python
   parameters=[{'my_param': 42}]
   ```

3. **YAML Files:** Loaded by launch file
   ```yaml
   node_name:
     ros__parameters:
       my_param: 42
   ```

4. **Default Values:** Declared in node code
   ```
   declare_parameter('my_param', 42)
   ```

This hierarchy allows setting defaults in code, common values in YAML, and overrides in launch files or command line.

### Parameter YAML Files

YAML files organize parameters hierarchically:

```yaml
/camera_node:
  ros__parameters:
    frame_rate: 30
    resolution: "1920x1080"
    auto_exposure: true
    exposure_value: 100

/detector_node:
  ros__parameters:
    confidence_threshold: 0.8
    max_detections: 10
    model_path: "/models/detector.onnx"
```

The namespace (`/camera_node`) matches the node name, ensuring parameters go to the correct node.

### Dynamic Reconfiguration

Some parameters can change at runtime:

**Parameter Callbacks:** Nodes register callbacks invoked when parameters change.

**Validation:** Callbacks can reject invalid values.

**State Updates:** Node updates internal state based on new parameter values.

**Example Use Cases:**
- Adjusting camera exposure while running
- Tuning PID control gains during testing
- Changing detection thresholds based on lighting
- Enabling/disabling debug output

### Parameter Best Practices

**Use Descriptive Names:** `camera_frame_rate` better than `cfr`.

**Provide Defaults:** Every parameter should have a sensible default.

**Document Parameters:** Explain what each parameter does, units, valid ranges.

**Validate Values:** Check for valid ranges, don't assume correctness.

**Group Related Parameters:** Use consistent naming schemes for related parameters.

**Version Parameters:** Include parameter file version for compatibility tracking.

## Build System: Colcon Concepts

### What is Colcon?

Colcon is ROS 2's build tool. It:

- Discovers packages in workspace
- Determines build order based on dependencies
- Invokes appropriate build system for each package (CMake, Python setuptools)
- Manages installation and environment setup
- Supports parallel builds for speed

### Build Process Overview

When you run `colcon build`, several steps occur:

**Discovery:** Colcon scans `src/` directory for packages (identified by `package.xml`).

**Dependency Resolution:** Reads `package.xml` to determine package dependencies and build order.

**Build Planning:** Determines which packages can build in parallel (no dependency between them).

**Build Execution:** For each package:
1. Generate build files (CMake, setuptools)
2. Compile source code
3. Link libraries and executables
4. Run build-time tests (optional)
5. Install to `install/` directory

**Environment Setup:** Generate setup scripts that configure environment to use built packages.

### Dependency Management

Package dependencies ensure correct build order:

**Build Dependencies:** Required to compile the package.
```xml
<build_depend>rclcpp</build_depend>
```

**Execution Dependencies:** Required to run the package.
```xml
<exec_depend>sensor_msgs</exec_depend>
```

**Build and Execution:** Required for both.
```xml
<depend>geometry_msgs</depend>
```

Colcon ensures packages are built after their dependencies.

### Incremental Builds

Colcon performs incremental builds—only rebuilding changed packages:

**Change Detection:** Tracks file modification times.

**Selective Rebuild:** Only rebuilds packages with changes and packages depending on them.

**Speed Improvement:** Avoids unnecessary recompilation.

**Force Rebuild:** Can force rebuild of specific packages if needed.

### Build Profiles and Options

Colcon supports build customization:

**Debug vs. Release:** Different optimization levels
```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**Parallel Jobs:** Control number of parallel builds
```
colcon build --parallel-workers 4
```

**Specific Packages:** Build only certain packages
```
colcon build --packages-select my_package
```

**Symlink Install:** For Python, link instead of copy for faster iteration
```
colcon build --symlink-install
```

### Workspace Overlay

ROS 2 supports workspace overlaying—building workspace that extends another:

**Underlay:** Base workspace (often ROS 2 installation)

**Overlay:** Your workspace, adds/overrides packages

**Sourcing Order:**
1. Source underlay: `source /opt/ros/humble/setup.bash`
2. Build overlay: `colcon build`
3. Source overlay: `source install/setup.bash`

This allows working with stable base ROS 2 while developing custom packages in overlay.

## Multi-Node System Design

### Design Principles

Effective multi-node systems follow key principles:

**Modularity:** Each node has single, well-defined responsibility.

**Loose Coupling:** Nodes interact through well-defined interfaces (topics, services, actions).

**Fault Isolation:** Node failures don't cascade to entire system.

**Scalability:** Can add nodes without redesigning system.

**Testability:** Individual nodes can be tested in isolation.

### Decomposition Strategies

How to divide functionality into nodes:

**By Hardware:** One node per sensor or actuator.
- Advantages: Hardware abstraction, easy to swap hardware
- Example: `camera_node`, `lidar_node`, `motor_controller_node`

**By Function:** One node per logical function.
- Advantages: Clear responsibilities, reusable
- Example: `object_detector`, `path_planner`, `localizer`

**By Frequency:** Nodes grouped by update rate.
- Advantages: Efficient scheduling, real-time support
- Example: `high_freq_controller` (1kHz), `planner` (10Hz), `ui` (1Hz)

**Hybrid:** Combine strategies as appropriate.

### Communication Architecture

Design communication patterns carefully:

**Data Flow:** Understand how data flows through system. Draw diagrams showing topic connections.

**Latency Budget:** Know maximum acceptable latency for each path. Camera→Detector→Planner→Controller.

**Bandwidth Requirements:** High-resolution images require more bandwidth than pose estimates.

**QoS Selection:** Match QoS to requirements. Commands need reliability, sensor streams can use best-effort.

### Namespace Organization

Organize nodes into logical namespaces:

```
/robot1/
  /sensors/
    /camera/
      /front/
        - image_raw
        - camera_info
      /rear/
        - image_raw
        - camera_info
    /lidar/
      - scan
  /perception/
    - detections
    - obstacles
  /planning/
    - path
  /control/
    - cmd_vel
    - joint_commands
```

Namespaces prevent naming conflicts and clarify system structure.

### Lifecycle Management

Managing node startup and shutdown:

**Startup Order:** Some nodes depend on others being ready (e.g., planners need localization).

**Health Monitoring:** Detect when nodes fail and restart or alert.

**Graceful Shutdown:** Ensure nodes clean up resources properly.

**State Synchronization:** Ensure consistent system state during transitions.

**Launch Files:** Encode startup order and dependencies.

### System-Level Design Example

Humanoid robot navigation system:

**Sensor Layer:**
- `camera_driver`: Publishes `/camera/image_raw`
- `imu_driver`: Publishes `/imu/data`
- `lidar_driver`: Publishes `/scan`

**Perception Layer:**
- `visual_odometry`: Subscribes `/camera/image_raw`, `/imu/data`; publishes `/vo/odom`
- `scan_matcher`: Subscribes `/scan`; publishes `/scan_odom`
- `sensor_fusion`: Subscribes `/vo/odom`, `/scan_odom`; publishes `/odometry`

**Planning Layer:**
- `global_planner`: Action server `/navigate_to_pose`; publishes `/global_plan`
- `local_planner`: Subscribes `/global_plan`, `/odometry`, `/scan`; publishes `/cmd_vel`

**Control Layer:**
- `motor_controller`: Subscribes `/cmd_vel`; publishes `/joint_states`

Each layer builds on previous layer, data flowing sensors→perception→planning→control.

## Conceptual Diagrams

### Diagram 1: Package Structure

```
ROS 2 Package Anatomy:

my_robot_package/
│
├── Package Metadata
│   ├── package.xml (dependencies, version, license)
│   └── CMakeLists.txt / setup.py (build instructions)
│
├── Source Code
│   ├── src/ (C++ implementations)
│   ├── include/ (C++ headers)
│   └── my_robot_package/ (Python modules)
│
├── Interfaces
│   ├── msg/ (custom message types)
│   ├── srv/ (custom service types)
│   └── action/ (custom action types)
│
├── Configuration
│   ├── launch/ (launch files)
│   ├── config/ (parameter YAML files)
│   └── urdf/ (robot descriptions)
│
└── Testing
    └── test/ (unit and integration tests)

Package relationships:

[Package A] ──depends on──> [Package B]
     ↓                           ↓
 [builds after]           [builds first]
```

### Diagram 2: Publisher-Subscriber Flow

```
Publisher-Subscriber Detailed Flow:

Publisher Side:
[Node Code]
     ↓
[create_publisher(topic, type, qos)]
     ↓
[DDS registers publisher]
     ↓
[DDS advertises on network]
     ↓ (discovery protocol)
[DDS finds compatible subscribers]
     ↓
[DDS establishes connection]
     ↓
[publish(message)]
     ↓
[Serialize message to bytes]
     ↓
[DDS transmits (UDP/shared memory)]

Subscriber Side:
[Node Code]
     ↓
[create_subscription(topic, type, callback, qos)]
     ↓
[DDS registers subscriber]
     ↓
[DDS advertises on network]
     ↓ (discovery protocol)
[DDS finds compatible publishers]
     ↓
[DDS establishes connection]
     ↓
[DDS receives bytes]
     ↓
[Deserialize to message object]
     ↓
[Invoke callback(message)]
     ↓
[User callback executes]
```

### Diagram 3: Communication Pattern Selection

```
Decision Tree for Communication Pattern:

Start: Need communication between nodes
   ↓
Question: Is this continuous data stream?
   ├─ Yes → USE TOPIC
   │         Examples: sensor data, state, commands
   │
   └─ No → Question: Is this a long-running task?
           ├─ Yes → USE ACTION
           │         Examples: navigation, manipulation, trajectories
           │
           └─ No → USE SERVICE
                     Examples: config changes, queries, computations

Topic Characteristics:
   ✓ Continuous/streaming
   ✓ Latest value matters
   ✓ One-to-many
   ✓ Asynchronous
   ✗ No confirmation
   ✗ No feedback

Service Characteristics:
   ✓ Request-response
   ✓ Confirmation needed
   ✓ Quick execution
   ✓ Synchronous
   ✗ One-to-one only
   ✗ Blocking
   ✗ No progress updates

Action Characteristics:
   ✓ Long-running
   ✓ Progress feedback
   ✓ Cancellable
   ✓ Asynchronous
   ✓ Confirmation (result)
   ✗ More complex
   ✗ One-to-one only
```

### Diagram 4: Launch System Hierarchy

```
Launch File Composition:

main.launch.py
│
├── sensors.launch.py
│   ├── Node: camera_driver
│   ├── Node: lidar_driver
│   └── Node: imu_driver
│
├── perception.launch.py
│   ├── Include: object_detection.launch.py
│   │   ├── Node: detector
│   │   └── Node: tracker
│   └── Include: localization.launch.py
│       ├── Node: visual_odometry
│       └── Node: sensor_fusion
│
└── control.launch.py
    ├── Node: motor_controller
    ├── Node: safety_monitor
    └── Include: robot_description.launch.py
        └── Node: robot_state_publisher

Execution:
1. Parse main.launch.py
2. Recursively include all sub-launch files
3. Resolve all parameters and remappings
4. Start nodes in dependency order
5. Monitor node health
```

### Diagram 5: Multi-Node System Architecture

```
Humanoid Robot Multi-Node System:

Sensor Nodes:
┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐
│ Camera  │  │  LiDAR  │  │   IMU   │  │  Force  │
└────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘
     │            │            │            │
     └────────────┴────────────┴────────────┘
                      ↓ (topics)
              Perception Nodes:
     ┌────────────────┬────────────────┐
     ↓                ↓                ↓
┌─────────┐  ┌──────────────┐  ┌───────────┐
│ Detector│  │ Localization │  │Point Cloud│
└────┬────┘  └──────┬───────┘  └─────┬─────┘
     │              │                 │
     └──────────────┴─────────────────┘
                    ↓ (topics)
           Planning Nodes:
     ┌──────────────┬──────────────┐
     ↓              ↓              ↓
┌─────────┐  ┌────────────┐  ┌─────────┐
│ Global  │  │   Local    │  │  Task   │
│ Planner │  │  Planner   │  │ Planner │
└────┬────┘  └─────┬──────┘  └────┬────┘
     │             │              │
     └─────────────┴──────────────┘
                   ↓ (actions, topics)
          Control Nodes:
     ┌─────────────┬──────────────┐
     ↓             ↓              ↓
┌──────────┐  ┌─────────┐  ┌──────────┐
│  Joint   │  │ Balance │  │  Safety  │
│Controller│  │Controller│  │ Monitor  │
└─────┬────┘  └────┬────┘  └────┬─────┘
      │            │            │
      └────────────┴────────────┘
                   ↓
            [Hardware Interface]
                   ↓
              [Robot Motors]

Data Flow: Sensors → Perception → Planning → Control → Hardware
```

## Key Concepts Summary

### Package
Fundamental organizational unit containing nodes, launch files, configuration, and dependencies. Encapsulates related functionality.

### Publisher
Component that produces data on a topic. Announces topic and type, serializes messages, sends via DDS.

### Subscriber
Component that consumes data from a topic. Registers interest, receives messages, invokes callbacks.

### Launch File
Script that starts multiple nodes with configuration, parameters, and remappings. Enables one-command system startup.

### Parameter
Runtime configuration value. Can be static (set at startup) or dynamic (changeable during execution).

### Colcon
Build tool that discovers packages, resolves dependencies, invokes build systems, and manages installation.

### Namespace
Hierarchical naming scheme for organizing topics, services, and nodes. Prevents naming conflicts.

### Workspace
Directory containing source, build, and install spaces for ROS 2 packages. Supports overlaying for extending functionality.

## Knowledge Checkpoint

Test your understanding of this chapter's concepts:

1. **Package Organization:**
   - Why is single responsibility important for package design?
   - You're building a camera driver that also performs image processing. Should this be one package or two? Justify your answer.
   - Explain the purpose of `package.xml` and why dependencies must be declared accurately.

2. **Communication Patterns:**
   - A humanoid robot must continuously send joint position updates to a visualization tool. Which communication pattern and why?
   - A planning node needs to compute inverse kinematics for a target pose. The computation takes 50ms. Which communication pattern and why?
   - A robot navigates to a goal 20 meters away, taking 30 seconds. You want progress updates every second and ability to cancel. Which communication pattern?

3. **Publishers and Subscribers:**
   - Explain how DDS enables publishers and subscribers to find each other without a central master.
   - Why is message serialization necessary for network communication?
   - What happens if a subscriber has "Reliable" QoS but the publisher has "Best Effort" QoS?

4. **Launch Systems:**
   - You have 15 nodes to start for your humanoid robot. Why use a launch file instead of manually starting each node?
   - Explain the benefits of composing launch files (including other launch files) versus one monolithic launch file.
   - How would you use conditional launching to start different nodes in simulation versus on real hardware?

5. **Parameters:**
   - What is the advantage of using parameters versus hardcoding values in node source code?
   - Explain the parameter priority hierarchy (command line, launch file, YAML file, default).
   - When would you use dynamic parameters that can change at runtime versus static parameters set at startup?

6. **Build System:**
   - Why does colcon need to resolve package dependencies before building?
   - Explain the difference between build dependencies and execution dependencies.
   - What is workspace overlaying and why is it useful?

7. **System Design:**
   - Design the node structure for a humanoid robot that must detect objects, navigate around obstacles, and grasp target objects. Identify nodes, topics, services, and actions.
   - Should sensor drivers run in the same node as perception algorithms? Why or why not?
   - Explain how namespaces help organize a complex multi-robot system.

## Chapter Summary

This chapter explored building systems with ROS 2:

**Package Structure:** Packages are the fundamental organizational unit, containing nodes, launch files, configuration, and interface definitions. Standard directory structure and naming conventions promote consistency and discoverability.

**Publishers and Subscribers:** Topic-based communication uses DDS for discovery, serialization, and distribution. Publishers and subscribers are decoupled, enabling flexible many-to-many data flow. Zero-copy transport optimizes performance for large messages.

**Communication Pattern Selection:** Topics for streaming data, services for request-response, actions for long-running tasks with feedback. Choosing appropriately is critical for system performance and maintainability.

**Launch Systems:** Launch files automate starting multi-node systems with correct configuration. Composition, conditional launching, and parameterization enable reusable, flexible launch configurations.

**Parameter Management:** Parameters enable runtime configuration without recompilation. YAML files, launch files, and command-line arguments provide flexible configuration sources. Dynamic parameters support runtime tuning.

**Build System:** Colcon discovers packages, resolves dependencies, and orchestrates builds. Incremental building and parallel compilation optimize development workflows. Workspace overlaying extends base installations.

**Multi-Node Design:** Effective systems decompose functionality into focused, loosely-coupled nodes. Careful communication architecture, namespace organization, and lifecycle management create robust, scalable systems.

These tools and patterns form the foundation for building the humanoid robot systems we develop in subsequent chapters.

## Further Reading

**Official Documentation:**
- ROS 2 Package Creation Tutorials
- ROS 2 Launch System Documentation
- Colcon Build System Guide
- ROS 2 Design Patterns

**Books:**
- "A Concise Introduction to Robot Programming with ROS 2" by Francisco Martín Rico
- "ROS 2 Cookbook" (community resource)

**Papers:**
- "Design Patterns for Robot Programming with ROS" (various authors)
- "ROS 2 Package Architecture Best Practices"

**Online Resources:**
- ROS 2 Examples and Demos (ros2/examples repository)
- ROS 2 Launch File Examples
- Parameter Management Tutorials

**Community Resources:**
- ROS Discourse (community discussion)
- ROS Answers (Q&A)
- GitHub ROS 2 repositories

## Looking Ahead

With understanding of building systems in ROS 2, we now turn specifically to humanoid robots. The next chapter explores robot description formats (URDF), coordinate frame management (tf2), controller interfaces, and bridging AI agents to robot control. These humanoid-specific tools and patterns build on the ROS 2 foundation established in this chapter.
