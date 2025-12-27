# Chapter 5: ROS 2 for Humanoid Robots

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand URDF (Unified Robot Description Format) and how it represents robot structure
- Analyze kinematic chains and joint hierarchies in humanoid robots
- Explain the tf2 transform system and coordinate frame management
- Describe the roles of robot_state_publisher and joint_state_publisher
- Understand controller interfaces and the ros2_control framework
- Design architectures for bridging AI agents to robot controllers
- Utilize RViz2 for visualization and debugging humanoid systems

## Introduction

Humanoid robots present unique challenges compared to simpler robotic systems. A humanoid has 20-30+ joints arranged in kinematic chains (legs, arms, torso, head), each joint requiring coordinated control. The robot's structure must be described precisely for physics simulation, collision checking, and motion planning. Coordinate frames must be managed for sensors, end effectors, and the environment. AI systems must translate high-level intentions into low-level motor commands.

ROS 2 provides specialized tools for humanoid robotics: URDF for describing robot geometry and kinematics, tf2 for managing coordinate transformations, ros2_control for hardware abstraction and controller management, and RViz2 for visualization. This chapter explores these humanoid-specific tools and patterns, building on the ROS 2 foundations from previous chapters.

## URDF: Unified Robot Description Format

### What is URDF?

URDF is an XML format for describing robot structure. A URDF file defines:

**Links:** Rigid bodies (bones in humanoid analogy). Each link has:
- Visual geometry (for visualization)
- Collision geometry (for collision checking)
- Inertial properties (mass, center of mass, inertia tensor)

**Joints:** Connections between links (like biological joints). Each joint has:
- Type (revolute, prismatic, fixed, continuous, planar, floating)
- Parent and child links
- Axis of rotation/translation
- Limits (position, velocity, effort)
- Dynamics (damping, friction)

**Sensors:** Cameras, LiDAR, IMU specifications and mounting locations

**Actuators:** Motor specifications (for simulation)

The URDF provides a complete geometric and kinematic description that physics simulators, motion planners, and visualization tools can interpret.

### URDF Structure

A simple URDF snippet for a joint and links:

```xml
<robot name="humanoid">
  <!-- Base link (torso) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="15.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Shoulder joint -->
  <joint name="right_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="upper_arm_right"/>
    <origin xyz="0.15 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <!-- Upper arm link -->
  <link name="upper_arm_right">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
</robot>
```

This defines a torso link, a revolute shoulder joint, and an upper arm link. The joint connects torso (parent) to upper arm (child), rotates around the Y-axis, and has position and torque limits.

### Joint Types

URDF supports several joint types:

**Revolute:** Rotates around an axis with limits (e.g., elbow, knee)
- Range: limited (e.g., -90° to +90°)
- Use: Most humanoid joints

**Continuous:** Rotates around an axis without limits (e.g., wheel)
- Range: unlimited
- Use: Rare in humanoids (perhaps head yaw in some designs)

**Prismatic:** Translates along an axis (e.g., linear actuator)
- Range: limited linear motion
- Use: Telescoping mechanisms, grippers

**Fixed:** No motion (rigid connection)
- Use: Connecting sensor mounts, cosmetic parts

**Planar:** Moves in a plane
- Use: Specialized mechanisms

**Floating:** Six degrees of freedom (3 translation, 3 rotation)
- Use: Base of mobile robot relative to world frame

Humanoid joints are predominantly revolute (shoulders, elbows, hips, knees, ankles), with fixed joints for sensor mounts and floating joints for the base.

### Visual vs. Collision Geometry

URDF separates visual and collision geometry for efficiency:

**Visual Geometry:**
- High-detail meshes for realistic visualization
- Complex textures and colors
- Computationally expensive for collision checking
- Example: Detailed humanoid skin mesh

**Collision Geometry:**
- Simplified shapes (boxes, cylinders, spheres)
- Approximate the link's envelope
- Fast collision checking
- Example: Cylinder approximating arm

This separation allows beautiful visualization while maintaining real-time collision detection performance.

### Inertial Properties

Accurate inertial properties are critical for physics simulation:

**Mass:** Total mass of the link (kilograms)

**Center of Mass:** Point around which mass is distributed (relative to link frame)

**Inertia Tensor:** 3x3 matrix describing resistance to rotation
- Diagonal elements (Ixx, Iyy, Izz): moments of inertia around principal axes
- Off-diagonal elements (Ixy, Ixz, Iyz): products of inertia

Inaccurate inertial properties cause unrealistic simulation behavior—robots that collapse, spin uncontrollably, or float. For real humanoid robots, inertial properties should match physical measurements.

### Xacro: Programmable URDF

Raw URDF is verbose and repetitive. Xacro (XML Macros) adds programming constructs:

**Variables:**
```xml
<xacro:property name="arm_length" value="0.3"/>
<cylinder length="${arm_length}" radius="0.05"/>
```

**Macros:**
```xml
<xacro:macro name="arm" params="prefix">
  <link name="${prefix}_upper_arm">...</link>
  <joint name="${prefix}_shoulder">...</joint>
  <link name="${prefix}_forearm">...</link>
  <joint name="${prefix}_elbow">...</joint>
</xacro:macro>

<!-- Instantiate for both arms -->
<xacro:arm prefix="left"/>
<xacro:arm prefix="right"/>
```

**Includes:**
```xml
<xacro:include filename="common_materials.xacro"/>
<xacro:include filename="sensors.xacro"/>
```

Xacro reduces duplication, especially for symmetric humanoids (left/right arms and legs identical).

### URDF Best Practices

**Modular Design:** Separate robot into logical components (torso, arms, legs, head).

**Consistent Naming:** Use clear, systematic names (`left_shoulder_pitch`, not `ls_p`).

**Coordinate Frame Conventions:** Follow ROS standards (X forward, Y left, Z up).

**Realistic Inertia:** Use CAD software or measurement to get accurate inertial properties.

**Simplified Collision:** Use primitive shapes when possible for performance.

**Test Incrementally:** Build URDF incrementally, testing visualization and simulation at each step.

## Kinematic Chains and Joint Hierarchies

### Understanding Kinematic Chains

A kinematic chain is a series of rigid links connected by joints. In humanoids:

**Serial Chains:** Links connected end-to-end (e.g., shoulder → upper arm → forearm → wrist → hand)

**Tree Structure:** Humanoids are kinematic trees with the torso as root and limbs as branches

**Degrees of Freedom:** Number of independent joint motions. A typical humanoid:
- Torso: 3 DOF (roll, pitch, yaw)
- Arm: 7 DOF (3 shoulder, 1 elbow, 3 wrist)
- Leg: 6 DOF (3 hip, 1 knee, 2 ankle)
- Head: 2 DOF (pan, tilt)
- Hand: 5+ DOF (finger joints)
- Total: 30+ DOF

### Joint Hierarchy Representation

Humanoid joint hierarchy (simplified):

```
[base_link] (floating base)
    │
[torso]
    ├── [left_upper_leg]
    │       └── [left_lower_leg]
    │               └── [left_foot]
    │
    ├── [right_upper_leg]
    │       └── [right_lower_leg]
    │               └── [right_foot]
    │
    ├── [left_upper_arm]
    │       └── [left_forearm]
    │               └── [left_hand]
    │
    ├── [right_upper_arm]
    │       └── [right_forearm]
    │               └── [right_hand]
    │
    └── [head]
```

Each link inherits transformations from its parent. Moving the torso moves all limbs; moving the shoulder moves the entire arm.

### Forward Kinematics

Forward kinematics computes end effector position/orientation from joint angles:

**Input:** Joint angles (θ1, θ2, ..., θn)

**Output:** End effector pose (position + orientation) in world frame

**Process:**
1. Start at base link
2. Apply transformation from each joint in chain
3. Multiply transformations sequentially
4. Result is end effector pose

**Example:** Given shoulder, elbow, and wrist angles, compute hand position.

Forward kinematics is straightforward—just matrix multiplication through the chain.

### Inverse Kinematics

Inverse kinematics computes joint angles needed to achieve a target end effector pose:

**Input:** Desired end effector pose

**Output:** Joint angles (θ1, θ2, ..., θn)

**Challenge:** Inverse kinematics is much harder than forward kinematics:
- May have no solution (target unreachable)
- May have multiple solutions (elbow up vs. elbow down)
- May have infinite solutions (redundant arms with 7 DOF)
- Computational complexity increases with DOF

**Solution Methods:**
- Analytical: Closed-form equations (fast but limited to specific geometries)
- Numerical: Iterative optimization (general but slower)
- Learning-based: Neural networks trained on IK solutions

Motion planning systems heavily rely on IK to convert desired end effector goals into joint trajectories.

### Jacobian and Differential Kinematics

The Jacobian matrix relates joint velocities to end effector velocities:

**Jacobian (J):** Partial derivatives of end effector position with respect to joint angles

**Velocity Relationship:** ẋ = J θ̇
- ẋ: End effector velocity (6D: 3 linear, 3 angular)
- θ̇: Joint velocities
- J: 6 × n Jacobian matrix (n = number of joints)

**Uses:**
- **Velocity Control:** Compute joint velocities for desired end effector velocity
- **Singularity Detection:** When J loses rank, robot loses DOF (singularity)
- **Force Control:** Map joint torques to end effector forces

The Jacobian is fundamental to many robot control algorithms.

## tf2: Transform System

### Why Transform Management Matters

Humanoid robots have dozens of coordinate frames:

**Robot Frames:**
- Base frame (robot origin)
- Each link has a frame
- Each joint has a frame
- End effectors (hands, feet) have frames

**Sensor Frames:**
- Camera frame
- LiDAR frame
- IMU frame
- Force sensor frames

**World Frames:**
- Map frame (global reference)
- Odom frame (odometry reference)
- Base frame (robot's current location)

Questions requiring transforms:
- Where is the detected object (camera frame) relative to the robot's hand (hand frame)?
- What is the robot's position (base frame) in the map (map frame)?
- Where should the foot (foot frame) be placed relative to the detected step (world frame)?

tf2 manages these coordinate transformations automatically.

### tf2 Architecture

tf2 maintains a transform tree:

**Tree Structure:** Frames connected in a tree (no loops). Each frame has one parent.

**Transform Specification:** Each transform specifies:
- Parent frame
- Child frame
- Translation (x, y, z)
- Rotation (quaternion: x, y, z, w)
- Timestamp (when transform is valid)

**Transform Lookup:** Query "what is the transform from frame A to frame B at time t?"

**Chain Computation:** tf2 finds the path through the tree from A to B and multiplies transforms along the path.

### Static vs. Dynamic Transforms

**Static Transforms:** Don't change over time
- Sensor mounting positions (camera on head)
- Link-to-link transforms (defined by URDF)
- Calibration offsets

**Dynamic Transforms:** Change over time
- Joint angles (link positions change as joints move)
- Robot pose in world (changes as robot moves)
- Object tracking (objects move in environment)

Static transforms published once; dynamic transforms published continuously.

### Common Frame Conventions

ROS 2 follows standard naming and conventions:

**map:** Global fixed frame (environment map reference)

**odom:** Odometry frame (continuous, drift-prone estimate of robot pose)

**base_link:** Robot's base (usually at center of mass or between feet)

**Relationship:**
- map → odom: Corrects odometry drift (from localization)
- odom → base_link: Odometry estimate (from wheel encoders, VIO)
- base_link → sensors/links: Robot structure (from URDF + joint states)

**Coordinate Convention:**
- X: Forward
- Y: Left
- Z: Up

Following conventions ensures interoperability between packages.

### Transform Broadcasting

Nodes publish transforms to tf2:

**Static Transform Broadcaster:**
- Publishes fixed transforms
- Example: Camera mount position on robot head
- Published once at startup or infrequently

**Transform Broadcaster:**
- Publishes dynamic transforms
- Example: Robot position in world, joint angles
- Published continuously (10-50 Hz typically)

**Example:** `robot_state_publisher` broadcasts transforms for all robot links based on joint angles.

### Transform Listening

Nodes query transforms from tf2:

**Transform Listener:**
- Subscribes to transform topics
- Maintains transform buffer (history of transforms)
- Provides lookup interface

**Lookup Query:**
- Source frame
- Target frame
- Time (can query historical transforms)

**Example:** Motion planner queries "hand position in base frame" to check if target is reachable.

### Transform Tree Example

Humanoid robot transform tree:

```
[map]
  ↓
[odom]
  ↓
[base_link]
  ├── [torso]
  │     ├── [head]
  │     │     ├── [camera_left]
  │     │     └── [camera_right]
  │     ├── [left_shoulder]
  │     │     └── [left_upper_arm]
  │     │           └── [left_forearm]
  │     │                 └── [left_hand]
  │     └── [right_shoulder]
  │           └── [right_upper_arm]
  │                 └── [right_forearm]
  │                       └── [right_hand]
  ├── [left_hip]
  │     └── [left_upper_leg]
  │           └── [left_lower_leg]
  │                 └── [left_foot]
  └── [right_hip]
        └── [right_upper_leg]
              └── [right_lower_leg]
                    └── [right_foot]
```

To find "left hand position in map frame", tf2 multiplies transforms along path: map → odom → base_link → torso → left_shoulder → left_upper_arm → left_forearm → left_hand.

### Time Synchronization

tf2 stores transform history, enabling time-synchronized queries:

**Use Case:** Camera image captured at t=10.5s shows object. Robot's hand was at different position then vs. now (t=11.0s). Need transform at t=10.5s to accurately plan grasp.

**Buffer Duration:** Typically 10-30 seconds of history. Older transforms discarded.

**Interpolation:** If exact timestamp not available, tf2 interpolates between nearby transforms.

## robot_state_publisher and joint_state_publisher

### robot_state_publisher

The `robot_state_publisher` node is central to humanoid robot operation:

**Purpose:** Publishes transforms for all robot links based on URDF and current joint states.

**Inputs:**
- URDF (robot description)
- Joint states (joint positions from controllers or sensors)

**Outputs:**
- tf2 transforms for all links

**Process:**
1. Parse URDF to understand kinematic tree
2. Subscribe to `/joint_states` topic
3. For each joint state update:
   - Compute forward kinematics for all links
   - Publish transforms to tf2

**Why Essential:** Provides real-time robot geometry to all other nodes. Motion planners, collision checkers, and visualizers need to know where every link is.

### joint_state_publisher

The `joint_state_publisher` node provides joint states when hardware is not available:

**Purpose:** Publish joint states for testing/visualization without real robot.

**Use Cases:**
- **Simulation:** Generate random or scripted joint movements
- **Debugging:** Manually control joints via GUI to test robot description
- **Demonstration:** Show robot capabilities without hardware

**GUI Mode:** Provides sliders to manually control each joint position.

**Not for Real Robots:** On real hardware, joint states come from motor encoders or other sensors, not `joint_state_publisher`.

### Joint State Message

Joint states published on `/joint_states` topic use `sensor_msgs/JointState`:

```
Header header
string[] name           # Joint names
float64[] position      # Joint positions (radians or meters)
float64[] velocity      # Joint velocities (rad/s or m/s)
float64[] effort        # Joint efforts (torques or forces)
```

**Example:**
```
name: ['right_shoulder_pitch', 'right_elbow', 'right_wrist_roll']
position: [0.5, 1.2, -0.3]  # radians
velocity: [0.1, 0.0, 0.05]  # rad/s
effort: [2.5, 1.0, 0.3]     # Nm
```

All controllers and sensors must publish joint states in this format for `robot_state_publisher` to process.

### Workflow: URDF to Visualization

Typical workflow for humanoid robot:

1. **URDF Creation:** Define robot structure (links, joints, geometry)
2. **Parameter Server:** Load URDF to ROS 2 parameter server (typically via launch file)
3. **robot_state_publisher:** Reads URDF, subscribes to `/joint_states`
4. **Joint Source:** Either:
   - `joint_state_publisher` (GUI for testing)
   - Hardware interface (real robot motor encoders)
   - Simulation (Gazebo, Isaac Sim publishes joint states)
5. **Transform Publishing:** `robot_state_publisher` computes and publishes all link transforms to tf2
6. **Visualization:** RViz2 subscribes to tf2 and `/robot_description` to display robot

This pipeline enables seeing the robot move in RViz2 as joints change.

## Controller Interfaces and ros2_control

### What is ros2_control?

`ros2_control` is a framework for robot control in ROS 2. It provides:

**Hardware Abstraction:** Unified interface for different motors, sensors, and actuators

**Controller Management:** Loading, starting, stopping, and switching controllers

**Real-Time Support:** Designed for real-time control loops (1-10 kHz)

**Modular Controllers:** Reusable controllers for common tasks (joint position, velocity, effort control)

**Plugin Architecture:** Extend with custom hardware interfaces and controllers

### ros2_control Architecture

The framework has three layers:

**Hardware Interface:**
- Communicates with physical hardware (motor drivers, sensors)
- Reads joint states (position, velocity, effort)
- Writes joint commands (position, velocity, effort)
- Implemented per robot (each robot needs custom hardware interface)

**Controller Manager:**
- Loads and manages controllers
- Routes commands and states between controllers and hardware
- Handles controller lifecycle (inactive, active, error states)
- Ensures only one controller writes to each joint

**Controllers:**
- Implement control algorithms
- Read joint states from hardware interface
- Write commands to hardware interface
- Examples: position controller, velocity controller, trajectory controller

**Data Flow:**
```
[Hardware] ←→ [Hardware Interface] ←→ [Controller Manager] ←→ [Controllers]
              (read states,              (route data,          (control
               write commands)            manage lifecycle)     algorithms)
```

### Hardware Interface Concepts

Hardware interface defines how to communicate with robot:

**Resource Declaration:** Specify available joints and interfaces (position, velocity, effort)

**State Interfaces:** Hardware provides current state
- position: Current joint angle
- velocity: Current joint velocity
- effort: Current joint torque

**Command Interfaces:** Hardware accepts commands
- position: Target joint angle
- velocity: Target joint velocity
- effort: Target joint torque

**Example:** A humanoid arm with 3 joints might export:
- State interfaces: `shoulder_pitch/position`, `elbow/position`, `wrist_roll/position`, plus velocity and effort for each
- Command interfaces: `shoulder_pitch/effort`, `elbow/effort`, `wrist_roll/effort` (torque control)

Controllers read state interfaces and write command interfaces.

### Common Controllers

ros2_control provides standard controllers:

**Joint State Broadcaster:**
- Reads joint states from hardware interface
- Publishes to `/joint_states` topic
- Required for `robot_state_publisher` to work

**Position Controller:**
- Accepts target joint positions
- Commands hardware to reach targets
- Simple PID or more advanced control

**Velocity Controller:**
- Accepts target joint velocities
- Maintains desired velocity

**Effort Controller:**
- Accepts target joint torques
- Direct torque control

**Joint Trajectory Controller:**
- Executes trajectories (sequences of waypoints with timing)
- Interpolates between waypoints
- Used by motion planners

**Admittance Controller:**
- Compliant control (responds to external forces)
- Essential for safe human-robot interaction

### Controller Manager

The controller manager orchestrates controllers:

**Loading:** Load controller plugins dynamically
```
ros2 control load_controller joint_trajectory_controller
```

**Configuration:** Controllers configured via parameters (gains, limits, etc.)

**Lifecycle Management:** Controllers have states:
- Unconfigured: Just loaded
- Inactive: Configured but not controlling
- Active: Actively controlling hardware
- Finalized: Cleaned up

**Switching:** Can switch between controllers:
- Stop position controller
- Start torque controller
- Ensures smooth transitions

**Exclusive Access:** Prevents multiple controllers from fighting over same joints

### Real-Time Considerations

Humanoid control requires real-time performance:

**Control Loop Frequency:** 1-10 kHz for balance and manipulation
- Balance: 1-2 kHz (fast response to disturbances)
- Joint control: 100-1000 Hz
- Planning: 10-100 Hz

**Determinism:** Control loops must execute at precise intervals. Late execution causes instability.

**Real-Time OS:** For hard real-time, use PREEMPT_RT Linux kernel

**ros2_control Design:** Separates real-time (hardware interface, controllers) from non-real-time (ROS 2 communication) components. Real-time components run in dedicated threads.

## Bridging AI Agents to ROS Controllers

### The AI-to-Robot Control Gap

Modern humanoid robots combine:

**High-Level AI:** Large language models, vision transformers, reinforcement learning policies
- Operate on abstract goals ("pick up the cup")
- Run at low frequency (1-10 Hz)
- Often run on GPU

**Low-Level Control:** Motor controllers, balance algorithms, trajectory execution
- Operate on joint commands
- Run at high frequency (100-1000 Hz)
- Often run on real-time CPU or microcontroller

**Gap:** AI outputs high-level intentions; controllers need low-level commands. Need architecture to bridge this gap.

### Hierarchical Control Architecture

Effective architecture uses hierarchy:

**Level 1: AI Agent (High-Level)**
- Input: Sensor data (images, point clouds), task description
- Processing: LLM reasoning, object detection, scene understanding
- Output: High-level goals ("grasp object at position X,Y,Z")
- Frequency: 1-10 Hz
- Hardware: GPU

**Level 2: Motion Planning (Mid-Level)**
- Input: High-level goals from AI
- Processing: Path planning, inverse kinematics, collision avoidance
- Output: Joint trajectories (sequences of joint angles with timing)
- Frequency: 10-50 Hz
- Hardware: CPU

**Level 3: Controllers (Low-Level)**
- Input: Joint trajectories from planners
- Processing: PID control, torque computation, balance
- Output: Motor commands (voltages, torques)
- Frequency: 100-1000 Hz
- Hardware: Real-time CPU or microcontroller

**Level 4: Hardware**
- Input: Motor commands from controllers
- Processing: Motor driver electronics
- Output: Actual joint motion
- Frequency: 1-10 kHz
- Hardware: Motor drivers, motors

**Data Flow:**
```
[AI Agent] --goals--> [Motion Planner] --trajectories--> [Controller] --commands--> [Hardware]
   ↑                        ↑                                ↑
   |                        |                                |
[Sensors] <---- [State Estimation] <---- [Joint States] <----+
```

### Interface Patterns

Several patterns bridge AI and control:

**Pattern 1: Action-Based Interface**

AI sends high-level goals as ROS 2 actions:

```
AI Agent --NavigateToPose action--> Navigation Stack
         --GraspObject action-----> Manipulation Planner
         --FollowPerson action----> Person Tracking

Each action server handles:
- Goal decomposition
- Motion planning
- Execution
- Feedback to AI
```

**Advantages:** Clean separation, asynchronous, progress feedback
**Use Case:** Discrete tasks with clear goals

**Pattern 2: Topic-Based Interface**

AI publishes commands on topics, controllers subscribe:

```
AI Agent --/cmd_vel topic--> Velocity Controller
         --/target_pose---> Position Controller
```

**Advantages:** Simple, low latency, continuous control
**Use Case:** Continuous control (teleoperation, learned policies)

**Pattern 3: Behavior Trees**

Hierarchical state machine coordinates AI and control:

```
[Behavior Tree]
    ├── Sequence: Grasp Object
    │     ├── [AI: Detect Object] --object pose--> context
    │     ├── [Planning: Compute IK] --joint angles--> context
    │     ├── [Control: Move Arm] --execute trajectory
    │     └── [Control: Close Gripper]
    └── Fallback: Recovery
          ├── [AI: Re-detect Object]
          └── [Control: Retreat to Safe Pose]
```

**Advantages:** Complex behaviors, error handling, modular
**Use Case:** Multi-step tasks requiring coordination

**Pattern 4: Shared Memory Interface**

AI and control share memory for high-frequency communication:

```
AI Policy (GPU) --shared memory--> Real-Time Controller (CPU)
   writes: target joint positions
   reads: current joint states
```

**Advantages:** Lowest latency, highest frequency
**Use Case:** Learned policies requiring tight control loop (RL policies)

### Example: LLM-Controlled Humanoid

Architecture for LLM-controlled humanoid:

**Components:**
1. **LLM Agent:**
   - Receives natural language commands
   - Reasons about task decomposition
   - Publishes high-level action goals

2. **Task Planner:**
   - Subscribes to LLM goals
   - Breaks down into motion primitives
   - Sends action requests to motion planners

3. **Motion Planners:**
   - Navigation planner (MoveIt2 or Nav2)
   - Manipulation planner (MoveIt2)
   - Whole-body planner
   - Generate trajectories

4. **Controller Manager:**
   - Executes trajectories via joint trajectory controller
   - Monitors execution, reports status

5. **Hardware Interface:**
   - Sends commands to motors
   - Reads joint states

**Communication:**
- LLM → Task Planner: ROS 2 service or topic
- Task Planner → Motion Planners: ROS 2 actions
- Motion Planners → Controllers: ROS 2 actions (FollowJointTrajectory)
- Controllers → Hardware: ros2_control interfaces

**Example Flow:**
```
User: "Pick up the red cup"
  ↓
[LLM]: Parse command, identify object "red cup", task "pick up"
  ↓
[LLM → Task Planner]: Goal: grasp(object="red cup", color="red")
  ↓
[Task Planner]:
  1. Detect red cup (call vision service)
  2. Plan grasp approach (call manipulation planner)
  3. Execute grasp trajectory
  ↓
[Vision Service]: Returns cup pose
  ↓
[Manipulation Planner]: Computes IK, plans collision-free path
  ↓
[Manipulation Planner → Controller]: FollowJointTrajectory action
  ↓
[Controller]: Executes trajectory, monitors joint states
  ↓
[Hardware]: Motors move arm to grasp cup
```

### Learned Policy Integration

For reinforcement learning policies:

**Training:** Policy trained in simulation (Gazebo, Isaac Sim)

**Deployment:** Policy runs in ROS 2 node

**Interface:**
- Policy subscribes to sensor topics (joint states, camera images)
- Policy publishes to command topics (joint positions or torques)
- Control frequency: Match training frequency (often 50-100 Hz)

**Challenges:**
- Sim-to-real gap: Policy trained in simulation may fail on real hardware
- Safety: Learned policies can be unpredictable; need safety monitors
- Timing: ROS 2 topic communication has jitter; may need shared memory for determinism

**Safety Wrapper:**
```
[Policy Node] --raw commands--> [Safety Monitor] --safe commands--> [Controller]
                                      ↑
                                      |
                               [Joint States]
                               [Collision Detector]
                               [Emergency Stop]
```

Safety monitor validates policy commands before sending to hardware.

## RViz2: Visualization and Debugging

### What is RViz2?

RViz2 is ROS 2's 3D visualization tool. It displays:

- Robot models (from URDF)
- Sensor data (cameras, LiDAR, point clouds)
- Coordinate frames (tf2)
- Planning results (trajectories, paths)
- Markers (for debugging: arrows, spheres, text)
- Maps and occupancy grids

RViz2 is essential for development, debugging, and demonstration.

### RViz2 Architecture

RViz2 subscribes to ROS 2 topics and visualizes data:

**Display Plugins:** Each data type has a display plugin
- RobotModel: Visualizes URDF
- Camera: Shows camera images
- PointCloud2: Renders point clouds
- TF: Shows coordinate frames
- Marker: Displays custom visualization markers

**Configuration:** RViz2 configurations saved to files, allowing project-specific setups

**Interactive Markers:** Allow interacting with visualization (move target poses, trigger actions)

### Visualizing the Robot

To see humanoid robot in RViz2:

**Requirements:**
1. URDF loaded to parameter server
2. `robot_state_publisher` running
3. Joint states being published
4. tf2 transforms available

**RViz2 Configuration:**
1. Add RobotModel display
2. Set robot description topic (typically `/robot_description`)
3. Robot appears, colored by link

**Interactive Control:** With `joint_state_publisher_gui`, slide joints to see robot move in real-time.

### Visualizing Sensor Data

**Camera Images:**
- Add Camera display
- Select image topic (e.g., `/camera/image_raw`)
- Image appears in 3D view or separate panel

**Point Clouds:**
- Add PointCloud2 display
- Select point cloud topic (e.g., `/scan` or `/camera/depth/points`)
- Configure color by intensity, height, or RGB
- Point cloud overlays robot model

**LiDAR Scans:**
- Add LaserScan display
- Select scan topic
- Configure visualization (points, flat squares, etc.)

### Visualizing Coordinate Frames

**TF Display:**
- Add TF display
- Toggle which frames to show (all frames can clutter view)
- Each frame shown as RGB axes (X=red, Y=green, Z=blue)
- Helps debug frame relationships and transform errors

**Use Cases:**
- Verify sensor mounting positions
- Debug transform tree structure
- Understand coordinate frame relationships

### Debugging with Markers

Markers visualize custom data:

**Marker Types:**
- Arrow: Direction vectors
- Sphere: Points of interest
- Line: Paths or connections
- Cube: Bounding boxes
- Text: Labels
- Mesh: Custom 3D shapes

**Publishing Markers:**
Nodes publish to `/visualization_marker` topic. Markers specify:
- Type, position, orientation
- Color, scale
- Lifetime (auto-deletion)

**Example Uses:**
- Show detected object bounding boxes
- Display planned path
- Indicate target grasp pose
- Show force vectors
- Label objects with text

### RViz2 for Humanoid Development

Typical RViz2 setup for humanoid:

**Displays:**
1. RobotModel: Show robot structure
2. TF: Key frames (base, hands, feet, cameras)
3. Camera: Front camera view
4. PointCloud2: Depth camera or LiDAR
5. Marker: Planning visualization
6. Path: Planned trajectories
7. Axes: Target poses

**Workflow:**
1. Start robot (hardware or simulation)
2. Launch RViz2 with configuration
3. See robot state in real-time
4. See sensor data overlaid on scene
5. See planning and control outputs
6. Debug by inspecting frames, data, and markers

### RViz2 vs. Simulation

Important distinction:

**RViz2:** Visualization only. Does not simulate physics. Shows what robot reports through topics and tf2.

**Gazebo/Isaac Sim:** Full physics simulation. Simulates robot dynamics, sensors, environment.

**Combined Use:**
- Run Gazebo for physics simulation
- Run RViz2 to visualize Gazebo's output
- RViz2 shows what Gazebo publishes (joint states, sensor data, tf)

Both tools serve different purposes: RViz2 for visualization and debugging, simulators for testing in virtual environments.

## Conceptual Diagrams

### Diagram 1: URDF Structure

```
URDF Anatomy:

Robot Description
│
├── Links (Rigid Bodies)
│   ├── Visual Geometry (what you see)
│   │     └── Mesh or primitives (box, cylinder, sphere)
│   ├── Collision Geometry (for collision checking)
│   │     └── Simplified shapes for performance
│   └── Inertial Properties
│         ├── Mass
│         ├── Center of mass
│         └── Inertia tensor
│
├── Joints (Connections)
│   ├── Type (revolute, prismatic, fixed, etc.)
│   ├── Parent Link
│   ├── Child Link
│   ├── Axis of motion
│   ├── Limits (position, velocity, effort)
│   └── Dynamics (damping, friction)
│
└── Sensors/Actuators
      └── Specifications and mounting

Example Hierarchy:
[torso]
   │
   ├─[joint: right_shoulder]─→ [link: right_upper_arm]
   │                               │
   │                               └─[joint: right_elbow]─→ [link: right_forearm]
   │
   └─[joint: left_shoulder]──→ [link: left_upper_arm]
                                   │
                                   └─[joint: left_elbow]──→ [link: left_forearm]
```

### Diagram 2: Kinematic Chain

```
Humanoid Kinematic Tree:

                    [base_link/torso]
                           |
        +------------------+------------------+
        |                  |                  |
    [left_leg]         [right_leg]         [arms & head]
        |                  |                  |
    [hip_yaw]          [hip_yaw]          [shoulders...]
        |                  |
    [hip_roll]         [hip_roll]
        |                  |
    [hip_pitch]        [hip_pitch]
        |                  |
    [knee]             [knee]
        |                  |
    [ankle_pitch]      [ankle_pitch]
        |                  |
    [ankle_roll]       [ankle_roll]
        |                  |
    [foot]             [foot]

Forward Kinematics Example (right arm):
    Joint Angles: θ = [θ_shoulder_pitch, θ_shoulder_roll, θ_shoulder_yaw, θ_elbow]
                                    ↓
    Transformations: T_base→hand = T₀ × T₁(θ_sp) × T₂(θ_sr) × T₃(θ_sy) × T₄(θ_e)
                                    ↓
    Hand Pose: [x, y, z, roll, pitch, yaw] in base frame

Inverse Kinematics Example:
    Desired Hand Pose: [x, y, z, roll, pitch, yaw]
                                    ↓
    Solver (analytical or numerical optimization)
                                    ↓
    Joint Angles: θ = [θ_shoulder_pitch, θ_shoulder_roll, θ_shoulder_yaw, θ_elbow]
```

### Diagram 3: tf2 Transform Tree

```
Transform Tree for Humanoid:

[map] (global reference)
  ↓ (localization correction)
[odom] (odometry frame)
  ↓ (odometry estimate)
[base_link] (robot base)
  ↓
[torso]
  ├─ [head]
  │    ├─ [camera_left]
  │    ├─ [camera_right]
  │    └─ [lidar]
  │
  ├─ [left_shoulder]
  │    ↓
  │  [left_upper_arm]
  │    ↓
  │  [left_forearm]
  │    ↓
  │  [left_hand]
  │    └─ [left_gripper]
  │
  ├─ [right_shoulder]
  │    ↓
  │  [right_upper_arm]
  │    ↓
  │  [right_forearm]
  │    ↓
  │  [right_hand]
  │    └─ [right_gripper]
  │
  ├─ [left_hip]
  │    ↓
  │  [left_upper_leg]
  │    ↓
  │  [left_lower_leg]
  │    ↓
  │  [left_foot]
  │
  └─ [right_hip]
       ↓
     [right_upper_leg]
       ↓
     [right_lower_leg]
       ↓
     [right_foot]

Transform Lookup Example:
Query: "camera_left to right_hand"
Path: camera_left → head → torso → right_shoulder → ... → right_hand
Result: Translation [x,y,z] and Rotation [qx,qy,qz,qw] at timestamp t
```

### Diagram 4: ros2_control Architecture

```
ros2_control Framework:

┌─────────────────────────────────────────────────────┐
│                  ROS 2 Ecosystem                    │
│  [Motion Planners] [Nav Stack] [Teleoperation]     │
└────────────────┬────────────────────────────────────┘
                 │ (topics, actions, services)
                 ↓
┌─────────────────────────────────────────────────────┐
│             Controller Manager                      │
│  ┌──────────────────────────────────────────────┐  │
│  │  Controllers:                                │  │
│  │  ┌─────────────────┐  ┌──────────────────┐  │  │
│  │  │Joint Trajectory │  │ Position Control │  │  │
│  │  │   Controller    │  │                  │  │  │
│  │  └────────┬────────┘  └────────┬─────────┘  │  │
│  │           │                    │            │  │
│  │  ┌────────▼────────┐  ┌────────▼─────────┐  │  │
│  │  │Joint State      │  │ Effort Control   │  │  │
│  │  │  Broadcaster    │  │                  │  │  │
│  │  └─────────────────┘  └──────────────────┘  │  │
│  └──────────────────────────────────────────────┘  │
│                 ↓ (command/state interfaces)       │
│  ┌──────────────────────────────────────────────┐  │
│  │           Hardware Interface                 │  │
│  │  - Read joint states (position, vel, effort) │  │
│  │  - Write joint commands                      │  │
│  │  - Abstract hardware specifics               │  │
│  └────────────────┬─────────────────────────────┘  │
└───────────────────┼─────────────────────────────────┘
                    │ (hardware-specific protocol)
                    ↓
┌─────────────────────────────────────────────────────┐
│              Physical Hardware                      │
│  [Motor Drivers] [Encoders] [Force Sensors]        │
└─────────────────────────────────────────────────────┘

Data Flow:
1. Planner sends trajectory → Controller Manager
2. Controller Manager activates Joint Trajectory Controller
3. Controller computes control commands
4. Hardware Interface writes commands to motors
5. Hardware Interface reads joint states from encoders
6. Joint State Broadcaster publishes to /joint_states
7. robot_state_publisher uses /joint_states to update tf2
```

### Diagram 5: AI-to-Control Architecture

```
Hierarchical AI-to-Control Architecture:

┌─────────────────────────────────────────────────────┐
│ Level 1: AI Agent (High-Level Reasoning)            │
│  - Large Language Model                             │
│  - Vision Transformers                              │
│  - Task Planning                                    │
│  Frequency: 1-10 Hz | Hardware: GPU                 │
└────────────────┬────────────────────────────────────┘
                 │ (goals: "grasp cup", "walk to kitchen")
                 ↓
┌─────────────────────────────────────────────────────┐
│ Level 2: Motion Planning (Mid-Level)                │
│  - Path Planning (RRT, A*)                          │
│  - Inverse Kinematics                               │
│  - Collision Avoidance                              │
│  Frequency: 10-50 Hz | Hardware: CPU                │
└────────────────┬────────────────────────────────────┘
                 │ (trajectories: joint waypoints + timing)
                 ↓
┌─────────────────────────────────────────────────────┐
│ Level 3: Controllers (Low-Level)                    │
│  - PID Controllers                                  │
│  - Balance Controller                               │
│  - Trajectory Execution                             │
│  Frequency: 100-1000 Hz | Hardware: RT CPU/MCU      │
└────────────────┬────────────────────────────────────┘
                 │ (motor commands: torques, voltages)
                 ↓
┌─────────────────────────────────────────────────────┐
│ Level 4: Hardware                                   │
│  - Motor Drivers                                    │
│  - Actuators                                        │
│  - Sensors                                          │
│  Frequency: 1-10 kHz | Hardware: Motor Electronics  │
└─────────────────────────────────────────────────────┘

Feedback Loop:
[Sensors] → [State Estimation] → [Perception] → [AI Agent]
    ↑                                                 ↓
[Hardware] ← [Controllers] ← [Motion Planning] ← [Goals]

Example Data Flow (LLM-Controlled Humanoid):
User: "Pick up the red cup"
    ↓
[LLM]: Understand intent, detect object, plan task
    ↓ (action goal: GraspObject)
[Task Planner]: Decompose into motion primitives
    ↓ (action: MoveToPreGrasp)
[Motion Planner]: Compute collision-free trajectory
    ↓ (FollowJointTrajectory action)
[Controller]: Execute trajectory with feedback
    ↓ (joint torque commands)
[Hardware]: Move arm to grasp pose
```

## Key Concepts Summary

### URDF (Unified Robot Description Format)
XML format describing robot structure: links (rigid bodies with geometry and inertia) and joints (connections with type, axis, limits).

### Kinematic Chain
Series of rigid links connected by joints. Humanoids are kinematic trees with torso as root.

### Forward Kinematics
Computing end effector pose from joint angles. Straightforward matrix multiplication through chain.

### Inverse Kinematics
Computing joint angles needed to achieve target end effector pose. More challenging, may have multiple or no solutions.

### tf2
ROS 2 transform library managing coordinate frames and transformations. Maintains transform tree, provides lookup and broadcasting.

### robot_state_publisher
Node that computes and publishes transforms for all robot links based on URDF and joint states.

### ros2_control
Framework for robot control providing hardware abstraction, controller management, and real-time support.

### Hardware Interface
Layer in ros2_control that communicates with physical hardware, reading states and writing commands.

### Controller
Component implementing control algorithm. Reads states, computes commands, writes to hardware interface.

### RViz2
3D visualization tool for ROS 2. Displays robot models, sensor data, coordinate frames, and debug information.

## Knowledge Checkpoint

Test your understanding of this chapter's concepts:

1. **URDF Understanding:**
   - Explain the difference between visual geometry and collision geometry in URDF. Why are they separated?
   - A humanoid arm has shoulder (3 DOF), elbow (1 DOF), and wrist (3 DOF). How many joints would you define in URDF?
   - Why are accurate inertial properties critical for physics simulation?

2. **Kinematics:**
   - What is the difference between forward kinematics and inverse kinematics?
   - A humanoid with 7-DOF arms (redundant) reaches for an object. Why might multiple joint configurations achieve the same hand pose?
   - Explain why the Jacobian matrix is important for robot control.

3. **Transform Management:**
   - Draw the transform tree for a humanoid robot including map, odom, base, and sensor frames.
   - Why is tf2 necessary? Why not just compute transforms directly in each node?
   - Explain the difference between static and dynamic transforms. Give examples of each.

4. **robot_state_publisher:**
   - What are the inputs and outputs of robot_state_publisher?
   - Why is robot_state_publisher essential for visualization and motion planning?
   - When would you use joint_state_publisher versus getting joint states from hardware?

5. **ros2_control:**
   - Explain the three layers of ros2_control architecture: hardware interface, controller manager, controllers.
   - Why is exclusive controller access important (preventing multiple controllers from writing to same joint)?
   - What is the difference between position control, velocity control, and effort (torque) control?

6. **AI Integration:**
   - Design a four-level hierarchy for an LLM-controlled humanoid: AI agent, motion planning, control, hardware. Specify frequency and data flow for each level.
   - Why can't an AI agent directly command motor torques at 1 Hz for a balancing humanoid?
   - Propose an architecture for integrating a learned RL policy (trained at 50 Hz) with ros2_control.

7. **Visualization:**
   - What is the difference between RViz2 and Gazebo?
   - List five types of data you would visualize in RViz2 when debugging a humanoid navigation system.
   - How do visualization markers help debug motion planning?

## Chapter Summary

This chapter explored ROS 2 tools and patterns specific to humanoid robotics:

**URDF:** The Unified Robot Description Format represents robot structure through links (rigid bodies with geometry and inertia) and joints (connections with type, axis, limits). Xacro adds programmability to reduce repetition. Accurate URDF is essential for simulation, planning, and visualization.

**Kinematic Chains:** Humanoids are kinematic trees with serial chains for limbs. Forward kinematics computes end effector poses from joint angles; inverse kinematics (more challenging) computes joint angles for target poses. The Jacobian relates joint velocities to end effector velocities.

**tf2 Transform System:** Manages coordinate frames and transformations in tree structure. Static transforms (sensor mounts) publish once; dynamic transforms (joint angles, robot pose) publish continuously. Transform lookup finds paths through tree and computes combined transformations.

**robot_state_publisher:** Central node that computes and publishes transforms for all robot links based on URDF and current joint states. Essential for visualization, planning, and control.

**ros2_control:** Framework for robot control with hardware abstraction, controller management, and real-time support. Three layers: hardware interface (talks to motors/sensors), controller manager (orchestrates controllers), controllers (implement control algorithms).

**AI-to-Control Integration:** Hierarchical architecture bridges high-level AI (goals at 1-10 Hz) to low-level control (motor commands at 100-1000 Hz). Motion planning layer translates goals into trajectories; controllers execute trajectories with feedback.

**RViz2 Visualization:** 3D tool for visualizing robot models, sensor data, coordinate frames, and planning outputs. Essential for development and debugging. Complements physics simulators (which simulate dynamics, not just visualize).

These humanoid-specific tools build on ROS 2 foundations to enable sophisticated robot systems.

## Further Reading

**Official Documentation:**
- URDF Tutorials (wiki.ros.org/urdf)
- tf2 Documentation (docs.ros.org)
- ros2_control Documentation
- RViz2 User Guide

**Books:**
- "Modern Robotics" by Lynch and Park (kinematics and dynamics)
- "Robotics: Modelling, Planning and Control" by Siciliano et al.
- "Springer Handbook of Robotics" (comprehensive reference)

**Papers:**
- "URDF: The Unified Robot Description Format" (Willow Garage)
- "ros_control: A Generic and Simple Control Framework for ROS"
- "tf: The Transform Library" (Foote)

**Online Resources:**
- MoveIt 2 Tutorials (motion planning with ros2_control)
- Gazebo tutorials with ros2_control
- ROS 2 Control Demos (ros-controls/ros2_control_demos)

**Tools:**
- check_urdf: Validate URDF syntax
- urdf_to_graphiz: Visualize kinematic tree
- robot_state_publisher documentation

## Looking Ahead

With understanding of humanoid-specific ROS 2 tools, we now turn to physics simulation. The next chapter explores Gazebo and Isaac Sim—physics engines that simulate robot dynamics, sensors, and environments. Simulation enables developing and testing humanoid behaviors safely before deployment to hardware, and provides environments for training learned policies. The URDF, tf2, and ros2_control concepts from this chapter integrate directly into these simulation platforms.
