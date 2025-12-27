# Chapter 3: Introduction to ROS 2

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand the architecture and design principles of ROS 2
- Explain the differences between ROS 1 and ROS 2 and why the migration matters
- Describe the role of DDS middleware in ROS 2 communication
- Analyze the computational graph and relationships between nodes, topics, services, and actions
- Configure Quality of Service (QoS) policies for reliable communication
- Recognize why ROS 2 is particularly suited for humanoid robotics applications

## Introduction

Building a humanoid robot requires coordinating dozens of components: sensors generating data streams, planning algorithms deciding actions, motor controllers executing movements, and safety systems monitoring everything. Each component operates at different frequencies, has different reliability requirements, and may run on different processors or even different machines.

The Robot Operating System 2 (ROS 2) provides the infrastructure to orchestrate this complexity. ROS 2 is not an operating system in the traditional sense—it is a middleware framework and ecosystem of tools that enables building complex robotic systems from modular, reusable components.

This chapter introduces ROS 2's architecture, communication patterns, and design philosophy. Understanding these foundations is essential for building the humanoid robot systems we will develop throughout this course.

## The Role of Middleware in Robotics

### What is Middleware?

Middleware sits between the operating system and application software, providing common services that applications need but operating systems do not provide. In robotics, middleware addresses challenges like:

**Distributed Communication:** Components may run on different processors (sensor processing on GPU, planning on CPU, motor control on microcontrollers). Middleware handles inter-process and inter-machine communication transparently.

**Data Flow Management:** Robot systems process multiple data streams—camera images at 30 Hz, LiDAR scans at 10 Hz, IMU data at 200 Hz. Middleware routes data from producers to consumers efficiently.

**Time Synchronization:** Actions must be coordinated in time. A robot arm motion command must align with gripper closure. Middleware provides timing mechanisms.

**Discovery and Configuration:** Components need to find each other dynamically. When you add a new sensor, other components should discover it automatically without manual wiring.

**Abstraction and Portability:** Middleware hides hardware differences. Your planning code should work whether the robot uses LiDAR or radar, servo motors or hydraulics.

### Why Robotics Needs Specialized Middleware

General-purpose middleware exists, but robotics has unique requirements:

**Real-Time Constraints:** Motor control loops must run at precise intervals (1-10 kHz). Missed deadlines can cause instability or damage.

**Massive Data Throughput:** A single high-resolution camera generates 200+ MB/sec. Point clouds from LiDAR can exceed 1 GB/sec. Middleware must handle this without copying data unnecessarily.

**Dynamic Topology:** Sensors may come online or fail. Algorithms may start and stop. The system must adapt without restarting.

**Heterogeneous Platforms:** Modern robots combine embedded microcontrollers, ARM processors, x86 CPUs, and GPUs. Middleware must work across this heterogeneity.

ROS 2 was designed specifically to address these robotics-specific challenges.

## From ROS 1 to ROS 2: Evolution of Robot Middleware

### ROS 1: The Original Robot Operating System

ROS 1, introduced in 2007, revolutionized robotics development by providing:

- A publish-subscribe communication model (topics)
- Remote procedure calls (services)
- A package management system
- Standard message definitions
- Visualization and debugging tools

ROS 1 enabled rapid development of complex robot systems and became the de facto standard in research and increasingly in industry.

However, ROS 1 had fundamental limitations:

**Single Master Architecture:** ROS 1 requires a central "master" node that coordinates all communication. If the master fails, the entire system fails. This single point of failure is unacceptable for production robots.

**No Real-Time Support:** ROS 1's communication layer cannot guarantee message delivery times, preventing its use in hard real-time control loops.

**No Security:** ROS 1 has no built-in authentication or encryption. Any process can read any topic or call any service.

**Limited Multi-Robot Support:** The single-master design makes coordinating multiple robots cumbersome.

**TCP-based Communication:** Default TCP transport introduces latency and overhead unsuitable for high-frequency control.

### ROS 2: A Complete Redesign

ROS 2, initially released in 2017, addresses ROS 1's limitations through fundamental architectural changes:

**No Single Master:** ROS 2 uses peer-to-peer discovery. Nodes find each other through DDS (Data Distribution Service), eliminating the single point of failure.

**Real-Time Capable:** ROS 2 can run on real-time operating systems and provides deterministic communication paths suitable for control loops.

**Security by Design:** Built-in support for authentication, access control, and encryption.

**DDS Middleware:** Leverages the OMG Data Distribution Service standard, providing mature, tested communication infrastructure.

**Quality of Service:** Configurable reliability, durability, and deadline policies allow tuning communication for different requirements.

**Multi-Language Support:** Improved support for Python, C++, and other languages through unified client libraries.

**Platform Support:** Runs on Linux, Windows, macOS, and real-time operating systems.

### Why the Migration to ROS 2 Matters

While ROS 1 remains widely used in research, ROS 2 is essential for production robotics:

**Safety-Critical Systems:** Humanoid robots working near humans need real-time guarantees and fault tolerance that only ROS 2 provides.

**Multi-Robot Systems:** Deploying fleets of humanoid robots requires robust multi-robot communication.

**Long-Term Support:** ROS 1 reached end-of-life in 2025. ROS 2 is the supported path forward.

**Industry Adoption:** Commercial robotics increasingly demands ROS 2's security, reliability, and real-time features.

For humanoid robotics—where safety, reliability, and real-time control are paramount—ROS 2 is not optional but essential.

## DDS: The Communication Foundation

### Data Distribution Service (DDS)

DDS is an Object Management Group (OMG) standard for real-time, peer-to-peer data distribution. ROS 2 uses DDS as its communication layer, gaining:

**Maturity:** DDS has been used in mission-critical systems (aerospace, defense, industrial control) for over a decade.

**Performance:** Optimized for low latency and high throughput.

**Scalability:** Supports hundreds of nodes and thousands of data streams.

**Reliability Options:** Configurable delivery guarantees from best-effort to guaranteed delivery.

**Discovery:** Automatic peer discovery without central coordination.

### How DDS Works

DDS uses a publish-subscribe model with additional features:

**Global Data Space:** All data exists in a conceptual "global data space" that publishers write to and subscribers read from. This abstraction hides network topology.

**Topics:** Named data channels with defined types. Publishers and subscribers refer to topics by name.

**Quality of Service (QoS):** Configurable policies control reliability, delivery ordering, data lifetime, and resource limits.

**Discovery Protocol:** Nodes announce their presence and discover peers through multicast or predefined discovery servers.

**Data-Centric:** DDS focuses on data flow rather than message passing. Subscribers get updates when data changes, not individual messages (though message-like semantics are also supported).

### DDS Implementations

ROS 2 supports multiple DDS implementations through an abstraction layer (RMW - ROS Middleware):

**Fast DDS (eProsima):** Default in most ROS 2 distributions. Good all-around performance.

**Cyclone DDS (Eclipse):** Lightweight and fast, excellent for embedded systems.

**Connext DDS (RTI):** Commercial implementation with advanced features and support.

**Gurum DDS:** Optimized for resource-constrained embedded systems.

The RMW layer allows switching DDS implementations without changing application code, though performance characteristics may differ.

### Why DDS for ROS 2?

Choosing DDS provided ROS 2 with:

**Proven Technology:** Years of deployment in critical systems.

**Standards Compliance:** Interoperability with other DDS systems.

**Feature Richness:** Real-time, security, QoS built-in rather than added on.

**Vendor Options:** Multiple implementations allow choosing based on requirements.

**Future-Proofing:** As DDS evolves, ROS 2 benefits from improvements.

## The ROS 2 Computational Graph

### Nodes: The Basic Computation Unit

A node is a process that performs computation. Each node is a separate executable with a single purpose:

**Single Responsibility:** A camera driver node reads camera data. A planning node generates trajectories. A visualization node displays data. Each does one thing well.

**Modularity:** Nodes are independent and replaceable. Swap a LiDAR driver for a radar driver without changing other components.

**Scalability:** Distribute nodes across processors and machines as needed.

**Fault Isolation:** If one node crashes, others continue operating.

**Example Nodes in a Humanoid Robot:**
- camera_driver: Captures and publishes images
- depth_processing: Converts depth images to point clouds
- object_detector: Identifies objects in images
- path_planner: Computes navigation paths
- joint_controller: Commands robot joints
- safety_monitor: Watches for dangerous conditions

### Topics: Asynchronous Data Streams

Topics implement publish-subscribe communication for streaming data:

**Publishers:** Nodes that produce data. A camera node publishes images.

**Subscribers:** Nodes that consume data. A visualization node subscribes to images.

**Topic Name:** A unique string identifier like "/camera/color/image_raw".

**Message Type:** The data structure sent on the topic (e.g., sensor_msgs/Image).

**Many-to-Many:** Multiple publishers and subscribers can connect to the same topic.

**Decoupling:** Publishers don't know who (if anyone) subscribes. Subscribers don't know who publishes.

**Example Topics:**
- /camera/image: RGB images from camera
- /imu/data: Inertial measurement unit readings
- /joint_states: Current positions of all robot joints
- /cmd_vel: Velocity commands for robot base
- /point_cloud: 3D point cloud from depth sensor

**Topic Communication Flow:**
```
[Camera Node] --publishes--> [/camera/image] <--subscribes-- [Object Detector]
                                  ↑
                                  |
                             <--subscribes-- [Visualization]
```

Multiple subscribers receive the same data independently.

### Services: Synchronous Request-Response

Services implement request-response communication for occasional, transactional interactions:

**Client:** Sends a request and waits for a response.

**Server:** Receives requests, processes them, and sends responses.

**Service Name:** Unique identifier like "/add_two_ints".

**Service Type:** Defines request and response message structures.

**Blocking:** Client blocks until server responds (or timeout).

**One-to-One:** Each request gets exactly one response.

**Example Services:**
- /set_camera_exposure: Configure camera settings
- /plan_path: Request path from A to B
- /reset_odometry: Reset robot position estimate
- /spawn_entity: Add object to simulation
- /get_map: Retrieve current map

**Service Communication Flow:**
```
[Client Node] --request--> [Service Server] --response--> [Client Node]
                                |
                          (processes request)
```

Services are appropriate when you need confirmation that an action completed or need computed results.

### Actions: Long-Running Goals with Feedback

Actions extend services for long-running tasks that need feedback and cancellation:

**Action Client:** Sends a goal and receives feedback and final result.

**Action Server:** Accepts goals, provides periodic feedback, and returns final results.

**Feedback:** Periodic updates on progress (e.g., "50% complete").

**Cancellation:** Client can cancel a goal in progress.

**Asynchronous:** Client doesn't block while action executes.

**Example Actions:**
- /navigate_to_pose: Navigate robot to target pose
- /grasp_object: Reach and grasp an object
- /follow_trajectory: Execute a joint trajectory
- /dock: Autonomously dock to charging station

**Action Communication Flow:**
```
[Client] --goal--> [Action Server] --feedback--> [Client]
                         |                           |
                    (executing)                      |
                         |                           |
                         ----result-----------------→
```

Actions are ideal for tasks like "navigate to kitchen" which take time and where you want progress updates and ability to cancel.

### Parameters: Runtime Configuration

Parameters allow configuring nodes without recompiling:

**Declaration:** Nodes declare parameters with names and types.

**Setting:** Parameters can be set from command line, launch files, or YAML files.

**Runtime Changes:** Parameters can be modified while node is running (if declared as dynamic).

**Namespacing:** Parameters exist in node namespaces, avoiding conflicts.

**Example Parameters:**
- camera_driver/frame_rate: Camera capture rate
- planner/max_velocity: Maximum planning velocity
- controller/kp_gain: Proportional control gain
- detector/confidence_threshold: Detection confidence threshold

Parameters enable reusing nodes in different contexts. The same camera driver works for different cameras by changing parameters.

### The Complete Computational Graph

A ROS 2 system consists of nodes connected through topics, services, and actions:

```
Computational Graph Example:

[Camera Node] --/image--> [Detector Node] --/detections--> [Planner Node]
      |                                                           |
      |                                                           |
      +--/camera_info--> [Calibration Node]                      |
                                                                  |
[IMU Node] --/imu_data--> [Localization Node] --/pose---------→ +
                                 ↑                                |
                                 |                                |
[Wheel Encoder] --/odom---------+                                |
                                                                  |
[Controller Node] <--/cmd_vel-----------------------------------+
      |
      +--/joint_commands--> [Motor Drivers]
```

This graph shows data flowing from sensors (Camera, IMU, Encoders) through processing (Detector, Localization) to planning (Planner) and control (Controller), ultimately commanding motors.

## Quality of Service (QoS)

### Why QoS Matters

Not all communication has the same requirements:

**Sensor Data:** Streams of data where latest value matters most. Old data is useless. Loss of occasional messages is acceptable.

**Commands:** Critical messages that must arrive. A "stop" command cannot be lost.

**State:** Information where subscribers need the current value, even if they join late.

**High-Frequency Control:** Data that must arrive within microseconds.

ROS 2's QoS policies allow configuring communication to meet these diverse requirements.

### QoS Policy Dimensions

**Reliability:**
- Best Effort: Send data but don't retry if lost. Lowest latency, highest throughput. Good for sensor streams where latest data matters.
- Reliable: Guarantee delivery through retransmission. Higher latency, lower throughput. Good for commands and critical data.

**Durability:**
- Volatile: New subscribers receive only future messages. Good for streaming data.
- Transient Local: New subscribers receive last N messages (based on history). Good for state information where late-joiners need current value.

**History:**
- Keep Last N: Store only the N most recent messages. Prevents unbounded memory growth.
- Keep All: Store all messages until delivered. Useful for ensuring no data loss but risks memory exhaustion.

**Deadline:**
- Maximum time between messages. If exceeded, triggers callback. Useful for detecting sensor failures.

**Lifespan:**
- Maximum age for messages. Old messages are discarded. Prevents acting on stale data.

**Liveliness:**
- Mechanism to detect if publishers are still alive. Enables fault detection.

### Common QoS Profiles

ROS 2 provides preset profiles for common scenarios:

**Sensor Data Profile:**
- Reliability: Best Effort
- Durability: Volatile
- History: Keep Last 10
- Use: High-frequency sensor streams (cameras, LiDAR, IMU)

**System Default:**
- Reliability: Reliable
- Durability: Volatile
- History: Keep Last 10
- Use: General-purpose communication

**Services:**
- Reliability: Reliable
- Durability: Volatile
- History: Keep Last 10
- Use: Request-response patterns

**Parameters:**
- Reliability: Reliable
- Durability: Transient Local
- History: Keep Last 1
- Use: Configuration values that should persist

**Custom Profiles:**
You can define custom profiles mixing policies for specific needs.

### QoS Example: Camera vs. Commands

**Camera Image Topic:**
```
Reliability: Best Effort (occasional loss acceptable)
Durability: Volatile (old images irrelevant)
History: Keep Last 1 (only latest image needed)
Deadline: 50ms (30 Hz camera)
```

This configuration minimizes latency and memory while ensuring fresh images.

**Emergency Stop Topic:**
```
Reliability: Reliable (cannot lose stop command)
Durability: Transient Local (late subscribers get stop state)
History: Keep Last 10 (ensure delivery despite network issues)
Liveliness: Automatic (detect if stop publisher dies)
```

This configuration prioritizes safety and guarantees over performance.

### QoS Compatibility

Publishers and subscribers must have compatible QoS settings to connect:

**Reliability:** Reliable subscriber can connect to Reliable publisher. Best Effort subscriber can connect to any publisher. Reliable subscriber cannot connect to Best Effort publisher.

**Durability:** Transient Local subscriber can connect to Transient Local publisher. Volatile subscriber can connect to any publisher.

Mismatched QoS prevents connection, a common source of confusion for new ROS 2 users.

## ROS 2 for Humanoid Robotics

### Why ROS 2 Excels for Humanoid Systems

Humanoid robots present unique challenges that ROS 2 addresses:

**Complex Sensor Integration:** Humanoids use cameras, depth sensors, LiDAR, IMUs, force sensors, tactile arrays. ROS 2's flexible topic system and QoS policies handle this heterogeneous sensor suite.

**Real-Time Control:** Bipedal balance requires motor control loops at 1-10 kHz with deterministic timing. ROS 2's real-time support enables this.

**Distributed Computation:** High-level planning (CPU), perception (GPU), motor control (microcontroller) run on different processors. ROS 2's DDS-based communication handles this seamlessly.

**Safety Requirements:** Humanoids near humans need redundant safety monitoring, emergency stops, and fault detection. ROS 2's reliable QoS and multiple communication patterns support safety architectures.

**Multi-Modal Interaction:** Humanoids combine navigation, manipulation, speech, gesture. ROS 2's modular architecture allows integrating diverse behaviors.

### The Humanoid Robot ROS 2 Ecosystem

ROS 2 provides extensive libraries for humanoid robotics:

**MoveIt 2:** Motion planning for manipulation. Computes collision-free paths for arm movements.

**Navigation2 (Nav2):** Autonomous navigation with obstacle avoidance, perfect for bipedal locomotion.

**ros2_control:** Hardware abstraction for motor controllers, joint state management, and controller switching.

**tf2:** Transform library for managing coordinate frames (critical for multi-link humanoids).

**RViz2:** 3D visualization for debugging and monitoring.

**robot_state_publisher:** Publishes robot geometry for visualization and planning.

**Gazebo/Isaac Sim Integration:** Physics simulation for development and testing.

**Real-Time Tools:** Priority scheduling, memory locking, and deterministic execution for control loops.

### Common Humanoid Robot Architectures

Typical ROS 2 architecture for a humanoid:

**Perception Layer:**
- Camera drivers (vision, depth)
- LiDAR driver
- IMU driver
- Force/torque sensor drivers
- Sensor fusion nodes (VIO, point cloud processing)

**Localization and Mapping:**
- Visual-Inertial Odometry
- SLAM (Simultaneous Localization and Mapping)
- Map server

**Planning Layer:**
- Global path planner
- Local planner with obstacle avoidance
- Manipulation planner
- Whole-body motion planner
- Task planner

**Control Layer:**
- Joint position controllers
- Balance controller
- Compliance controller
- Gripper controller
- Whole-body controller

**Safety and Monitoring:**
- Joint limit monitor
- Collision detection
- Emergency stop handler
- Health monitoring

**High-Level Behavior:**
- State machine for task execution
- Speech recognition and synthesis
- Human-robot interaction manager
- AI/LLM integration for decision making

Each layer uses appropriate ROS 2 primitives (topics for sensor streams, actions for long-running tasks, services for configuration).

## Conceptual Diagrams

### Diagram 1: ROS 1 vs. ROS 2 Architecture

```
ROS 1 Architecture:

                [ROS Master]
                     |
        +------------+------------+
        |            |            |
    [Node A] <---> [Node B] <---> [Node C]

    - Single point of failure (Master)
    - Master coordinates all communication
    - XML-RPC for discovery, TCP for data


ROS 2 Architecture:

    [Node A] <--DDS--> [Node B] <--DDS--> [Node C]
         ↕                ↕                 ↕
         +----------- DDS Bus --------------+

    - Peer-to-peer discovery
    - No single point of failure
    - DDS handles all communication
    - Nodes discover each other automatically
```

### Diagram 2: Communication Patterns

```
Topics (Publish-Subscribe):

[Publisher Node] --publishes--> [Topic: /sensor_data]
                                       ↓
                                   (data flow)
                                       ↓
                     +-----------------+-----------------+
                     ↓                 ↓                 ↓
              [Subscriber 1]    [Subscriber 2]    [Subscriber 3]

              - Asynchronous
              - One-to-many
              - Decoupled


Services (Request-Response):

[Client Node] --request--> [Service: /compute] --response--> [Client Node]
                                  |
                            (synchronous)

              - Synchronous
              - One-to-one
              - Blocking


Actions (Goal-Feedback-Result):

[Client] --goal--> [Action Server: /navigate]
                         |
                         +--feedback--> [Client] (periodic)
                         |
                         +--result----> [Client] (when done)

              - Asynchronous
              - Long-running
              - Cancellable
              - Progress updates
```

### Diagram 3: QoS Policy Impact

```
Best Effort vs. Reliable:

Best Effort:
[Publisher] ====X====> [Subscriber]
    t1: ✓               ✓ received
    t2: ✓               ✗ lost (no retry)
    t3: ✓               ✓ received

    - Lower latency
    - Higher throughput
    - Occasional loss


Reliable:
[Publisher] =========> [Subscriber]
    t1: ✓               ✓ received
    t2: ✓ retry→✓       ✓ received (after retry)
    t3: ✓               ✓ received

    - Higher latency
    - Guaranteed delivery
    - No loss


Durability Example:

Volatile:
[Publisher starts] → sends data → [Subscriber joins] → receives only future data

    Time:      t1        t2        t3        t4
    Pub:       D1        D2        --        D3
    Sub:       --        --      [join]      D3 ✓

    Late subscriber misses D1, D2


Transient Local:
[Publisher starts] → sends data → [Subscriber joins] → receives last N messages

    Time:      t1        t2        t3        t4
    Pub:       D1        D2        --        D3
    Sub:       --        --      [join]    D2✓,D3✓

    Late subscriber gets cached D2, then D3
```

### Diagram 4: Humanoid Robot ROS 2 Graph

```
Humanoid Robot Computational Graph:

Sensors:
[Camera] --/image_raw--> [Image Processing]
[LiDAR] --/scan--------> [Point Cloud Processing]
[IMU] --/imu_data------> [State Estimation]
[Force Sensors] --/ft--> [Contact Detection]

Perception & Localization:
[Image Processing] --/detections--> [Scene Understanding]
[Point Cloud] --/obstacles-------> [Obstacle Avoidance]
[State Estimation] --/odom-------> [Localization]

Planning:
[Scene Understanding] --------+
[Obstacle Avoidance] ---------+---> [Global Planner]
[Localization] ---------------+            |
                                           |
                                   [Local Planner]
                                           |
                                   [Motion Planner]

Control:
[Motion Planner] --/joint_trajectory--> [Joint Controller]
                                              |
                                    [Balance Controller]
                                              |
                                   [Hardware Interface]
                                              |
                                         [Motors]

Safety:
[Contact Detection] ----+
[Joint States] ---------+--> [Safety Monitor] --/emergency_stop--> [All Controllers]
[Localization] ---------+
```

## Key Concepts Summary

### DDS (Data Distribution Service)
Industry-standard middleware providing real-time, peer-to-peer communication. The foundation of ROS 2's communication layer.

### Computational Graph
The network of nodes connected through topics, services, and actions. Represents the complete system architecture.

### Node
An independent process performing a specific computation. The basic building block of ROS 2 systems.

### Topic
A named data stream using publish-subscribe pattern. Used for continuous data flow like sensor readings.

### Service
Request-response communication pattern for occasional, transactional interactions requiring confirmation.

### Action
Extended service pattern for long-running tasks with feedback and cancellation support.

### Quality of Service (QoS)
Configurable policies controlling reliability, durability, history, and timing of communication.

### RMW (ROS Middleware)
Abstraction layer allowing ROS 2 to work with different DDS implementations.

## Knowledge Checkpoint

Test your understanding of this chapter's concepts:

1. **Architecture Understanding:**
   - Explain why ROS 2's peer-to-peer architecture is more robust than ROS 1's master-based design.
   - What role does DDS play in ROS 2, and why was it chosen over building custom middleware?
   - Describe the relationship between nodes, topics, and the computational graph.

2. **Communication Patterns:**
   - When would you use a topic versus a service versus an action? Provide specific humanoid robot examples for each.
   - A navigation system needs to send velocity commands to motors while receiving position feedback. Which communication pattern would you use and why?
   - Why are topics asynchronous while services are synchronous? What are the implications for system design?

3. **Quality of Service:**
   - You're designing a humanoid robot that must receive emergency stop commands reliably, even if components restart. What QoS policies would you configure?
   - Explain why camera image topics typically use "Best Effort" reliability while command topics use "Reliable".
   - What happens when a subscriber with "Reliable" QoS tries to connect to a publisher with "Best Effort" QoS?

4. **Application Design:**
   - Design the computational graph for a humanoid robot that must navigate to a location while avoiding obstacles. Identify nodes, topics, services, and actions.
   - For each sensor in a humanoid robot (camera, LiDAR, IMU, force sensors), specify appropriate QoS settings and justify your choices.
   - A humanoid must grasp an object. The task takes several seconds and you want progress updates. Which communication pattern would you use and why?

5. **Critical Thinking:**
   - Why is ROS 2's real-time support essential for humanoid bipedal balance control?
   - Compare the trade-offs between running all processing on a single powerful computer versus distributing across multiple processors.
   - How does ROS 2's modularity through nodes facilitate development and testing compared to a monolithic application?

## Chapter Summary

This chapter introduced ROS 2, the middleware framework that enables building complex robotic systems:

**Middleware for Robotics:** ROS 2 provides distributed communication, data flow management, time synchronization, and hardware abstraction. It addresses robotics-specific challenges like real-time constraints, massive data throughput, and heterogeneous platforms.

**Evolution from ROS 1:** ROS 2 eliminates the single-master architecture, adds real-time support, implements security, and leverages DDS for mature, proven communication infrastructure. These improvements make ROS 2 suitable for production humanoid robots.

**DDS Foundation:** Data Distribution Service provides peer-to-peer discovery, high-performance communication, and configurable Quality of Service. Multiple DDS implementations allow optimizing for different deployment scenarios.

**Computational Graph:** ROS 2 systems consist of nodes connected through topics (streaming data), services (request-response), and actions (long-running goals with feedback). This modular architecture enables building complex systems from reusable components.

**Quality of Service:** Configurable policies for reliability, durability, history, and timing allow tuning communication for different requirements. Sensor streams use different QoS than commands, which differ from state information.

**Humanoid Robot Advantages:** ROS 2's real-time support, distributed computation, safety mechanisms, and extensive ecosystem make it ideal for humanoid robotics. The modular architecture naturally matches the layered structure of perception, planning, and control.

Understanding ROS 2's architecture and communication patterns provides the foundation for building the humanoid robot systems we will develop in subsequent chapters.

## Further Reading

**Official Documentation:**
- ROS 2 Official Documentation (docs.ros.org)
- DDS Specification (OMG Data Distribution Service)
- ROS 2 Design Documents (design.ros2.org)

**Books:**
- "A Concise Introduction to Robot Programming with ROS 2" by Francisco Martín Rico
- "ROS 2 Developer's Guide" (forthcoming)
- "Programming Robots with ROS" by Quigley, Gerkey, and Smart (ROS 1 but concepts transfer)

**Papers:**
- "ROS 2: The Robot Operating System Version 2" (Maruyama et al.)
- "Data Distribution Service (DDS) for Real-time Systems" (OMG)
- "Evaluation of the ROS 2 Architecture" (various authors)

**Online Resources:**
- ROS Discourse (community discussion)
- ROS 2 tutorials and demos
- DDS vendor documentation (eProsima, RTI, Eclipse Cyclone)

**Videos:**
- ROSCon presentations on ROS 2 architecture
- DDS technology overviews
- ROS 2 migration guides

## Looking Ahead

With understanding of ROS 2's architecture and communication patterns, we now turn to building systems with ROS 2. The next chapter explores package structure, publishers and subscribers, launch systems, parameters, and multi-node system design. You will learn how to organize code, manage configuration, and orchestrate complex behaviors—skills essential for developing humanoid robot applications.
