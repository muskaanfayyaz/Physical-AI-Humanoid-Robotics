# Chapter 11: Humanoid Robot Kinematics and Dynamics

## Introduction

When a humanoid robot reaches for an object, walks across a room, or maintains balance on uneven terrain, complex mathematical machinery works behind the scenes to translate desired motions into precise motor commands. Understanding how robot joints relate to end-effector positions, how velocities propagate through kinematic chains, and how forces and torques govern motion is fundamental to humanoid robot development.

Kinematics describes the geometry of motion without considering the forces that cause it, while dynamics incorporates mass, inertia, and forces to predict and control actual physical behavior. For humanoid robots with dozens of joints, multiple limbs, and complex interaction with the environment, mastering these concepts becomes critical for achieving coordinated, efficient, and stable motion.

This chapter explores the mathematical foundations that enable humanoid robots to move purposefully through space. We begin with forward kinematics, which computes where the robot's limbs are given joint angles, then tackle the inverse problem of determining what joint angles achieve a desired position. We examine how velocities and forces propagate through kinematic chains, address singularities where motion control breaks down, and explore the dynamic equations that relate torques to motion. By understanding these principles, you will grasp how motion planning algorithms, control systems, and physical design choices interact to create capable humanoid platforms.

## Core Concepts

### The Kinematic Problem Space

Humanoid robots present a unique kinematic challenge: they possess high degrees of freedom, operate in three-dimensional space, must coordinate multiple limbs simultaneously, and interact with their environment through contacts that change over time. Unlike industrial manipulators with fixed bases, humanoids lack a permanently grounded reference frame, complicating the mathematical description of their configuration.

The configuration space, or C-space, represents all possible robot configurations. For a humanoid with n joints, this space has n dimensions, though constraints like joint limits, collision avoidance, and balance requirements restrict the feasible region. Understanding how to navigate this space efficiently while respecting constraints forms the foundation of motion planning and control.

Kinematic chains connect rigid links through joints, creating a tree-like structure in humanoids. The torso typically serves as the root, with legs, arms, and head branching outward. Each chain has its own forward and inverse kinematic solutions, but achieving coordinated whole-body motion requires considering coupling between chains and managing the overall center of mass.

### Reference Frames and Transformations

Every point and orientation in robotics must be expressed relative to some reference frame. Humanoids use multiple coordinate systems: a world frame fixed in space, a base frame attached to the robot's torso or pelvis, joint frames at each articulation, and end-effector frames at the hands, feet, and head. Transforming quantities between these frames requires homogeneous transformation matrices.

A homogeneous transformation matrix combines rotation and translation in a 4x4 format:

```
T = | R  p |
    | 0  1 |
```

where R is a 3x3 rotation matrix and p is a 3x1 position vector. This compact representation allows chaining transformations through matrix multiplication: if T_AB transforms from frame A to B and T_BC transforms from B to C, then T_AC = T_AB * T_BC transforms directly from A to C.

Rotation representations include rotation matrices, Euler angles, axis-angle notation, and quaternions. Each has advantages and drawbacks. Rotation matrices provide direct transformation but use nine parameters for three degrees of freedom. Euler angles are intuitive but suffer from gimbal lock. Quaternions avoid singularities and interpolate smoothly but are less intuitive. Choosing the appropriate representation depends on the application's requirements.

### Denavit-Hartenberg Parameters

The Denavit-Hartenberg (DH) convention provides a systematic method for establishing coordinate frames and deriving forward kinematics. It reduces the transformation between adjacent joints to four parameters: link length (a), link twist (alpha), link offset (d), and joint angle (theta). For revolute joints, theta varies while other parameters remain fixed; for prismatic joints, d varies.

Two DH conventions exist: classic (original) and modified (Craig). The classic convention places frame i at joint i+1, while the modified convention places frame i at joint i. The modified convention often simplifies calculations and matches intuition better, making it popular in modern robotics libraries.

Establishing DH frames follows specific rules: the z-axis aligns with the joint axis, the x-axis points along the common normal between consecutive z-axes, and the y-axis completes the right-handed coordinate system. While this process can seem arbitrary, following the systematic procedure ensures consistency and correct transformation matrices.

### Kinematic Redundancy

A humanoid arm typically has seven or more degrees of freedom to reach and orient its hand in 3D space, which requires only six degrees of freedom (three for position, three for orientation). This excess, called kinematic redundancy, provides flexibility but complicates inverse kinematics since infinitely many joint configurations can achieve the same end-effector pose.

Redundancy offers significant advantages: avoiding joint limits, navigating around obstacles, optimizing secondary criteria like manipulability or energy efficiency, and continuing operation when one joint fails. However, it requires sophisticated algorithms to select among the infinite solutions, typically by optimizing some criterion while achieving the primary task.

The null space of the Jacobian matrix captures directions in joint space that don't affect the end-effector. Motion in this null space allows secondary objectives without disturbing the primary task. For example, an arm can maintain its hand position while adjusting its elbow height to avoid an obstacle or stay far from joint limits.

### Dynamic Foundations

While kinematics describes motion geometry, dynamics incorporates physical laws to relate forces and torques to accelerations. Newton's second law (F = ma) and its rotational equivalent (tau = I * alpha) govern how forces create motion. For humanoid robots with many interconnected bodies, dynamics becomes substantially more complex than for single rigid bodies.

The inertia tensor characterizes how mass distributes within a rigid body, determining its resistance to rotational acceleration. Unlike scalar mass, the inertia tensor is a 3x3 matrix that varies with the chosen reference point and coordinate frame orientation. Computing the overall inertia of a humanoid requires combining individual link inertias through appropriate transformations.

Humanoid dynamics exhibits strong coupling: moving one joint generates forces throughout the robot due to inertial effects. A rapidly accelerating arm creates torques at the shoulder, affects the torso orientation, and can disturb balance. Understanding and accounting for these coupled dynamics enables coordinated motion control and efficient energy usage.

## Practical Understanding

### Forward Kinematics: From Joints to Space

Forward kinematics computes the position and orientation of robot links given joint angles. For a kinematic chain, this involves multiplying transformation matrices from the base to the end-effector. Each matrix represents the transformation due to one joint and link.

Consider a simple two-link planar arm. Link 1 has length L1 and rotates by angle theta1. Link 2 has length L2 and rotates by theta2 relative to link 1. The end-effector position follows directly from trigonometry:

```
x = L1 * cos(theta1) + L2 * cos(theta1 + theta2)
y = L1 * sin(theta1) + L2 * sin(theta1 + theta2)
```

For spatial (3D) robots, the mathematics extends to include three rotational degrees of freedom. Using DH parameters, the transformation matrix from joint i to i+1 takes the standard form involving four elementary rotations and translations. The overall forward kinematics chains these matrices: T_0n = T_01 * T_12 * ... * T_(n-1)n.

In humanoid robots, forward kinematics serves multiple purposes beyond computing hand positions. It determines foot placement during walking, tracks head orientation for vision systems, computes the center of mass location for balance control, and provides collision detection by tracking all link positions.

Efficient implementation matters for real-time control. Rather than recomputing all transformations from scratch at each timestep, incremental updates exploit the fact that only one or a few joints typically change between control cycles. Caching intermediate results and updating only affected branches reduces computational burden significantly.

### Inverse Kinematics: From Desired Pose to Joint Angles

Inverse kinematics (IK) solves the reverse problem: given a desired end-effector position and orientation, find joint angles that achieve it. This problem is generally more difficult than forward kinematics. For many robot configurations, no closed-form solution exists, multiple solutions may satisfy the requirements, or the desired pose may be unreachable.

Analytical IK solutions derive explicit formulas for joint angles using geometry and trigonometry. These solutions are fast, exact, and provide all possible configurations. However, they exist only for specific kinematic structures, particularly robots with six degrees of freedom and certain geometric properties like intersecting joint axes or parallel consecutive axes.

For the two-link planar arm, analytical IK uses the law of cosines and inverse trigonometry. Given desired position (x, y), first compute theta2:

```
cos(theta2) = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2)
theta2 = atan2(±sqrt(1 - cos^2(theta2)), cos(theta2))
```

The ± indicates two solutions: elbow-up and elbow-down configurations. Then compute theta1 using the known theta2. This simple example illustrates solution multiplicity and the need to choose among alternatives.

Numerical IK methods work for any robot structure by iteratively adjusting joint angles to minimize the error between current and desired poses. The Jacobian pseudo-inverse method is popular: at each iteration, compute the Jacobian matrix J relating joint velocities to end-effector velocities, then update joint angles by delta_q = J^+ * delta_x, where J^+ is the pseudo-inverse and delta_x is the pose error.

Numerical methods introduce several considerations. They require an initial guess, may converge to local minima rather than global solutions, can be slow for complex robots, and may produce joint angle trajectories that jump discontinuously between solutions. Damped least squares methods improve numerical stability by adding a damping term that trades exact solution accuracy for smoother behavior near singularities.

Humanoid whole-body IK extends these concepts to handle multiple simultaneous constraints: both feet must maintain contact with the ground, the center of mass must remain over the support polygon, joint limits must be respected, and perhaps the hands must reach target positions simultaneously. This requires solving a constrained optimization problem, often formulated as quadratic programming with equality and inequality constraints.

### Jacobian Matrices and Velocity Kinematics

The Jacobian matrix J relates joint velocities to end-effector velocities: v = J * q_dot, where v is the end-effector velocity (linear and angular), q_dot is the vector of joint velocities, and J depends on the current joint configuration. Each column of J represents how the end-effector moves when one joint moves while others remain fixed.

Computing the Jacobian involves differentiating forward kinematics with respect to joint angles. For revolute joints, the linear velocity contribution is the cross product of the joint axis with the vector from joint to end-effector. The angular velocity contribution is simply the joint axis direction scaled by joint velocity.

The Jacobian structure reveals important properties. Its rank indicates the number of independent directions the end-effector can move instantaneously. For a six-DOF arm, J should have rank 6 when away from singularities. The null space dimension equals the number of degrees of redundancy. The condition number measures how uniform the robot's mobility is across different directions.

In control applications, the Jacobian enables Cartesian velocity control: specify desired end-effector velocities, then compute required joint velocities via q_dot = J^+ * v. The pseudo-inverse J^+ generalizes matrix inversion to rectangular and singular matrices. For redundant robots, it provides the minimum-norm solution, but extended formulas can incorporate secondary objectives through null space projections.

Force relationships follow from the Jacobian transpose: tau = J^T * F, where tau is the vector of joint torques and F is the end-effector force (including torques). This relationship reflects the principle of virtual work: power at joints equals power at the end-effector. The Jacobian transpose maps desired end-effector forces to required joint torques, crucial for force control and gravity compensation.

### Singularities and Their Implications

Singularities occur when the Jacobian matrix loses rank, meaning the robot cannot instantaneously move in some direction regardless of joint velocities. At singularities, the robot's reachable velocity space has reduced dimension, inverse kinematics becomes ill-conditioned, and required joint velocities may become infinite for certain end-effector velocities.

Several singularity types exist. Boundary singularities occur at the workspace edge when the robot is fully extended or retracted. Interior singularities happen within the workspace when certain joint axes align. For example, when the elbow of a three-link planar arm lies on the line connecting the shoulder to the wrist, forward and backward elbow motion produce the same wrist motion.

Detecting singularities involves monitoring the Jacobian's determinant or singular values. The manipulability measure, defined as the square root of det(J * J^T), quantifies how far the configuration is from singular. It reaches zero at singularities and achieves maximum values in well-conditioned configurations. Motion planning algorithms often use manipulability as an optimization criterion to maintain distance from singular configurations.

Handling singularities requires different strategies depending on the application. Task planning can avoid singular configurations entirely by selecting alternative paths or intermediate waypoints. Damped least squares IK adds a damping term that gracefully degrades tracking performance near singularities rather than producing unbounded joint velocities. Algorithmic singularities in specific IK formulations can sometimes be resolved by reformulating the problem or switching between multiple IK solutions.

For humanoid robots, singularities pose particular challenges during whole-body tasks. Leg singularities during walking can cause control instability. Arm singularities limit manipulation capability. However, kinematic redundancy often allows reconfiguring the robot to escape singular configurations while maintaining end-effector constraints.

### Robot Dynamics: Forces and Motion

Dynamic equations describe how joint torques relate to joint accelerations, accounting for inertia, Coriolis effects, centrifugal forces, and gravity. For a humanoid with n joints, the equations of motion take the form:

```
tau = M(q) * q_ddot + C(q, q_dot) * q_dot + G(q)
```

where tau is the vector of joint torques, M(q) is the n-by-n mass matrix (configuration-dependent inertia), C(q, q_dot) captures Coriolis and centrifugal effects, and G(q) represents gravitational torques. This compact form conceals substantial complexity in computing these matrices.

The mass matrix M is symmetric and positive definite, representing the robot's resistance to acceleration. Its diagonal elements indicate how much torque is needed to accelerate each joint when others remain stationary. Off-diagonal elements capture coupling: accelerating joint i requires torque at joint j due to inertial coupling between links.

Coriolis forces arise from the interaction between rotation and linear motion. When a link rotates while mass moves along it, Coriolis forces perpendicular to both the rotation axis and the motion direction result. Centrifugal forces push outward from rotation centers. Together, these velocity-dependent terms can be substantial during rapid motion and must be compensated for accurate trajectory tracking.

Gravitational torques depend on link masses, center-of-mass locations, and current configuration. A horizontal arm requires constant torque to support its weight against gravity, while a vertical arm needs no gravity compensation torque. Computing G(q) involves summing the gravitational forces on all links and mapping them to joint torques through the Jacobian transpose.

### Newton-Euler Formulation

The Newton-Euler recursive formulation computes dynamic equations efficiently by propagating velocities, accelerations, and forces through the kinematic tree. It consists of two passes: a forward pass from base to end-effector propagating velocities and accelerations, and a backward pass from end-effector to base propagating forces and torques.

In the forward pass, each link's linear and angular velocities and accelerations are computed from the previous link's quantities plus the contribution from the connecting joint. For link i connected by a revolute joint with axis z_i and velocity theta_dot_i:

```
omega_i = omega_(i-1) + theta_dot_i * z_i
omega_dot_i = omega_dot_(i-1) + theta_ddot_i * z_i + omega_(i-1) × (theta_dot_i * z_i)
```

The linear acceleration includes both the propagated acceleration and the centrifugal/Coriolis terms from rotation.

The backward pass applies Newton's and Euler's equations to each link to compute the forces and torques it exerts on its predecessor. Starting from the end-effector (with known external forces), forces and torques propagate backward. The joint torque equals the component of the propagated torque along the joint axis.

This recursive formulation has O(n) computational complexity, substantially more efficient than the O(n^4) naive approach of symbolically deriving equations of motion. Modern implementations optimize further through parallel computation and exploiting problem structure.

### Lagrangian Formulation

The Lagrangian approach derives equations of motion from energy principles. Define the Lagrangian L as kinetic energy K minus potential energy P: L = K - P. The Euler-Lagrange equation states:

```
d/dt(dL/dq_dot) - dL/dq = tau
```

Applied to each joint, this yields the full dynamic equations. The mass matrix M emerges from second derivatives of kinetic energy with respect to joint velocities. Coriolis and centrifugal terms come from first derivatives. Gravity terms arise from potential energy derivatives.

Computing kinetic energy requires summing contributions from all links. Each link's kinetic energy includes translational and rotational components:

```
K_i = (1/2) * m_i * v_i^T * v_i + (1/2) * omega_i^T * I_i * omega_i
```

where m_i is link mass, v_i is center-of-mass velocity, omega_i is angular velocity, and I_i is the inertia tensor. Expressing these quantities in terms of joint positions and velocities, then differentiating, yields the equations of motion.

The Lagrangian formulation provides elegant mathematical structure and systematically handles constraints through Lagrange multipliers. It's particularly useful for deriving symbolic equations for small systems or analyzing theoretical properties. However, for numerical computation in real-time control, the Newton-Euler recursion is generally more efficient.

### Torque Requirements and Motor Sizing

Understanding dynamics enables sizing motors appropriately. Required torque at each joint depends on the loads it must accelerate, the robot's weight distribution, and the desired motion profiles. Undersized motors cannot achieve desired performance; oversized motors waste power, money, and payload capacity.

Peak torque requirements occur during maximum acceleration, rapid direction changes, or when supporting heavy loads far from joints. For a humanoid arm lifting an object, the shoulder must provide torque to accelerate the arm's mass, overcome gravity acting on the horizontal moment arm, and accelerate the payload. These components sum directly:

```
tau_total = tau_inertial + tau_gravity + tau_payload
```

Dynamic simulation with representative task scenarios reveals actual torque requirements. Simulate reaching motions at maximum speed, lifting maximum expected payloads, and rapid obstacle avoidance maneuvers. Record peak torques at each joint across all scenarios. Add safety margins (typically 20-50%) to account for modeling uncertainties and unanticipated situations.

Motor selection involves trade-offs between torque, speed, weight, and efficiency. High-torque motors are typically heavy. Gear reduction increases torque at the expense of speed and introduces backlash and friction. Harmonic drives provide high ratios in compact packages but cost significantly more than planetary gears. Brushless DC motors offer good power density and controllability but require more sophisticated drive electronics than brushed motors.

Thermal considerations matter for continuous operation. Motors dissipate power as heat based on current squared times winding resistance. Peak torque ratings apply for short durations; continuous ratings are substantially lower. Humanoid robots performing prolonged tasks must respect continuous limits or provide active cooling.

### Computational Tools and Libraries

Several well-established libraries compute kinematics and dynamics for robot systems. The Kinematics and Dynamics Library (KDL), part of the Orocos project, provides a C++ implementation with Python bindings. It handles kinematic chains and trees, computes forward and inverse kinematics using various solvers, and calculates dynamic quantities through recursive Newton-Euler algorithms.

The Rigid Body Dynamics Library (RBDL) offers efficient dynamics computation optimized for control applications. It implements multiple algorithms for forward dynamics (computing accelerations from torques), inverse dynamics (computing torques from desired accelerations), and their Jacobians. Particular attention to numerical efficiency makes RBDL suitable for model predictive control where dynamics must be evaluated thousands of times per second.

Pinocchio, developed by the Gepetto team, provides state-of-the-art performance for rigid body dynamics. It exploits spatial algebra and Lie group structure to achieve exceptional computational efficiency. Pinocchio supports various kinematic representations, computes analytical derivatives of dynamic quantities (crucial for optimization-based control), and interfaces naturally with modern optimal control frameworks.

These libraries share common abstractions: a robot model defined by links, joints, and their parameters; functions to compute forward kinematics, Jacobians, and dynamics; and utilities for parsing standard formats like URDF (Unified Robot Description Format). They differ in implementation language, computational efficiency, breadth of algorithms, and interface design.

Understanding the concepts behind these libraries enables effective use and debugging. When an IK solver fails to converge, recognizing singular configurations suggests modifying the target pose or initial guess. When simulated torques exceed hardware limits, dynamic analysis reveals whether the problem lies in excessive acceleration, insufficient gear reduction, or supporting weight far from joints.

### Whole-Body Dynamics and Contact Constraints

Humanoid robots interact with their environment through contacts: feet on the ground, hands grasping objects, or potentially the torso against a wall. These contacts create kinematic constraints (the foot cannot penetrate the floor) and force constraints (the contact can only push, not pull). Incorporating contact constraints into dynamics requires careful mathematical treatment.

Contact models range from simple to sophisticated. Hard contacts treat the constraint as perfectly rigid: contact points have zero acceleration perpendicular to the contact surface. Soft contacts model compliance with spring-damper systems, allowing some penetration. Hybrid models switch between contact and non-contact based on computed forces and positions.

The equations of motion with contacts include constraint forces. In Lagrangian mechanics, these appear as:

```
M(q) * q_ddot + C(q, q_dot) * q_dot + G(q) = tau + J_c^T * lambda
```

where J_c is the constraint Jacobian and lambda represents contact forces. The constraint equation J_c * q_ddot = 0 (or equals acceleration of the contact surface for non-stationary contacts) must be satisfied. Together, these form a differential-algebraic equation system.

Solving constrained dynamics involves computing both the motion q_ddot and the constraint forces lambda simultaneously. For hard contacts, substituting the constraint equation into the equations of motion yields a reduced system. For soft contacts, the contact forces are explicit functions of penetration depth and velocity, avoiding the algebraic constraints.

Walking robots alternate between different contact configurations: single support when one foot is on the ground, double support when both feet contact, and flight phase when neither foot touches (if running or jumping). Each configuration has different constraint Jacobians and requires different dynamic equations. Transitioning between configurations creates discrete events that hybrid system models capture.

### Computational Efficiency and Real-Time Performance

Real-time control of humanoid robots demands computing kinematics and dynamics within tight time constraints, typically 1 millisecond or less per control cycle. A 30-joint humanoid poses substantial computational challenges. The full mass matrix contains 900 elements, and naively computing them requires thousands of arithmetic operations.

Exploiting problem structure dramatically improves efficiency. The mass matrix is sparse for tree-structured robots; most links don't directly couple. Recursive algorithms compute only necessary elements. When joint angles change slightly between timesteps, perturbation methods update quantities incrementally rather than recomputing from scratch.

Symbolic code generation can achieve exceptional efficiency. Rather than general-purpose dynamic algorithms that work for any robot, generate specialized code for one robot's specific kinematic structure and parameters. Computer algebra systems derive and simplify symbolic expressions for M, C, and G, then generate optimized C code. This eliminates unnecessary operations and enables aggressive compiler optimization.

Parallel computation offers additional speedup. Computing dynamics for different kinematic chains simultaneously exploits multi-core processors. GPU acceleration can parallelize operations like matrix multiplication and forward kinematics across many end-effectors or trajectory waypoints. However, the recursive structure of many algorithms limits parallelization opportunities.

### From Theory to Implementation

Implementing kinematics and dynamics in a real humanoid system requires bridging multiple abstraction layers. At the highest level, task planning specifies desired end-effector trajectories or contact sequences. Inverse kinematics converts these to joint-space trajectories. Dynamic models predict required torques. Low-level control drives motors to track commanded trajectories.

Modeling accuracy matters enormously. Inaccurate inertial parameters cause feedforward torques to miss their targets, requiring feedback control to compensate. Unmodeled friction creates steady-state errors. Joint flexibility not captured in rigid-body models introduces oscillations. System identification procedures measure actual parameters by exciting the robot with known trajectories and fitting models to observed responses.

Kinematic and dynamic models also serve purposes beyond control. Simulation enables testing algorithms before deploying on hardware, reducing risk and development time. State estimation combines noisy sensor measurements with dynamic predictions to produce better estimates of joint positions and velocities. Fault detection compares predicted and actual torques to identify damaged joints or unexpected external forces.

The mathematical foundations covered in this chapter underpin essentially all aspects of humanoid robot development. Motion planning relies on forward kinematics to evaluate candidate trajectories. Optimization-based control requires dynamic gradients. Even learning-based approaches benefit from incorporating kinematic structure into neural network architectures or using model-based rollouts to improve sample efficiency.

## Conceptual Diagrams

### Forward Kinematics Chain Diagram

```
Base Frame {0}
    |
    | theta1 (revolute)
    |
Joint 1 ----[Link 1: L1, m1, I1]----
    |
    | theta2 (revolute)
    |
Joint 2 ----[Link 2: L2, m2, I2]----
    |
    | theta3 (revolute)
    |
End-Effector Frame {3}

Transformation Chain:
T_03 = T_01(theta1) * T_12(theta2) * T_23(theta3)

Each T matrix contains:
- Rotation from DH parameters (alpha, theta)
- Translation from DH parameters (a, d)
```

### Humanoid Kinematic Tree Structure

```
                    Head
                     |
                   Neck
                     |
    Left Arm --- Torso --- Right Arm
    (7 DOF)       |        (7 DOF)
                  |
            Pelvis/Base
              /      \
             /        \
        Left Leg    Right Leg
        (6 DOF)     (6 DOF)

Total: ~30+ DOF for typical humanoid
Each limb has own FK/IK
Coupled through torso/pelvis
```

### Jacobian Velocity Relationship

```
Joint Space          Jacobian           Task Space
                       (J)
q1_dot              [J11 ... J16]        v_x
q2_dot              [J21 ... J26]        v_y
q3_dot       --->   [J31 ... J36]  --->  v_z
q4_dot              [J41 ... J46]        omega_x
q5_dot              [J51 ... J56]        omega_y
q6_dot              [J61 ... J66]        omega_z

v = J * q_dot
Dimensions: [6x1] = [6xn] * [nx1]

For redundant robots (n > 6):
- Infinite solutions exist
- Null space allows secondary objectives
```

### Singularity Configuration Example

```
Normal Configuration (Non-singular):

    O---\____     Elbow bent
         \    \
          \____O  End-effector

Manipulability > 0
Can move in all directions

Singular Configuration:

    O---------O---------O  Fully extended

Manipulability = 0
Cannot move directly toward/away from base
Joint velocities -> infinity for certain end-effector velocities
```

### Dynamic Force Propagation

```
Backward Force Propagation (Newton-Euler):

End-Effector
    |
    | F_ext (external force)
    v
Link n
    | F_n, tau_n (computed from F_ext + inertial forces)
    v
Link n-1
    | F_(n-1), tau_(n-1) (accumulated forces)
    v
...
    v
Link 1
    | F_1, tau_1
    v
Base

Joint Torque = (projected tau along joint axis) + inertial contributions
```

### Equations of Motion Components

```
tau = M(q)*q_ddot + C(q,q_dot)*q_dot + G(q) + tau_ext

M(q): Mass Matrix
[m11(q)  m12(q)  ... ]     Configuration-dependent inertia
[m21(q)  m22(q)  ... ]     Symmetric, positive definite
[  ...     ...    ... ]     Dimension: n x n

C(q,q_dot): Coriolis/Centrifugal Matrix
- Velocity-dependent forces
- Captures interaction between moving joints
- Dimension: n x n

G(q): Gravity Vector
[g1(q)]     Gravitational torques at each joint
[g2(q)]     Depends on configuration and link masses
[ ... ]     Dimension: n x 1

tau_ext: External Torques
- Contact forces
- Applied loads
- Mapped via Jacobian transpose: J^T * F_external
```

### Inverse Kinematics Solution Multiplicity

```
Target Position: (x, y)

Elbow-Up Solution:        Elbow-Down Solution:

    O                         O----
     \                             \
      \                             \
       \                             O
        O

theta1a, theta2a          theta1b, theta2b

Both reach target!
Selection criteria:
- Distance from current configuration
- Joint limit avoidance
- Singularity avoidance
- Collision avoidance
- Manipulability optimization
```

### Contact Dynamics Diagram

```
Humanoid in Double Support:

        [Torso/Mass]
           /    \
          /      \
    [Left Foot]  [Right Foot]
         |            |
    ========Ground========

Constraints:
- Foot positions fixed: p_left = const, p_right = const
- No penetration: z >= 0
- Friction cone: |F_tangential| <= mu * F_normal
- No pulling: F_normal >= 0

Modified Dynamics:
M*q_ddot + C*q_dot + G = tau + J_left^T * F_left + J_right^T * F_right

Subject to: J_left * q_ddot = 0, J_right * q_ddot = 0
```

## Knowledge Checkpoint

Test your understanding of humanoid kinematics and dynamics with these questions:

1. **Forward Kinematics**: Explain how the Denavit-Hartenberg convention reduces the number of parameters needed to describe the transformation between adjacent joint frames. Why are exactly four parameters sufficient?

2. **Inverse Kinematics**: A 7-DOF humanoid arm must position its hand at a specific point in 3D space without constraining orientation (3 constraints). How many degrees of freedom remain unconstrained? What practical purposes might these extra degrees of freedom serve?

3. **Jacobian Matrix**: If a 6-DOF arm's Jacobian has a very small determinant, what does this indicate about the robot's configuration? What practical problems might arise when attempting Cartesian velocity control in this configuration?

4. **Singularities**: Describe three different strategies for handling kinematic singularities in humanoid motion planning and control. What are the trade-offs of each approach?

5. **Redundancy Resolution**: For a redundant humanoid arm maintaining its hand position while reaching into a confined space, how would you use null-space motion to avoid obstacles without disturbing the hand?

6. **Mass Matrix**: Explain why the mass matrix M(q) in the dynamic equations depends on configuration q, even though individual link masses and inertias are constant. Provide an intuitive example.

7. **Coriolis Forces**: When a humanoid rapidly swings its arm sideways while rotating its torso, significant Coriolis forces arise. Explain the physical origin of these forces and why they must be compensated in the control system.

8. **Newton-Euler vs Lagrangian**: Compare the computational complexity of the recursive Newton-Euler formulation with direct application of the Lagrangian method. Why is Newton-Euler preferred for real-time control?

9. **Motor Sizing**: A humanoid's shoulder joint must support the arm (mass 3 kg, center of mass 0.3 m from joint) horizontally. Estimate the required torque accounting only for gravity. If the arm must accelerate upward at 5 m/s^2, how does the torque requirement change?

10. **Contact Constraints**: When a humanoid places both feet flat on the ground, these contacts create kinematic constraints. Explain how these constraints modify the equations of motion and why the system becomes a differential-algebraic equation (DAE).

11. **Workspace Analysis**: How do joint limits, singularities, and obstacle avoidance each restrict the humanoid arm's reachable workspace? Which restriction is kinematic, which is practical, and which is fundamental?

12. **Whole-Body IK**: When computing inverse kinematics for a humanoid with both feet fixed, both hands reaching targets, and the center of mass constrained over the support polygon, you have more constraints than a single kinematic chain can handle. How would you formulate this as an optimization problem?

## Chapter Summary

This chapter explored the mathematical foundations governing how humanoid robots move through space and interact with their environment. We began with forward kinematics, which computes end-effector positions from joint angles using transformation matrices and Denavit-Hartenberg parameters. The systematic DH convention reduces each joint transformation to four parameters, enabling compact representation of complex kinematic chains.

Inverse kinematics reverses this process, finding joint angles that achieve desired end-effector poses. We examined both analytical solutions, which provide closed-form answers for specific kinematic structures, and numerical methods like Jacobian pseudo-inverse, which work for general configurations but require iterative solution. The challenge of solution multiplicity and the advantage of kinematic redundancy emerged as key themes.

The Jacobian matrix connects joint velocities to end-effector velocities, enabling Cartesian velocity control and revealing manipulability properties. Singularities occur where the Jacobian loses rank, creating configurations where certain motions become impossible or require infinite joint velocities. Understanding and avoiding singularities is critical for robust motion control.

Robot dynamics incorporates forces and torques, extending purely geometric kinematics to predict actual physical behavior. The equations of motion relate joint torques to accelerations through the configuration-dependent mass matrix, velocity-dependent Coriolis and centrifugal terms, and gravitational torques. Two complementary formulations emerged: the recursive Newton-Euler approach optimized for numerical efficiency, and the energy-based Lagrangian method providing elegant mathematical structure.

Contact constraints fundamentally alter the dynamics, creating differential-algebraic equations where contact forces must be solved simultaneously with motion. Humanoids continuously transition between contact configurations during locomotion, requiring careful handling of hybrid dynamics.

Practical considerations included motor sizing based on dynamic torque requirements, computational efficiency through recursive algorithms and code generation, and leveraging established libraries like KDL, RBDL, and Pinocchio. These tools encapsulate sophisticated algorithms while providing clean interfaces for robot modeling and computation.

The concepts in this chapter form the foundation for essentially all higher-level humanoid capabilities. Motion planning relies on forward kinematics to evaluate trajectories. Control systems use inverse kinematics to translate task specifications into joint commands. Dynamic models enable feedforward control and accurate simulation. Balance and locomotion algorithms fundamentally depend on managing the center of mass and contact forces through dynamic equations.

Mastering kinematics and dynamics provides the mathematical language for reasoning about robot motion and the computational tools for implementing sophisticated behaviors. As humanoid robots take on increasingly complex tasks in unstructured environments, these foundational concepts remain central to achieving capable, robust, and efficient performance.

## Further Reading

### Foundational Textbooks

1. Murray, R. M., Li, Z., & Sastry, S. S. (1994). "A Mathematical Introduction to Robotic Manipulation." CRC Press.
   - Rigorous mathematical treatment using geometric methods, screw theory, and Lie groups. Advanced but provides deep understanding of kinematic foundations.

2. Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G. (2009). "Robotics: Modelling, Planning and Control." Springer.
   - Comprehensive coverage of kinematics, dynamics, and control with detailed examples. Excellent balance between theory and application.

3. Craig, J. J. (2017). "Introduction to Robotics: Mechanics and Control" (4th ed.). Pearson.
   - Classic textbook with clear explanations of DH parameters, Jacobians, and dynamics. Includes numerous worked examples and exercises.

### Humanoid-Specific Resources

4. Kajita, S., Hirukawa, H., Harada, K., & Yokoi, K. (2014). "Introduction to Humanoid Robotics." Springer.
   - Dedicated to humanoid robots with detailed coverage of kinematics, dynamics, and their application to walking and manipulation.

5. Goswami, A., & Vadakkepat, P. (Eds.). (2019). "Humanoid Robotics: A Reference." Springer.
   - Multi-volume reference covering all aspects of humanoid robotics, including extensive sections on modeling and control.

### Computational Methods

6. Featherstone, R. (2014). "Rigid Body Dynamics Algorithms." Springer.
   - Definitive reference on efficient algorithms for computing robot dynamics. Essential for understanding modern implementations.

7. Park, F. C., & Lynch, K. M. (2017). "Introduction to Robotics: Mechanics, Planning, and Control." Cambridge University Press.
   - Modern approach using product of exponentials formulation and geometric methods. Excellent for understanding alternative kinematics representations.

### Software and Tools Documentation

8. Pinocchio Documentation: https://stack-of-tasks.github.io/pinocchio/
   - Comprehensive documentation for the high-performance dynamics library, including tutorials and examples.

9. ROS 2 Control Documentation: https://control.ros.org/
   - Framework documentation showing how kinematics and dynamics integrate into complete robot control systems.

### Research Papers

10. Nakamura, Y. (1991). "Advanced Robotics: Redundancy and Optimization." Addison-Wesley.
    - Foundational work on kinematic redundancy and its exploitation for secondary objectives.

11. Sentis, L., & Khatib, O. (2005). "Synthesis of Whole-Body Behaviors through Hierarchical Control of Behavioral Primitives." International Journal of Humanoid Robotics, 2(4), 505-518.
    - Influential paper on whole-body control of humanoid robots using operational space formulation.

### Online Resources

12. Modern Robotics Specialization (Coursera): Northwestern University course with accompanying textbook and software.
    - Free online course covering kinematics, dynamics, and motion planning with excellent visualizations.

13. Underactuated Robotics (MIT OpenCourseWare): Russ Tedrake's course materials on dynamics and control.
    - Focuses on underactuated systems including walking robots, with emphasis on dynamic modeling.

## Looking Ahead

With a solid foundation in kinematics and dynamics, we're prepared to tackle one of the most challenging aspects of humanoid robotics: bipedal locomotion and balance. Chapter 12 builds directly on the concepts developed here, applying them to the complex problem of walking on two legs.

Walking requires continuous management of the robot's dynamics to maintain balance while progressing forward. The Zero Moment Point (ZMP) criterion, which we'll explore in depth, uses dynamic equations to ensure the robot won't tip over. Center of mass kinematics, computed through forward kinematics of all links, determines whether the robot remains balanced. Trajectory generation creates joint angle profiles that achieve desired foot placements while satisfying dynamic constraints.

The Jacobian matrices we studied enable computing contact forces from joint torques, essential for verifying that the robot can maintain foot contact without slipping. Inverse kinematics translates desired foot and pelvis trajectories into joint commands. Dynamic models predict whether planned motions respect actuator torque limits.

Beyond walking, we'll examine how Model Predictive Control uses dynamic models to optimize future trajectories in real-time. The computational efficiency techniques discussed here become critical when dynamics must be evaluated thousands of times per control cycle. Understanding singularities helps explain why certain leg configurations create balance difficulties.

Balance control requires managing the relationship between gravitational forces, inertial forces from motion, and ground reaction forces. The dynamic equations of motion provide the framework for analyzing these forces. Contact dynamics, briefly introduced here, become central to understanding how foot-ground interactions constrain and enable locomotion.

As we progress to manipulation (Chapter 13) and human-robot interaction (Chapter 14), kinematic and dynamic concepts continue to play essential roles. Grasp stability depends on force analysis through contact Jacobians. Compliant control for safe interaction requires accurate dynamic models to predict collision forces. Natural motion generation benefits from understanding the robot's kinematic capabilities and limitations.

The mathematical tools developed in this chapter—transformation matrices, Jacobians, dynamic equations—serve as the language in which we express robot capabilities and control objectives. Comfort with these concepts enables understanding advanced research papers, debugging control problems, and designing novel behaviors. The investment in mastering these foundations pays dividends throughout the entire field of humanoid robotics.
