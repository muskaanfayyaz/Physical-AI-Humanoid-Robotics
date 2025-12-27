# Chapter 12: Bipedal Locomotion and Balance

## Introduction

Walking on two legs seems effortless to humans, yet it represents one of the most complex challenges in robotics. Every step involves precise coordination of dozens of joints, continuous balance maintenance against gravity and inertial forces, and adaptation to terrain variations. Unlike wheeled robots that enjoy passive stability, bipedal humanoids are inherently unstable systems that must actively control their dynamics to avoid falling.

The challenge of bipedal locomotion combines multiple technical domains: kinematics to position the legs, dynamics to manage forces and accelerations, control theory to stabilize inherently unstable motion, and planning to generate feasible trajectories. A walking robot must simultaneously satisfy kinematic constraints (feet must move along valid trajectories), dynamic constraints (ground reaction forces must keep the robot balanced), and physical limits (joint angles, velocities, torques within bounds).

This chapter explores the fundamental principles and practical techniques that enable humanoid robots to walk. We begin with the biomechanics of walking—the gait cycle phases and how weight transfers between feet. The Zero Moment Point (ZMP) emerges as the central concept for balance analysis, providing a mathematical criterion for stable walking. We examine how to generate walking patterns that maintain ZMP within stable regions, explore advanced concepts like the Capture Point for analyzing dynamic balance, and investigate Model Predictive Control approaches that optimize future motion in real-time.

Understanding bipedal locomotion requires integrating knowledge from previous chapters—kinematics to compute foot and center of mass positions, dynamics to predict forces and torques, and control theory to track desired trajectories while maintaining stability. By mastering these concepts, you'll understand both the fundamental principles that govern walking and the practical algorithms that enable real humanoid robots to traverse complex environments.

## Core Concepts

### The Gait Cycle and Walking Phases

Human walking follows a repeating pattern called the gait cycle, which begins when one foot contacts the ground and ends when that same foot contacts again. This cycle divides into distinct phases that humanoid robots must replicate to achieve natural, stable walking.

The stance phase occurs while a foot remains in contact with the ground, bearing some or all of the robot's weight. It begins at heel strike (initial contact) and continues until toe-off when the foot leaves the ground. During stance, the foot provides support and propulsion. The swing phase describes the period when a foot lifts off, swings forward through the air, and prepares for the next heel strike.

A complete gait cycle for one leg includes one stance and one swing phase, but the two legs operate with offset phases. When the right leg is in stance, the left leg may be in swing, and vice versa. This creates alternating periods of single support (one foot on the ground) and double support (both feet on the ground).

Double support phases occur twice per gait cycle: when the front foot contacts while the rear foot hasn't lifted yet, and when the rear foot lifts while the front foot has already landed. During double support, the robot enjoys greater stability since both feet can exert forces on the ground. Weight transfer from one leg to the other happens during these phases.

Single support phases are dynamically more challenging. With only one foot providing support, the robot must precisely control its center of mass trajectory to maintain balance. The supporting leg must provide all vertical support force and any necessary corrective torques. The swing leg's motion affects the overall center of mass and creates inertial forces throughout the robot.

Walking velocity determines the relative duration of each phase. Slow walking includes longer double support periods, providing more stable weight transfer. As walking speed increases, double support duration decreases and may vanish entirely during running, where a flight phase with no ground contact appears.

### Static versus Dynamic Balance

The distinction between static and dynamic balance fundamentally shapes walking strategies. Static balance requires the center of mass (CoM) vertical projection to remain within the support polygon—the convex hull of all ground contact points. If this condition holds and the robot remains stationary, it won't fall.

For a stationary humanoid with both feet flat on the ground, the support polygon is the rectangular region between and including both feet. As long as the CoM projection stays within this region, the robot can maintain balance without moving. This static stability criterion guides slow, careful walking where the CoM never ventures outside the support polygon.

Dynamic balance permits the CoM projection to temporarily exit the support polygon, provided the robot's momentum allows it to return. A walking human constantly violates static balance: during single support, the CoM moves outside the supporting foot's area but momentum carries it forward until the other foot lands, re-establishing balance. This dynamic strategy enables faster, more efficient walking.

The inverted pendulum serves as the simplest model of bipedal balance. Imagine a point mass atop a massless rod whose base can move horizontally. If the base remains stationary beneath the mass (or moves slowly), the pendulum stays upright through static balance. If the pendulum tilts forward, the base must accelerate forward to catch it—dynamic balance through motion.

Humanoid walking resembles continuously catching yourself from falling forward. Each step begins with the CoM ahead of the support foot, creating a toppling tendency. The swing leg reaches forward and plants, creating new support before the robot falls. This controlled falling and recovery, repeated cyclically, produces walking.

Understanding this fundamental instability explains why walking control is challenging. Unlike statically stable systems that naturally resist disturbances, bipedal robots require active control every millisecond to prevent falling. Sensor feedback, predictive models, and rapid control response all become essential.

### Center of Mass and Center of Pressure

The center of mass (CoM) represents the average position of the robot's mass distribution. For a rigid body, it's the point where gravitational force effectively acts. For multi-body systems like humanoids, the overall CoM is the mass-weighted average of individual link centers of mass.

Computing the CoM position requires knowing each link's mass and center of mass location:

```
CoM = (sum over all links i of: m_i * p_i) / (sum over all links i of: m_i)
```

where m_i is link i's mass and p_i is its center of mass position in world coordinates. Forward kinematics provides each p_i based on current joint angles, making CoM computation straightforward given an accurate model.

The center of pressure (CoP) represents the point on the ground where the resultant contact force effectively acts. For a foot on the ground, distributed pressure across the sole creates forces at many points. The CoP is the weighted average position where a single equivalent force would create the same total force and moment.

Ground reaction forces must support the robot's weight and any additional forces from acceleration. The vertical component equals the robot's weight plus vertical acceleration (F_z = m * (g + a_z)). Horizontal components balance any horizontal accelerations. The total force vector acts at the CoP.

During double support, each foot exerts forces with its own CoP. The overall CoP lies somewhere between the two feet, shifting from rear to front foot as weight transfers during walking. In single support, the CoP must remain within the supporting foot's contact area; if it reaches the foot's edge, the foot will rotate and the robot will fall.

The relationship between CoM and CoP determines balance. If the CoM is directly above the CoP and both are stationary, the robot is in static equilibrium. If they differ, gravitational and inertial forces create a moment about the CoP, causing angular acceleration. Walking control manages this relationship to produce stable motion.

### Zero Moment Point: The Fundamental Balance Criterion

The Zero Moment Point (ZMP) is perhaps the most important concept in bipedal walking. It provides a mathematical criterion for determining whether a planned walking motion will maintain balance and avoid tipping over. Despite its central role, ZMP is often misunderstood, so we'll develop the concept carefully.

Consider a humanoid robot with its feet on the ground. Forces and moments act at the contact points: gravity pulls downward, ground reaction forces push upward, and various moments arise from dynamics. Now imagine a horizontal plane just above the ground surface and ask: is there a point on this plane where the total moment (except the moment about the vertical axis) equals zero?

If such a point exists and lies within the support polygon, it's the ZMP. The robot can maintain this motion without tipping. If no such point exists within the support polygon, the robot will begin to rotate about the foot edge—the beginning of a fall.

Mathematically, ZMP represents the point where the sum of moments due to gravity and inertial forces equals zero (excluding yaw moments). Computing ZMP requires the full dynamic state: positions, velocities, and accelerations of all links. The ZMP coordinates (x_zmp, y_zmp) are:

```
x_zmp = (sum of: m_i * (g + a_zi) * x_i - sum of: m_i * a_xi * z_i) / (sum of: m_i * (g + a_zi))
y_zmp = (sum of: m_i * (g + a_zi) * y_i - sum of: m_i * a_yi * z_i) / (sum of: m_i * (g + a_zi))
```

where the sums run over all links, m_i is link mass, (x_i, y_i, z_i) is link CoM position, and (a_xi, a_yi, a_zi) are CoM accelerations.

The ZMP stability criterion states: if the computed ZMP lies strictly within the support polygon, the robot can execute the motion without rotating about foot edges. If ZMP reaches the boundary, the foot is about to rotate. If ZMP falls outside, the robot is already rotating—it's falling.

This criterion provides a powerful tool for walking control. By computing the ZMP for planned motions before executing them, we can verify stability. By adjusting motion plans to keep ZMP within safe margins from the support polygon boundary, we ensure stable walking even with modeling errors or disturbances.

ZMP differs subtly from CoP. The CoP is measured directly from ground reaction forces and always lies within the contact area (you can't push where you're not touching). The ZMP is computed from the robot's motion and may theoretically lie outside the support polygon. When ZMP is inside, ZMP equals CoP. When the computed ZMP is outside, the actual CoP is at the support polygon edge nearest to the computed ZMP, and the robot is rotating.

### Walking Pattern Generation Using ZMP

With the ZMP criterion in hand, we can generate walking trajectories that maintain balance. The approach involves planning trajectories for key points—typically the CoM and swing foot—such that the resulting ZMP remains within the support polygon throughout the gait cycle.

The planning problem has multiple layers. At the highest level, we specify footstep locations: where each foot should land during walking. These footsteps define the support polygon at each phase of the gait. At the intermediate level, we plan the CoM trajectory that produces acceptable ZMP trajectories for the given footsteps. At the lowest level, we compute full-body joint angles via inverse kinematics that achieve the planned CoM and foot motions.

Simplified models enable tractable planning. The Linear Inverted Pendulum Model (LIPM) treats the robot as a point mass CoM atop massless legs, constrained to move at constant height. Despite severe simplifications, LIPM captures essential walking dynamics and permits analytical solutions.

For LIPM on flat ground with CoM height h, the horizontal dynamics decouple into independent x and y directions. In the x direction:

```
x_ddot = (g / h) * (x - x_zmp)
```

This simple equation relates CoM acceleration to the distance between CoM and ZMP. If we specify a ZMP trajectory x_zmp(t), we can integrate to find the CoM trajectory x(t) that produces it. Alternatively, we can specify a CoM trajectory and compute the resulting ZMP.

Preview control, developed by Kajita and colleagues, uses this relationship to generate CoM trajectories. Given a sequence of future footstep locations (defining ZMP reference positions), preview control computes a smooth CoM trajectory that tracks the ZMP reference while maintaining dynamically consistent motion. The controller looks ahead (previews) future footsteps to begin CoM motion early enough to achieve smooth weight transfer.

Pattern generation typically proceeds in stages:

1. Footstep planning: Determine where feet should land based on desired walking direction, velocity, and terrain
2. ZMP reference generation: Create a ZMP trajectory that transitions smoothly from one support polygon to the next
3. CoM trajectory generation: Compute a CoM path that produces the reference ZMP using LIPM dynamics
4. Swing foot trajectory generation: Plan how the swing foot lifts, swings forward, and lands
5. Full-body inverse kinematics: Compute joint angles that achieve the planned CoM, stance foot, and swing foot positions
6. Dynamic filtering: Verify and adjust trajectories to account for full-body dynamics beyond LIPM simplifications

This process generates a reference trajectory that satisfies kinematic and dynamic constraints. During execution, feedback control tracks this reference while compensating for modeling errors, disturbances, and terrain irregularities.

### Capture Point and Orbital Stability

While ZMP provides a local stability criterion for the current instant, the Capture Point (CP) concept addresses the question: if the robot stopped taking steps right now, could it avoid falling? This global stability measure complements ZMP and provides insight into recovery from disturbances.

The Capture Point, also called the Capture Point or Extrapolated Center of Mass (XCoM), represents the point on the ground where the robot must step to come to a complete stop. It accounts for both the current CoM position and its velocity, extrapolating where the CoM's momentum will carry it.

For the Linear Inverted Pendulum Model, the Capture Point has a simple formula:

```
CP = CoM_xy + (CoM_velocity_xy / omega)
```

where CoM_xy is the horizontal CoM position, CoM_velocity_xy is horizontal velocity, and omega = sqrt(g/h) is the natural frequency of the inverted pendulum with height h.

If the CP lies within the support polygon, the robot can step to that location and come to rest without falling. If the CP lies outside, the robot cannot stop without taking at least one more step. This provides immediate insight into stability: a robot with CP far outside its support polygon is in danger of falling and needs aggressive recovery actions.

During walking, the CP constantly moves. In the LIPM model, it follows a simple trajectory: it moves exponentially from its initial position toward the ZMP. If the ZMP is fixed, the CP converges to it asymptotically. This behavior forms the basis of orbital stability analysis.

Orbital stability asks whether a walking pattern is inherently stable across multiple steps. A periodic walking gait is orbitally stable if small perturbations decay over subsequent steps rather than growing. The Capture Point provides elegant tools for analyzing orbital stability: a gait is orbitally stable if the CP at the end of one step equals the CP at the beginning of that step in the limit cycle.

The stepping strategy for balance recovery becomes intuitive with CP: step toward the Capture Point. If a disturbance pushes the robot, its CoM velocity changes, shifting the CP. By stepping toward the new CP location, the robot can arrest the disturbance and regain stability. This strategy underlies many push recovery controllers.

### Model Predictive Control for Walking

Model Predictive Control (MPC) offers a powerful framework for walking control that can optimize over multiple future steps while respecting constraints. Rather than following pre-generated trajectories, MPC solves an optimization problem at each control cycle to determine the best control actions given the current state and predictions of future behavior.

The MPC approach involves several key elements:

1. Predictive model: A mathematical model (often LIPM) predicts how the robot's state will evolve given control inputs
2. Receding horizon: The optimization looks ahead a fixed time window into the future, but only the first control action is executed
3. Cost function: An objective that penalizes deviations from desired behavior (e.g., tracking a reference velocity, minimizing energy)
4. Constraints: Hard limits on ZMP location, foot placement, joint limits, etc.

At each control cycle, MPC solves an optimization problem: find the sequence of control inputs over the prediction horizon that minimizes the cost function while satisfying all constraints, given the current measured state. After solving, MPC applies only the first control action, then measures the new state and repeats the optimization.

For walking, control inputs typically include footstep locations and timing, while the state includes CoM position, velocity, and current support configuration. The model predicts how the CoM will move given planned footsteps. The cost function might penalize deviation from a desired walking velocity, excessive ZMP variation, or large control effort.

Formulating walking MPC requires discretizing time into steps and using the LIPM dynamics to relate states across time steps. The ZMP constraint (must remain in support polygon) becomes inequality constraints in the optimization. Footstep constraints (where the robot can feasibly step) add additional inequalities.

MPC provides several advantages over open-loop trajectory generation:

- Adaptability: Re-optimization at each step accounts for disturbances and model errors
- Constraint handling: Hard constraints ensure feasibility and safety
- Optimality: The solution optimizes a meaningful objective rather than following heuristics
- Flexibility: Changing objectives or constraints doesn't require redesigning the controller

However, MPC demands substantial computation. Each control cycle requires solving a constrained optimization problem, often a quadratic program with dozens of variables and constraints. Real-time implementation requires fast solvers and careful problem formulation to ensure solutions complete within milliseconds.

Recent humanoid robots increasingly use MPC for locomotion, enabled by improved computational hardware and efficient optimization algorithms. The ability to adapt footsteps online, handle uneven terrain, and optimize energy consumption provides significant practical advantages over purely feedforward pattern generation.

### Trajectory Generation for Bipedal Motion

Beyond high-level planning of CoM and footsteps, walking requires generating detailed trajectories for all degrees of freedom. These trajectories must satisfy kinematic constraints (feet and pelvis follow planned paths), maintain balance (ZMP within bounds), respect joint limits and velocity limits, and produce smooth, continuous motion.

Swing foot trajectories require particular attention. The foot must lift clear of the ground to avoid tripping, swing forward smoothly to the next footstep location, and land gently to avoid impact forces. Common approaches use polynomial splines or Bezier curves that interpolate between lift-off and landing positions while satisfying velocity and acceleration constraints at endpoints.

A typical swing trajectory has three phases:

1. Lift: The foot accelerates upward and forward from its stance position, clearing the ground
2. Swing: The foot moves at roughly constant height while advancing forward
3. Land: The foot decelerates and descends to the landing position, arriving with downward velocity near zero

The maximum swing height must exceed expected terrain variations plus a safety margin. Too low risks catching on obstacles; too high wastes energy and time. The forward velocity should roughly match the overall walking speed for smooth motion.

Torso trajectory planning maintains posture and assists balance. Tilting the torso forward during walking shifts the CoM forward, affecting ZMP location. Rotating the torso can compensate for swing leg inertia. However, excessive torso motion appears unnatural and can indicate control problems. Smooth, minimal torso movement while maintaining upright posture generally produces the most efficient walking.

Arm swing provides counterbalance to leg motion and improves efficiency. When the right leg swings forward, the left arm swings forward simultaneously, canceling angular momentum. This reduces the torque required at the stance ankle and torso. While not strictly necessary for balance, arm swing measurably improves walking stability and energy efficiency.

Whole-body trajectory optimization can simultaneously determine all joint trajectories by posing walking as a large-scale optimization problem. The decision variables include all joint angles at discrete time steps. Constraints enforce kinematics, dynamics, collision avoidance, and joint limits. The objective minimizes energy, torque, or tracking error. While computationally expensive, trajectory optimization can produce highly efficient motions for complex scenarios.

### Fall Detection and Recovery

Despite careful planning, falls can occur due to unexpected disturbances, model errors, or terrain irregularities. Detecting incipient falls early and executing recovery strategies can prevent actual falls and enable robust walking in challenging environments.

Fall detection analyzes multiple indicators:

- ZMP approaching support polygon boundaries: Indicates loss of balance
- CoM velocity exceeding safe thresholds: Large momentum makes recovery difficult
- Capture Point outside support polygon: Impossible to stop without stepping
- Excessive torso tilt: Suggests rotational instability
- Joint torques saturating: Indicates inability to generate necessary forces

Combining multiple indicators improves reliability. A single indicator might trigger false alarms, but multiple simultaneous warning signs reliably indicate genuine fall risk.

Recovery strategies depend on the fall severity and direction:

For mild imbalance, adjusting the CoM trajectory may suffice. Accelerating the CoM toward the support polygon by leaning or shifting weight can bring the ZMP back within safe bounds. This requires sufficient torque authority and time before the fall progresses.

For moderate imbalance, rapid stepping toward the Capture Point arrests the fall. The controller discards the planned footstep sequence and immediately steps toward the CP, creating new support under the CoM's projected trajectory. This strategy can recover from surprisingly large disturbances if executed quickly enough.

For severe falls where recovery is impossible, minimizing impact damage becomes the goal. The robot should lower its CoM to reduce potential energy, protect sensitive components by landing on reinforced areas, and extend limbs to absorb impact over larger areas and longer times.

Proactive balance maintenance reduces fall risk. Maintaining ZMP margins from support polygon boundaries, limiting CoM velocity, avoiding singular leg configurations, and monitoring terrain for upcoming irregularities all help prevent falls before they begin.

### Terrain Adaptation

Real-world environments present uneven terrain, slopes, stairs, and obstacles that challenge walking systems designed for flat ground. Adapting to terrain variations requires perception, planning, and control capabilities beyond nominal walking.

Terrain perception uses vision systems, lidar, or depth cameras to build maps of the environment. Identifying flat surfaces for foot placement, detecting obstacles to avoid, estimating slope angles, and recognizing features like stairs enable informed footstep planning.

Footstep planning for rough terrain modifies the nominal pattern to place feet on stable, level surfaces. The planner considers reachability (can the robot step there given kinematics?), stability (will the foot have adequate support?), and safety (avoiding edges, unstable surfaces). Search algorithms like A* or RRT find sequences of footsteps that navigate from start to goal while respecting terrain constraints.

Slope walking requires adjusting the stance to maintain balance. On upward slopes, leaning forward shifts the CoM projection forward, keeping it within the support polygon despite the tilted ground. On downward slopes, leaning backward provides the same effect. The torso pitch angle must compensate for the slope angle while maintaining comfortable posture.

Stair climbing combines precise foot placement with increased joint range of motion. The swing leg must lift higher to clear each step. The stance leg must extend more to push the CoM upward. Handholds may provide additional support. Descending stairs is often more challenging than ascending because it requires controlled lowering of the CoM, testing the stance leg's eccentric strength and control precision.

Compliant control improves robustness to terrain irregularities. Rather than rigidly tracking planned trajectories, compliant controllers allow some deviation when forces indicate contact with unexpected surfaces. This prevents damage from impacts and enables the robot to adapt locally without high-level replanning.

Online trajectory adaptation updates the walking pattern based on real-time feedback. If the foot lands earlier than expected (terrain higher than estimated), the controller adjusts the CoM trajectory accordingly. If the foot slips, recovery actions activate. If terrain differs significantly from the map, replanning generates new footsteps.

### The Challenge of Underactuation

Bipedal walking involves fundamental underactuation: the robot has fewer control inputs than degrees of freedom when considering contact constraints. A humanoid in single support can exert forces through one foot, but the location of this force (the CoP) cannot be controlled arbitrarily—it must lie within the foot's contact area.

This underactuation creates a hierarchy of control objectives. The primary objective is maintaining balance; without it, all other goals become irrelevant. Secondary objectives like tracking desired walking velocity or minimizing energy can be pursued only to the extent that they don't compromise balance.

Hierarchical control formulations respect this priority structure. Balance constraints are strictly enforced as hard constraints. Trajectory tracking and other objectives are optimized subject to maintaining balance. If conflicts arise, balance always takes precedence.

The unactuated degrees of freedom—particularly the global position and orientation—must be controlled indirectly through their dynamic coupling to actuated joints. Moving the legs affects the CoM location, which affects balance, which determines whether the robot continues walking or falls. This indirect control requires understanding the full-body dynamics and carefully coordinating all joints.

Underactuation also creates vulnerabilities. If a disturbance exceeds the robot's ability to compensate through actuated joints alone, falling becomes inevitable. For example, a strong lateral push might drive the CoM beyond the foot's medial/lateral range faster than the robot can step to recover. Robust walking requires anticipating and avoiding such situations.

## Practical Understanding

### Computing the Zero Moment Point

Implementing ZMP-based walking begins with accurately computing the ZMP from the robot's state. While the mathematical formula appears straightforward, practical implementation requires careful attention to coordinate frames, efficient computation, and numerical stability.

The ZMP calculation requires knowing each link's mass, center of mass location, and acceleration. Forward kinematics provides positions; differentiating the kinematics gives velocities and accelerations. For real-time implementation, numerical differentiation of encoder positions provides velocity and acceleration estimates, though filtering is necessary to reduce noise.

The computation proceeds in several steps:

1. Update forward kinematics for all links given current joint angles
2. Compute or estimate velocities and accelerations (from finite differences or direct measurement)
3. Calculate each link's contribution to the ZMP numerator and denominator
4. Sum contributions and divide to get ZMP coordinates

Spatial algebra and recursive algorithms improve efficiency. Rather than computing each link independently, recursive approaches propagate velocities and accelerations through the kinematic tree, similar to the recursive Newton-Euler dynamics computation. This reduces complexity from O(n^2) to O(n) for n joints.

The ZMP formula assumes flat horizontal ground. For sloped or uneven terrain, the calculation must be performed in the ground plane's coordinate frame rather than a horizontal plane. Transformation matrices rotate the robot's state into the appropriate frame before ZMP computation.

Numerical issues can arise when the total vertical force approaches zero (e.g., during jumping). The ZMP formula involves division by the sum of vertical forces; near-zero denominators create numerical instability. Practical implementations add small epsilon values or use alternative stability criteria when vertical forces are small.

### Generating Walking Patterns with Preview Control

Preview control provides a systematic method for generating CoM trajectories that maintain ZMP within the support polygon. The approach uses a Linear Inverted Pendulum Model and solves an optimal control problem with preview of future ZMP references.

The LIPM dynamics in discrete time relate CoM position at the next time step to the current state and ZMP:

```
x[k+1] = A * x[k] + B * u[k]
```

where x contains CoM position and velocity, u is the ZMP position, and A and B are matrices derived from the continuous-time LIPM dynamics and discretization time step.

The preview controller minimizes a cost function that penalizes deviation from a reference ZMP trajectory and large changes in ZMP:

```
Cost = sum over k of: (Q * e[k]^2 + R * u[k]^2)
```

where e[k] is the ZMP tracking error and Q, R are weighting matrices balancing tracking accuracy against control effort.

The optimal control solution involves feedback from the current state and feedforward from the preview of future reference ZMP values. The control law takes the form:

```
u[k] = -K_x * x[k] + K_ref * sum over preview window of: (preview_gain[i] * zmp_ref[k+i])
```

Computing the gain matrices requires solving a Riccati equation, which can be done offline for a given set of parameters. During runtime, the controller simply applies the pre-computed gains to the current state and preview window.

Implementing preview control requires:

1. Footstep planning to determine support polygon transitions
2. ZMP reference generation that smoothly transitions between footstep centers
3. Setting up the LIPM model with appropriate CoM height and discretization time
4. Tuning Q and R matrices to balance tracking versus smoothness
5. Choosing preview window length (longer preview smooths motion but requires more computation)
6. Computing gain matrices offline
7. Running the controller in real-time with state feedback

The generated CoM trajectory maintains ZMP close to the reference while producing smooth, dynamically consistent motion. However, LIPM simplifications mean the actual full-body ZMP may differ from predictions. Feedback control during execution compensates for these model errors.

### Footstep Planning Fundamentals

Footstep planning determines where the robot should place its feet to walk toward a goal while maintaining balance and avoiding obstacles. This high-level planning layer provides inputs to lower-level trajectory generation.

For straight-line walking on flat ground, footstep planning is straightforward: alternate feet with constant step length and width. Step length typically ranges from 0.1 to 0.5 meters depending on robot size and desired speed. Step width (lateral separation between feet) should be sufficient for stability but not so wide that it wastes energy; typical values range from 0.15 to 0.3 meters.

Turning requires modifying footstep orientation. Each footstep rotates by some fraction of the total desired turn angle. Small turn angles (a few degrees per step) produce smooth turning. Large turn angles risk instability and require slowing the forward walking speed.

For complex environments, footstep planning becomes a search problem. The planner discretizes possible footstep locations into a grid or samples them continuously. Each potential footstep is evaluated for reachability (can kinematics achieve it?), stability (does it provide adequate support polygon?), and collision (does it avoid obstacles?). Graph search algorithms like A* find sequences of valid footsteps from start to goal.

The search state includes foot position, orientation, and which foot is currently in support. Transitions correspond to taking a step: from left support to right support by stepping with the right foot, or vice versa. Costs might include distance traveled, number of steps, energy, or risk.

Heuristics guide the search toward promising regions. The Euclidean distance to goal provides an admissible heuristic. More sophisticated heuristics account for orientation error, expected terrain difficulty, or learned cost-to-go from previous experience.

Footstep planning typically runs slower than real-time since it searches over many possibilities. However, it needs to replan faster than the robot executes steps. If the robot takes 1 second per step, the planner must find a new plan (or verify the existing plan remains valid) within 1 second. Practical implementations often compute plans several steps ahead, then incrementally extend the plan as the robot walks.

### Implementing Model Predictive Control

MPC for walking solves a quadratic program (QP) at each control cycle to optimize footstep locations and timing. While conceptually straightforward, efficient implementation requires careful problem formulation and choice of solver.

The decision variables typically include footstep locations for several steps ahead (perhaps 3-10 steps) and the state trajectory (CoM position and velocity) at each step. The dynamics constraints link these variables: given footstep locations, the LIPM dynamics determine how CoM evolves.

The cost function might include terms for:

- Velocity tracking: penalize deviation from desired walking velocity
- Reference tracking: penalize deviation from a reference CoM trajectory
- Control effort: penalize large footstep adjustments from nominal
- Regularization: penalize excessive changes from previous solution

Constraints include:

- Dynamics: LIPM equations relating states across time steps
- ZMP stability: ZMP must remain in support polygon
- Kinematic reachability: footsteps must be within kinematic reach
- Joint limits: resulting full-body motion must respect joint limits (often simplified or checked post-optimization)

The resulting QP has a sparse structure due to the dynamics constraints only coupling adjacent time steps. Exploiting this sparsity dramatically improves solution speed. Specialized QP solvers like qpOASES, OSQP, or Gurobi can solve walking MPCs with dozens of variables in milliseconds.

Warm-starting significantly reduces solution time. Rather than solving from scratch at each control cycle, initialize the optimization with the previous solution shifted forward one time step. Since the problem changes only slightly between cycles, the shifted solution is close to optimal and convergence is rapid.

The MPC formulation must run on the robot's onboard computer within the control cycle period, typically 10-50 milliseconds. This requires careful implementation:

- Minimize dynamic memory allocation (allocate buffers once at startup)
- Use efficient linear algebra libraries (Eigen, BLAS)
- Exploit problem sparsity
- Tune solver parameters for speed versus accuracy trade-offs
- Consider simplified dynamics or reduced horizon if computation is insufficient

### Capture Point-Based Stepping Control

The Capture Point provides an intuitive strategy for reactive balance control: when disturbed, step toward the CP. Implementing this strategy requires computing the CP, determining feasible step locations, and coordinating with nominal walking patterns.

Computing the CP from the current state is straightforward using the formula CP = CoM_xy + CoM_velocity_xy / omega. The challenging part is determining where to step. The ideal step location equals the CP, but kinematic constraints, obstacles, and terrain may prevent stepping exactly there.

The feasible stepping region depends on the current support configuration and kinematic limits. For a humanoid in single support on the right foot, the left foot can step within a region determined by leg reach, joint limits, and avoiding self-collision. This region is typically an ellipse or polygon around the current support.

If the CP lies within the feasible region, stepping directly to it arrests the disturbance. If the CP lies outside, stepping to the boundary point closest to the CP provides the best recovery. This may not fully stabilize the robot in one step, but subsequent steps can continue adjusting.

Integrating CP-based stepping with planned walking requires deciding when to abandon the plan and use reactive stepping. Simple strategies use thresholds: if the CP deviates more than some distance from the planned CP, activate reactive stepping. More sophisticated approaches blend nominal and reactive control based on the confidence in the plan and the severity of disturbance.

The timing of reactive steps matters. Immediately stepping upon detecting disturbance provides fastest response but may lead to excessive stepping that interferes with the primary task. Waiting for the next planned step reduces disruption but may allow the disturbance to grow. Adaptive timing based on CP deviation balances these concerns.

### Swing Foot Trajectory Generation

Creating smooth, collision-free swing foot trajectories requires considering multiple objectives: clearing the ground and obstacles, arriving at the target footstep with appropriate orientation and low velocity, and coordinating with the overall gait timing.

Polynomial splines provide a flexible representation. A fifth-order polynomial can independently satisfy position, velocity, and acceleration boundary conditions at both the start and end of the swing phase. The six polynomial coefficients are determined by the six boundary conditions.

For a swing trajectory from (x0, y0, z0) at time t0 to (x1, y1, z1) at time t1, we might specify:

- Initial position: (x0, y0, z0)
- Initial velocity: zero in z, forward velocity in x, y matching walking speed
- Final position: (x1, y1, z1)
- Final velocity: zero in z, forward velocity in x, y matching walking speed
- Maximum height: z_max occurs at time t_mid = (t0 + t1) / 2

The vertical trajectory might use a different polynomial to achieve ground clearance. A simple approach uses a cubic polynomial from z0 to z_max in the first half, then another cubic from z_max to z1 in the second half, ensuring continuous velocity at the midpoint.

Collision checking verifies that the swing trajectory doesn't intersect obstacles or the stance leg. If collisions are detected, the trajectory can be adjusted by increasing maximum height, shifting the lateral position, or modifying the timing.

The swing foot orientation typically transitions smoothly from the lift-off orientation to the landing orientation. Quaternion SLERP (spherical linear interpolation) provides smooth rotation interpolation, avoiding the singularities and discontinuities of Euler angle interpolation.

During execution, the swing trajectory serves as a feedforward reference. Feedback control tracks this reference, compensating for modeling errors and disturbances. Impedance control at the ankle allows some compliance during landing, reducing impact forces.

### Whole-Body Inverse Kinematics for Walking

With planned trajectories for the CoM, stance foot, and swing foot, inverse kinematics computes joint angles that achieve these Cartesian targets. For walking, this IK problem has multiple constraints that must be satisfied simultaneously.

The constraints typically include:

1. Stance foot position and orientation fixed (6 constraints)
2. Swing foot position and orientation following planned trajectory (6 constraints)
3. CoM at planned location (3 constraints)
4. Pelvis orientation upright or at planned angle (3 constraints)

A humanoid with 12 leg DOF (6 per leg) has exactly enough degrees of freedom to satisfy these 12-15 constraints. However, the solution may not exist if the constraints are incompatible (e.g., desired CoM too far from stance foot), or multiple solutions may exist.

Formulating this as an optimization problem handles inconsistencies gracefully:

```
minimize: sum over constraints i of: w_i * (error_i)^2
subject to: joint limits
```

where w_i are weights prioritizing different constraints. Stance foot constraints typically have very high weight (strict enforcement), while CoM and swing foot constraints might have lower weights allowing small deviations.

The optimization variables are joint angles. The objective measures how well current joint angles satisfy constraints. Gradient-based optimizers like BFGS or sequential quadratic programming can solve this quickly enough for real-time control.

For real-time performance, analytical IK for individual legs can provide initial guesses or even direct solutions. If the pelvis position and orientation are fixed, each leg's IK can be solved independently using analytical or numerical methods for 6-DOF manipulators. Then pelvis position can be adjusted to achieve the desired CoM.

Hierarchical IK handles priority levels explicitly. First, solve for joint angles satisfying high-priority constraints (stance foot position). Then, optimize remaining DOF to satisfy lower-priority constraints (swing foot, CoM) using null-space projections that don't disturb higher-priority tasks.

### Balance Feedback Control

Even with perfect planning, model uncertainties and external disturbances require feedback control to maintain balance during walking. Multiple feedback strategies can be combined:

ZMP feedback adjusts the CoM trajectory based on measured ZMP error. If the actual ZMP drifts toward the support polygon boundary, the controller shifts the CoM back toward the polygon center. This can be implemented as a simple proportional controller: CoM_adjustment = K_zmp * (ZMP_desired - ZMP_measured).

Capture Point feedback modifies footstep locations. If the CP deviates from its planned trajectory, the next footstep location is adjusted toward the actual CP. This provides rapid disturbance rejection by directly compensating for momentum errors.

Ankle torque feedback provides immediate response to small disturbances. Tilting the foot adjusts the CoP location within the support foot, providing local balance correction. This is fastest but has limited authority (only effective while the CoP can move within the foot).

Hip torque modulates the torso angle, shifting the CoM horizontally. This provides larger correction authority than ankle torque but is slower to take effect due to greater inertia.

Combining multiple feedback loops creates a cascade of balance controllers operating at different time scales:

1. Ankle torque: millisecond response, small authority
2. Hip/torso adjustment: tens of milliseconds, medium authority
3. CoM trajectory modification: hundreds of milliseconds, large authority
4. Footstep adjustment: seconds, largest authority

Each level compensates for disturbances within its capability. Small disturbances are handled by fast, local controllers. Large disturbances trigger higher-level adjustments that have greater authority but slower response.

Gain tuning balances responsiveness against oscillation. High gains provide aggressive disturbance rejection but may cause the robot to oscillate. Low gains are stable but allow large tracking errors. Adaptive gains that increase based on error magnitude provide good compromise.

### Handling Uneven Terrain

Walking on uneven terrain requires integrating perception, planning, and control beyond flat-ground capabilities. The process begins with terrain mapping to identify where to step and how to adapt gait.

Terrain mapping uses onboard sensors—cameras, lidar, depth sensors—to build a height map of the environment. Processing identifies flat regions suitable for foot placement, estimates local slope angles, detects obstacles, and assesses surface properties (slippery, compliant, rigid).

Footstep planning on uneven terrain searches for sequences of footholds that are:

- Reachable: within kinematic limits from current stance
- Stable: large enough and flat enough for secure support
- Safe: avoiding edges, steep slopes, or unstable surfaces

The planner might prefer certain regions (flat, level surfaces) while penalizing others (slopes, small footholds). Path planning algorithms like A* or sampling-based methods explore the space of possible footstep sequences.

Adapting the CoM trajectory for terrain requires accounting for foot height variations. When stepping up onto a higher surface, the CoM must rise accordingly. The LIPM assumption of constant CoM height breaks down, requiring either a variable-height model or transitions between different constant-height models.

Landing on uneven surfaces creates impact uncertainties. The foot might contact earlier or later than expected if height estimates are inaccurate. Compliant control allows the leg to absorb these impacts gracefully. Force sensing at the foot detects contact and triggers transition to stance phase even if timing differs from the plan.

Slope adaptation adjusts body posture to maintain balance on tilted ground. The torso pitch angle must compensate for the ground slope: on uphill slopes, lean forward; on downhill slopes, lean back. The adjustment keeps the CoM projection within the support polygon despite the tilted support surface.

Step-to-step adaptation updates the plan based on feedback. If the robot deviates from the planned trajectory due to terrain irregularities, subsequent steps are adjusted to compensate. If terrain differs significantly from the map, local replanning generates new footsteps using updated terrain information.

## Conceptual Diagrams

### Gait Cycle Phases

```
Complete Gait Cycle (One Full Cycle = Right Heel Strike to Right Heel Strike)

Right Leg:  |----STANCE PHASE----|----SWING PHASE----|
Left Leg:   |--SWING--|--STANCE PHASE----|--SWING--|

Detailed Phases:

Double     Single      Double    Single      Double
Support    Support     Support   Support     Support
|  DS1  |    SS1    |   DS2   |    SS2    |   DS3  |
|       |           |         |           |        |
v       v           v         v           v        v
RHS    LTO         LHS       RTO         RHS
(Right (Left       (Left     (Right      (Right
Heel   Toe-off)    Heel      Toe-off)    Heel
Strike)            Strike)               Strike)

DS = Double Support (both feet on ground, weight transfer)
SS = Single Support (one foot on ground, other swinging)
RHS/LHS = Right/Left Heel Strike
RTO/LTO = Right/Left Toe Off

Typical timing (1.0 sec gait cycle):
- DS1: 0.0-0.1 sec (10%)
- SS1: 0.1-0.5 sec (40%)
- DS2: 0.5-0.6 sec (10%)
- SS2: 0.6-1.0 sec (40%)
```

### Static vs Dynamic Balance

```
STATIC BALANCE:
     |          Support Polygon (top view)
     |          +-------------------+
     | CoM      |      [ ]CoM       |  <-- CoM projection inside
     |          |                   |      polygon = Static Stable
     v          +-------------------+
   Ground       Left Foot   Right Foot

If CoM moves slowly (quasi-static), remains stable.


DYNAMIC BALANCE:
     |           Support Polygon
     |           +------+
     | CoM       |      |    [ ]CoM  <-- CoM projection OUTSIDE
     |           |      |            but momentum carries forward
     v           +------+            until next foot lands
   Ground      Right Foot

                         (next instant)
                         +------+    +------+
                             [ ]CoM  |      |
                         +------+    +------+
                       Right Foot  Left Foot (just landed)

Walking = Controlled dynamic instability + recovery
```

### Zero Moment Point Illustration

```
Side View of Walking Robot:

           CoM (center of mass)
             *
            /|\
           / | \
          /  |  \
    Torso   |   Arms
           /|\
          / | \
         /  |  \
    Leg1   |   Leg2
       |   |   |
       |   v   |     <-- ZMP (point where moment = 0)
    ===|===X===|=======  Ground
       |       |
    Foot1    Foot2

ZMP Calculation:
- Sum all gravitational forces on each link
- Sum all inertial forces (mass * acceleration) on each link
- Find point where these create zero net moment

ZMP Stability Criterion:

Support Polygon (top view):
+-------------------------+
|                         |
|        X (ZMP)          |  <-- ZMP inside = Stable
|                         |      Can maintain this motion
+-------------------------+
Left Foot        Right Foot

+--------+
|        |
|        |      X (ZMP)     <-- ZMP outside = Unstable
+--------+                     Robot will tip over
```

### Linear Inverted Pendulum Model (LIPM)

```
Simplified Model for Walking Analysis:

      * m (point mass - entire robot CoM)
      |
      | length h (constant height)
      |
      O (massless rod)
     /|
    / | (can tilt in any direction)
   /  |
  /   v
=========== Ground (ZMP can move along ground)

Dynamics (horizontal x direction):

x_ddot = (g/h) * (x - x_zmp)

Where:
- x = CoM horizontal position
- x_zmp = ZMP horizontal position
- h = CoM height (constant)
- g = gravity

Key insight:
- If ZMP ahead of CoM -> CoM accelerates forward
- If ZMP behind CoM -> CoM accelerates backward
- Control ZMP to control CoM motion
```

### Capture Point Concept

```
Current State:              Can Stop Here?

  CoM at x_com              CP = x_com + x_dot/omega
  velocity x_dot
        -->                      Support Polygon
         [ ]CoM                  +--------+
                                 |    X CP|  <-- CP inside = YES
                                 +--------+
                                           Can step to CP and stop


Unstable State:             Cannot Stop!

  High velocity
      ---->
        [ ]CoM              CP far ahead
                                          +--------+
                                          |        |
                                          +--------+
                                                    X CP <-- outside

Must take multiple steps to stabilize.
Step toward CP at each step.

Recovery Strategy:
1. Compute CP = CoM_xy + velocity_xy / omega
2. If CP outside support, step toward it
3. Repeat until CP converges inside support
```

### Walking Pattern Generation Flow

```
High-Level Plan
    |
    | (desired velocity, direction)
    v
+-------------------+
| Footstep Planning |
+-------------------+
    |
    | (footstep positions, timing)
    v
+-----------------------+
| ZMP Reference         |  (where ZMP should be each instant)
| Generation            |
+-----------------------+
    |
    | (ZMP reference trajectory)
    v
+------------------------+
| CoM Trajectory         |  (use Preview Control or MPC)
| Generation             |  (what CoM motion produces this ZMP?)
+------------------------+
    |
    | (CoM trajectory)
    v
+-------------------------+
| Swing Foot Trajectory   |  (how swing foot moves through air)
| Generation              |
+-------------------------+
    |
    | (CoM + swing foot + stance foot targets)
    v
+---------------------------+
| Whole-Body Inverse        |  (what joint angles achieve these?)
| Kinematics                |
+---------------------------+
    |
    | (joint angle trajectories)
    v
+-----------------------------+
| Joint Position Control      |  (track these angles with motors)
+-----------------------------+
    |
    v
Actual Robot Motion
```

### Model Predictive Control Architecture

```
Current State (measured)
    |
    | (CoM pos/vel, feet positions)
    v
+------------------------+
| MPC Optimization       |
| Horizon: N steps ahead |
+------------------------+
    | Optimize over:
    | - Future footstep locations
    | - CoM trajectory
    |
    | Subject to:
    | - LIPM dynamics
    | - ZMP in support polygon
    | - Kinematic constraints
    |
    | Minimize:
    | - Velocity tracking error
    | - Control effort
    v
Optimal Control Sequence
    |
    | (apply only FIRST control action)
    v
Robot Executes One Step
    |
    | Measure new state
    v
(Repeat MPC optimization with updated state)

Receding Horizon:
Time:  0    1    2    3    4    5    6
Plan:  |====OPTIMIZE N STEPS====>|
Execute: X   (only first step)

Time:      1    2    3    4    5    6    7
Plan:      |====OPTIMIZE N STEPS====>|
Execute:    X   (only first step)

(Horizon "recedes" as robot advances)
```

### Terrain Adaptation

```
Flat Ground Walking:
======================  (constant CoM height h)
     h
    ---  [ ]CoM
        /   \
       /     \
      /       \
======================  Ground (z = 0)


Stepping Up:
           [ ]CoM        (CoM rises to maintain
          /   \           height above new stance)
         /     \
        /   =====#####   Step (height delta_z)
       /   ====#####
      /  ====#####
========#####
   ^
   Step up:
   - Swing leg lifts higher (clearance)
   - Stance leg extends to push CoM up
   - CoM height relative to stance maintained


Sloped Ground:
              [ ]CoM
         Torso / | \ (tilted to compensate slope)
              /  |  \
             /   |   \
            /    |    \
           /     |     \
        ==============
       /  Slope angle θ
      /
   ==/

Compensation:
- Torso pitch = -θ (lean into slope)
- Keeps CoM projection in support polygon
- Adjust ZMP calculations to sloped plane
```

### Fall Detection and Recovery

```
FALL DETECTION INDICATORS:

1. ZMP Approaching Boundary:
   +----------+
   |        X-| <-- ZMP near edge (warning!)
   +----------+

2. High CoM Velocity:
        --->-->  [ ]CoM  (excessive momentum)

3. Capture Point Outside:
   +----------+
   |          |
   +----------+
              X CP (cannot stop!)

4. Torso Tilt Excessive:

        /  [ ]  (tilted beyond threshold)
       /
      /

RECOVERY STRATEGIES:

Mild: Adjust CoM Trajectory
  [ ]CoM --adjust-->  [ ]CoM  (shift back toward center)

Moderate: Rapid Stepping to CP
  [ ]CoM
    -->      (step)
             +------+
             |  X CP|  (new support at CP)
             +------+

Severe: Protective Falling
  [ ]CoM
   |
   v  (lower CoM, extend arms, absorb impact)
  \|/
===X=== (minimize damage)
```

## Knowledge Checkpoint

Test your understanding of bipedal locomotion and balance:

1. **Gait Phases**: During the double support phase of walking, both feet are on the ground. Explain why this phase provides more stability than single support, and why humans shorten double support duration when walking faster.

2. **Static vs Dynamic Balance**: A humanoid robot walks forward with its center of mass projection briefly exiting the support polygon. Explain why this doesn't necessarily mean the robot will fall, using the concept of dynamic balance.

3. **ZMP Computation**: The ZMP formula includes both gravitational terms and inertial (acceleration) terms. Explain why a stationary robot's ZMP equals its CoM projection, but a moving robot's ZMP differs from its CoM projection.

4. **ZMP Stability Criterion**: If a walking controller computes a planned trajectory and finds the ZMP reaches the support polygon boundary during single support, what are three possible modifications to make the trajectory stable?

5. **Center of Pressure vs ZMP**: Explain the relationship between CoP (measured from force sensors) and ZMP (computed from dynamics). Under what conditions are they equal? When do they differ?

6. **LIPM Dynamics**: The Linear Inverted Pendulum Model equation x_ddot = (g/h) * (x - x_zmp) shows that CoM acceleration depends on the CoM-ZMP distance. If the ZMP is ahead of the CoM, which direction does the CoM accelerate? Why does this make intuitive sense?

7. **Preview Control**: Walking pattern generation using preview control looks ahead at future footstep locations. Explain why this preview is necessary—why can't the controller simply react to the current footstep location?

8. **Capture Point**: A humanoid in single support receives a push that adds velocity to its CoM. Explain how the Capture Point changes in response to this disturbance, and describe the recovery strategy.

9. **Kinematic Redundancy in Walking**: A humanoid leg typically has 6 DOF, which exactly suffices to position and orient the foot (6 constraints). During walking, the foot position is constrained by the planned footstep. How can the robot still have freedom to adjust the CoM height?

10. **MPC for Walking**: Model Predictive Control solves an optimization at each control cycle. Explain why MPC is more computationally expensive than feedforward pattern generation, yet is increasingly used on real humanoid robots.

11. **Swing Foot Trajectory**: Why do swing foot trajectories typically use higher-order polynomials (5th order or higher) rather than simple straight-line paths between lift-off and landing positions?

12. **Terrain Adaptation**: When walking uphill on a slope, the robot must lean forward to maintain balance. Explain this using the ZMP stability criterion and how the support polygon orientation changes on sloped ground.

13. **Underactuation**: During single support, explain why the humanoid robot is underactuated and how this underactuation constrains the walking controller's ability to track arbitrary CoM trajectories.

14. **Fall Recovery**: Describe the trade-offs between three fall recovery strategies: (1) ankle torque modulation, (2) hip/torso adjustment, and (3) rapid stepping. Consider response time, authority, and complexity.

## Chapter Summary

This chapter examined the complex problem of bipedal locomotion, exploring how humanoid robots walk while maintaining balance against gravity and inertial forces. We began with the fundamental structure of walking: the gait cycle with its alternating stance and swing phases, double support periods for stable weight transfer, and the distinction between static and dynamic balance.

The Zero Moment Point emerged as the central concept for analyzing and ensuring walking stability. ZMP represents the point where moments from gravity and inertial forces cancel, and its location relative to the support polygon determines whether the robot can maintain balance. We developed the mathematical foundation for computing ZMP from the robot's dynamic state and established the stability criterion: ZMP must remain within the support polygon.

Walking pattern generation transforms desired walking velocity and direction into detailed joint trajectories. The Linear Inverted Pendulum Model provides a simplified but tractable representation of walking dynamics, enabling analytical solutions and efficient computation. Preview control uses LIPM dynamics to generate CoM trajectories that maintain ZMP within bounds while smoothly tracking reference trajectories derived from planned footsteps.

Advanced concepts extended basic ZMP walking. The Capture Point characterizes global stability and provides intuitive stepping strategies for disturbance rejection: step toward the CP to arrest unwanted motion. Orbital stability analysis determines whether walking patterns are inherently stable across multiple steps. Model Predictive Control optimizes future motion in real-time, adapting to disturbances and changing objectives while respecting hard constraints.

Practical implementation requires generating complete motion plans spanning multiple abstraction layers: footstep planning determines where to step, CoM and swing foot trajectory generation creates Cartesian motion plans, and whole-body inverse kinematics computes joint angles that achieve these targets while satisfying all constraints. Feedback control compensates for modeling errors and external disturbances through multiple cascaded loops operating at different time scales.

Terrain adaptation extends walking capabilities beyond flat ground. Perception systems map the environment, identifying footholds and obstacles. Footstep planning searches for sequences of stable, reachable footholds. Gait adaptation adjusts body posture, step height, and timing to accommodate slopes, steps, and uneven surfaces. Compliant control allows graceful response to unexpected terrain variations.

Throughout the chapter, the theme of underactuation and constraint management appeared repeatedly. Walking requires maintaining balance as the primary constraint, with all other objectives subordinate. The controller must work within the limited authority provided by contact forces that can only push (not pull) and must remain within friction limits.

The principles and techniques developed here form the foundation for locomotion in humanoid robotics. While research continues to improve robustness, efficiency, and versatility, the core concepts—ZMP stability, dynamic walking, predictive control, and adaptive planning—remain central to state-of-the-art systems.

## Further Reading

### Foundational Papers

1. Vukobratovic, M., & Borovac, B. (2004). "Zero-Moment Point—Thirty Five Years of its Life." International Journal of Humanoid Robotics, 1(1), 157-173.
   - Historical overview and detailed explanation of ZMP concept by its original developers.

2. Kajita, S., Kanehiro, F., Kaneko, K., Fujiwara, K., Harada, K., Yokoi, K., & Hirukawa, H. (2003). "Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point." Proceedings of IEEE International Conference on Robotics and Automation.
   - Seminal paper introducing preview control for ZMP-based walking pattern generation.

3. Pratt, J., Carff, J., Drakunov, S., & Goswami, A. (2006). "Capture Point: A Step toward Humanoid Push Recovery." Proceedings of IEEE-RAS International Conference on Humanoid Robots.
   - Introduction of the Capture Point concept for balance analysis and recovery.

### Textbooks and Comprehensive Resources

4. Kajita, S., Hirukawa, H., Harada, K., & Yokoi, K. (2014). "Introduction to Humanoid Robotics." Springer.
   - Comprehensive coverage of bipedal walking including detailed mathematical derivations and practical implementation.

5. Westervelt, E. R., Grizzle, J. W., Chevallereau, C., Choi, J. H., & Morris, B. (2007). "Feedback Control of Dynamic Bipedal Robot Locomotion." CRC Press.
   - Advanced treatment of walking dynamics and control using hybrid systems theory.

6. Goswami, A., & Vadakkepat, P. (Eds.). (2019). "Humanoid Robotics: A Reference." Springer.
   - Multi-volume reference with extensive sections on locomotion, balance, and motion planning.

### Model Predictive Control for Walking

7. Wieber, P. B. (2006). "Trajectory Free Linear Model Predictive Control for Stable Walking in the Presence of Strong Perturbations." Proceedings of IEEE-RAS International Conference on Humanoid Robots.
   - Early application of MPC to bipedal walking with focus on disturbance rejection.

8. Herdt, A., Diedam, H., Wieber, P. B., Dimitrov, D., Mombaur, K., & Diehl, M. (2010). "Online Walking Motion Generation with Automatic Footstep Placement." Advanced Robotics, 24(5-6), 719-737.
   - MPC framework that optimizes both CoM trajectory and footstep locations online.

### Dynamic Walking and Limit Cycles

9. McGeer, T. (1990). "Passive Dynamic Walking." International Journal of Robotics Research, 9(2), 62-82.
   - Foundational work on passive dynamic walkers demonstrating natural gait emergence from mechanical design.

10. Collins, S., Ruina, A., Tedrake, R., & Wisse, M. (2005). "Efficient Bipedal Robots Based on Passive-Dynamic Walkers." Science, 307(5712), 1082-1085.
    - Demonstrates energy-efficient walking by exploiting natural dynamics.

### Terrain Adaptation and Robust Walking

11. Deits, R., & Tedrake, R. (2014). "Footstep Planning on Uneven Terrain with Mixed-Integer Convex Optimization." Proceedings of IEEE-RAS International Conference on Humanoid Robots.
    - Optimization-based footstep planning for complex terrain.

12. Kuindersma, S., Deits, R., Fallon, M., Valenzuela, A., Dai, H., Permenter, F., Koolen, T., Marion, P., & Tedrake, R. (2016). "Optimization-based Locomotion Planning, Estimation, and Control Design for the Atlas Humanoid Robot." Autonomous Robots, 40(3), 429-455.
    - Complete walking system for DARPA Robotics Challenge with terrain adaptation.

### Practical Implementation

13. Englsberger, J., Ott, C., & Albu-Schäffer, A. (2015). "Three-Dimensional Bipedal Walking Control Based on Divergent Component of Motion." IEEE Transactions on Robotics, 31(2), 355-368.
    - Practical control framework based on Capture Point (Divergent Component of Motion).

14. Hopkins, M., Hong, D., & Leonessa, A. (2015). "Compliant Locomotion Using Whole-Body Control and Divergent Component of Motion Tracking." Proceedings of IEEE International Conference on Robotics and Automation.
    - Incorporating compliance and disturbance rejection into walking control.

### Software and Tools

15. OpenHRP3 Documentation: https://fkanehiro.github.io/openhrp3-doc/en/
    - Open-source humanoid robotics platform including walking pattern generation tools.

16. TOWR (Trajectory Optimizer for Walking Robots): https://github.com/ethz-adrl/towr
    - Modern optimization-based locomotion library with clear documentation.

### Online Courses and Lectures

17. Underactuated Robotics (MIT 6.832): http://underactuated.mit.edu/
    - Course by Russ Tedrake covering dynamics, control, and planning for underactuated systems including bipeds.

18. Bipedal Locomotion lectures from CMU Robotics Institute: Available on YouTube and course websites.
    - Comprehensive lecture series covering theory and implementation.

## Looking Ahead

Having mastered bipedal locomotion, we now turn our attention to the upper body: manipulation and grasping. Chapter 13 explores how humanoid robots use their hands and arms to interact with objects in their environment, complementing their walking capabilities with dexterous manipulation.

Manipulation builds on the kinematic and dynamic foundations established in Chapter 11, applying inverse kinematics to reach targets and dynamic models to predict manipulation forces. Just as walking required managing the center of mass for balance, manipulation requires managing contact forces and grasp stability to prevent objects from slipping or being damaged.

The challenges of manipulation parallel those of walking in interesting ways. Walking must maintain balance while moving—an unstable equilibrium actively controlled. Grasping must maintain force closure while manipulating—preventing object motion through carefully coordinated finger forces. Both domains require managing underactuation: walking through limited foot contact, manipulation through limited fingers relative to object degrees of freedom.

The Zero Moment Point concept in walking finds its analog in grasp stability metrics. Just as ZMP must remain within the support polygon, contact forces must remain within friction cones to prevent slipping. The hierarchical control structures developed for walking—primary balance objectives with secondary motion goals—appear in manipulation as primary grasp maintenance with secondary object manipulation.

Model Predictive Control, which optimized walking trajectories while respecting constraints, extends naturally to manipulation. MPC can plan reach-and-grasp motions that avoid obstacles, respect joint limits, and optimize manipulability. The real-time optimization techniques and computational efficiency considerations carry directly forward.

Beyond individual skills, integrating locomotion and manipulation enables complex behaviors. A humanoid might walk to a location, reach for an object while maintaining balance, grasp it, and carry it while walking. Understanding both domains and their interaction becomes essential for capable humanoid systems.

Chapter 13 will explore hand design principles, grasp taxonomies, force closure theory, and motion planning for manipulation. The mathematical tools and algorithmic approaches developed for walking will prove valuable as we tackle the complementary challenge of dexterous object interaction.

