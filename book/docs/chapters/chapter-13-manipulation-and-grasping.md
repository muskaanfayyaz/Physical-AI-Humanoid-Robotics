# Chapter 13: Manipulation and Grasping

## Introduction

The ability to grasp and manipulate objects distinguishes truly capable robots from those limited to observation and locomotion. A humanoid that can walk but cannot pick up objects, open doors, or use tools remains severely limited in its utility. Manipulation transforms passive observers into active participants in the physical world, enabling robots to modify their environment and accomplish complex tasks.

Grasping presents unique challenges that differ fundamentally from locomotion. While walking involves intermittent contact with predictable surfaces (the ground), manipulation requires establishing and maintaining stable contact with diverse objects of varying shapes, sizes, materials, and weights. A hand must apply sufficient force to prevent slipping while avoiding crushing fragile items. Fingers must coordinate precisely to maintain force closure while adapting to object geometry.

Anthropomorphic hands, inspired by human hand anatomy, provide remarkable versatility through many degrees of freedom and distributed tactile sensing. However, this versatility comes with control complexity: coordinating 15-20 joints to perform smooth, stable grasps requires sophisticated planning and feedback. Understanding grasp stability, force distribution, and tactile feedback enables effective use of these capable manipulators.

This chapter explores the principles and techniques underlying robotic manipulation and grasping. We begin with anthropomorphic hand design, examining the mechanical structures that enable dexterous manipulation. Grasp taxonomies categorize the wide variety of hand-object interactions humans naturally employ. The mathematics of force closure provides rigorous criteria for grasp stability. We investigate grasp planning algorithms that select finger placements and contact forces, explore in-hand manipulation for reorienting grasped objects, and examine bi-manual coordination where two arms work together.

By understanding these concepts, you will grasp (pun intended) how humanoid robots achieve dexterous manipulation, why certain grasps succeed while others fail, and how motion planning, force control, and tactile feedback integrate to enable robust object interaction in unstructured environments.

## Core Concepts

### Anthropomorphic Hand Design Principles

Human hands achieve remarkable dexterity through a sophisticated mechanical design: 27 bones, over 30 joints, numerous muscles and tendons, and rich tactile sensing across the palm and fingertips. Anthropomorphic robotic hands seek to replicate this capability through similar kinematic structures and actuation strategies.

The typical humanoid hand features four fingers and an opposable thumb, arranged to provide both power grasps (full-hand grips on large objects) and precision grasps (fingertip control of small objects). Each finger generally contains three joints: the metacarpophalangeal (MCP) joint at the base, proximal interphalangeal (PIP) joint in the middle, and distal interphalangeal (DIP) joint near the fingertip. The thumb includes similar joints but with different orientations enabling opposition.

Underactuation plays a central role in practical hand designs. Fully actuating every joint would require one motor per joint, creating hands too heavy and complex for most applications. Instead, clever mechanical coupling allows fewer motors to drive multiple joints. For instance, tendons routed through multiple finger segments distribute one motor's torque across several joints, with springs providing passive compliance.

The Shadow Dexterous Hand exemplifies high-fidelity anthropomorphic design with 20 degrees of freedom and individual actuation of most joints. Air muscles provide compliant actuation resembling biological muscle. Rich tactile sensing covers the palm and fingertips. This design prioritizes capability over simplicity, suitable for research exploring the limits of dexterous manipulation.

Alternatively, the Barrett Hand demonstrates effective underactuated design with only four motors controlling eight degrees of freedom. Three fingers with two joints each use differential mechanisms that automatically adapt finger posture to object shape. This design prioritizes robustness and practical grasping capability, suitable for industrial and service applications.

The choice between full actuation and underactuation involves fundamental trade-offs. Full actuation provides precise control of each joint, enabling complex manipulations but requiring more motors, power, and control complexity. Underactuation achieves robust grasping with fewer resources by exploiting passive adaptation but limits control authority for complex in-hand manipulation.

Material selection significantly impacts hand performance. Rigid links provide precise position control but can damage objects during contact. Compliant materials absorb impact and conform to object geometry but complicate position control. Hybrid designs use rigid structures for primary kinematics with compliant coverings on contact surfaces, balancing control precision with contact safety.

Sensor integration enables feedback-driven grasping. Tactile sensors at fingertips detect contact forces and slip. Joint encoders measure finger configurations. Force/torque sensors at the wrist measure overall hand loading. Integrating these sensing modalities provides comprehensive awareness of hand-object interaction essential for stable manipulation.

### Grasp Taxonomies

Human grasping exhibits remarkable diversity: we use vastly different hand configurations when holding a pencil, carrying a briefcase, opening a jar, or handling fragile objects. Categorizing this diversity helps organize understanding and guide robotic grasp selection.

The Cutkosky taxonomy, developed through observation of human manufacturing tasks, categorizes grasps into power grasps, precision grasps, and intermediate types. Power grasps use the entire hand including palm to firmly secure objects, prioritizing stability over fine control. Precision grasps use fingertips without palm contact, prioritizing dexterity over maximum force.

Power grasps include the cylindrical grasp (wrapping fingers around a cylinder), spherical grasp (fingers distributed around a ball), and hook grasp (fingers curled to support object weight). These grasps generate large forces and resist disturbances well but offer limited ability to manipulate the grasped object.

Precision grasps include the pinch grasp (thumb opposing one or two fingers), tripod grasp (thumb, index, and middle finger forming a triangle), and lateral grasp (thumb pressing against the side of the index finger). These grasps enable fine position and orientation control but generate less force and are more sensitive to disturbances.

The Feix taxonomy extends this classification, identifying 33 distinct grasp types based on detailed analysis of human grasping. While comprehensive, this detailed taxonomy can overwhelm practical robot implementation. Most robotic systems focus on a subset of fundamental grasp types applicable to common manipulation tasks.

Beyond geometric classification, grasps can be categorized by stability properties. Form-closure grasps constrain all object motions through purely geometric contact, independent of friction. These represent the ideal stable grasp but require complex finger positioning. Force-closure grasps prevent object motion through appropriate contact forces, relying on friction. Most practical robotic grasps achieve force closure rather than form closure.

Enveloping versus fingertip grasps represent another important distinction. Enveloping grasps use large contact areas including palm and multiple finger segments, distributing forces widely. Fingertip grasps use small contact regions at fingertips, providing better control but concentrating forces. The choice depends on object fragility, required manipulation precision, and grasp stability requirements.

### Force Closure and Grasp Stability

A fundamental question underlies all grasping: will the hand maintain secure contact with the object under expected disturbances? Force closure provides the mathematical framework for answering this question rigorously.

A grasp achieves force closure if the contact forces and torques can resist arbitrary external forces and torques on the object. More precisely, the contacts must be able to generate any wrench (combined force and torque vector) within some region around zero. This ensures the hand can counteract disturbances from any direction.

The condition for force closure depends on contact models. Point contacts with friction can exert forces within a friction cone: the contact force must have a component normal to the surface (pushing, not pulling) and tangential components limited by the coefficient of friction. For a friction coefficient mu, the tangential force magnitude must not exceed mu times the normal force magnitude.

With k contact points, each can exert forces within its friction cone. The grasp achieves force closure if the union of all possible contact force combinations can generate any net wrench on the object. Mathematically, this requires the grasp matrix (mapping contact forces to object wrenches) to have certain properties related to its rank and positive spanning.

For planar grasping (2D), at least three contacts with friction are necessary for force closure, arranged so their normals don't intersect at a common point. For spatial grasping (3D), at least four contacts with friction are generally necessary, with geometric conditions ensuring they span the wrench space.

Form closure represents a special case where contact geometry alone prevents object motion, without requiring friction. This requires more contacts (at least seven for general 3D objects with point contacts) but provides grasp stability independent of friction coefficient. Form closure is difficult to achieve with typical robotic hands but represents the theoretical ideal.

Grasp stability also depends on object dynamics. A static analysis considers force balance under constant external forces. A dynamic analysis includes inertial effects from object acceleration and impact forces during manipulation. Dynamic stability is generally more challenging, requiring larger safety margins in contact forces.

The concept of grasp equilibrium provides another perspective. A grasped object is in equilibrium if the sum of contact forces and external forces (including gravity) equals zero, and the sum of contact torques and external torques equals zero. Force closure ensures that for any external wrench, contact forces can be found that restore equilibrium.

### Grasp Quality Metrics

While force closure provides a binary yes/no answer to grasp stability, quality metrics quantify how good a force-closure grasp is. Better grasps resist larger disturbances, require less contact force, or provide more manipulation capability.

The epsilon quality metric measures the largest disturbance wrench the grasp can resist before failure. Compute the minimum over all directions of the maximum disturbance magnitude the grasp can counteract. Larger epsilon indicates better grasp quality. This metric directly relates to robustness: higher quality grasps tolerate larger disturbances without losing contact.

Computing epsilon quality requires optimization. For each potential disturbance direction, solve for the maximum disturbance magnitude such that contact forces remain within their friction cones while balancing the disturbance. The minimum across all directions gives epsilon. This computation can be expensive but provides meaningful physical interpretation.

The Ferrari-Canny metric takes a related approach based on the grasp wrench space (the set of all wrenches the grasp can apply to the object). The metric equals the radius of the largest ball centered at the origin that fits entirely within the grasp wrench space. Larger radii indicate the grasp can exert larger forces uniformly in all directions.

Volume-based metrics measure the volume of the grasp wrench space, with larger volumes indicating greater overall capability. These metrics are easier to compute than epsilon or Ferrari-Canny but don't directly measure worst-case performance in the weakest direction.

Manipulability measures how easily the grasped object can be moved and reoriented. Analogous to manipulability for robot arms, grasp manipulability depends on the grasp Jacobian relating finger velocities to object velocities. Well-conditioned Jacobians enable easy object manipulation; poorly conditioned ones create directions where object motion is difficult.

Task-specific metrics evaluate grasps relative to particular manipulation goals. A grasp for carrying a heavy object should maximize force capability in the vertical direction. A grasp for precise insertion should maximize orientation control about certain axes. Optimizing task-specific metrics produces grasps well-suited to their intended use.

Multiple metrics can be combined to balance different objectives. A weighted sum might include stability (epsilon quality), efficiency (required contact forces), and manipulability. Multi-objective optimization can identify Pareto-optimal grasps that cannot be improved in one metric without degrading another.

### Tactile Sensing and Feedback

Vision provides crucial information for identifying objects and planning grasps, but tactile feedback becomes essential once contact begins. Touch reveals contact forces, object surface properties, slip detection, and shape information not accessible to vision (occluded surfaces, transparent objects, fine texture).

Tactile sensor technologies vary widely. Force-sensing resistors (FSRs) change resistance under applied force, providing simple, inexpensive force measurement but with limited spatial resolution. Capacitive sensors detect changes in capacitance from deformation, enabling high spatial resolution in slim packages. Optical tactile sensors use cameras to observe deformation of compliant surfaces, achieving very high resolution but requiring more space and processing.

BioTac sensors, inspired by human fingertips, combine multiple sensing modalities in a single package: force-sensitive electrodes detect contact forces and vibrations, a pressure sensor measures overall loading, and temperature sensors detect heat transfer. This multi-modal approach provides rich tactile information analogous to human touch.

Spatial resolution determines what features can be detected through touch. High-resolution sensor arrays (spacing below 1mm) can detect fine textures and small objects. Lower resolution sensors (5-10mm spacing) suffice for grasp stability monitoring but miss fine details. Trade-offs involve cost, wiring complexity, and data processing requirements.

Slip detection enables reactive grasp adjustment. When an object begins slipping, tangential forces at the contact change and vibrations occur. High-frequency tactile sensing (hundreds to thousands of Hz) can detect these signatures. Upon detecting slip, the controller increases grip force to restore stability before the object fully escapes.

Force control uses tactile feedback to regulate contact forces. The controller measures actual contact forces and compares them to desired values, adjusting motor commands to minimize error. This enables gentle grasping of fragile objects (minimal force) and firm grasping of heavy objects (sufficient force to prevent slip).

Tactile servoing uses tactile feedback to guide manipulation. Just as visual servoing uses vision to control motion, tactile servoing uses touch. For example, sliding a grasped object along a surface while maintaining constant contact force, or exploring object shape by moving fingers along its surface while monitoring contact.

Multi-modal integration combines vision and touch. Vision initially guides the hand toward the object. When contact begins, tactile feedback takes over for fine adjustment. During manipulation, vision tracks object motion while touch monitors contact stability. This sensory fusion provides robust perception despite individual sensor limitations.

### Grasp Planning Algorithms

Selecting where to place fingers and how much force to apply constitutes the grasp planning problem. Given an object model (shape, mass, friction), find contact locations and forces that achieve force closure, optimize quality metrics, and are kinematically reachable.

Analytical grasp planning uses geometric reasoning to identify candidate grasps. For simple shapes (cylinders, boxes, spheres), geometric rules directly specify good contact locations. A cylindrical object: place fingers around the circumference equidistant from each other and from the cylinder axis. A box: contact flat faces opposing each other. These heuristics work well for common shapes but don't extend to arbitrary complex objects.

Sampling-based grasp planning generates many candidate grasps by randomly sampling contact locations on the object surface, then evaluates each grasp's quality. High-quality grasps are retained; poor grasps are discarded. Sampling continues until sufficient good grasps are found or time limits are reached.

The sampling process typically proceeds as follows: randomly select a point on the object surface as the first contact, select subsequent contacts satisfying geometric constraints (appropriate distance from first contact, contact normals pointing toward object interior), check force closure, compute quality metrics, and store the grasp if quality exceeds a threshold.

Thousands or millions of candidate grasps might be evaluated in offline planning. Fast quality metric computation becomes essential. Approximations and bounds can quickly eliminate obviously poor grasps before expensive exact evaluation.

Simulation-based approaches test grasps in physics simulators. Candidate grasps are applied to simulated objects, disturbances are applied, and stability is observed. Grasps that maintain stability under large disturbances are preferred. This approach naturally accounts for dynamics but requires accurate simulation and significant computation.

Learning-based grasp planning uses machine learning to predict grasp success from object features. Training datasets contain objects with labeled successful and failed grasps. Neural networks learn to map object representations (point clouds, images) to grasp quality predictions. At test time, the network evaluates candidate grasps without explicit geometric analysis.

Dexterous grasping planners must consider hand kinematics. A grasp might satisfy force closure and achieve high quality metrics but be unreachable due to kinematic constraints or collisions. The planner must verify that inverse kinematics can find a collision-free hand configuration achieving the desired contacts.

Grasp planning for complex hands with many degrees of freedom creates high-dimensional search spaces. Optimization-based approaches formulate grasp planning as a constrained optimization problem: maximize quality subject to force closure, kinematic reachability, and collision avoidance. Gradient-based optimization or derivative-free methods search this space for good solutions.

### In-Hand Manipulation

Once an object is grasped, repositioning or reorienting it without releasing and re-grasping constitutes in-hand manipulation. This advanced capability enables assembling parts, adjusting tool pose, and adapting to changing task requirements without interrupting the manipulation sequence.

Finger gaiting involves sequentially breaking and making contacts to walk fingers around an object. One or more fingers release contact, move to new positions, and re-establish contact while remaining fingers maintain grasp stability. By repeating this process, the hand can significantly reorient the object.

The challenge in finger gaiting lies in maintaining force closure throughout the motion sequence. When some fingers release, the remaining fingers must still achieve force closure. Planning finger gaiting requires identifying sequences of intermediate grasps that form a path through the space of stable grasps from initial to final configuration.

Rolling contacts allow object rotation without sliding. Fingers maintain contact while the object rotates against them, like a ball rolling on a surface. The no-slip constraint couples object rotation to finger motion. Coordinating multiple rolling contacts enables controlled object reorientation.

The kinematics of rolling contact are more complex than point contact. The contact point on the finger surface changes as the object rolls, and the contact normal direction changes. Forward kinematics must account for these geometric changes. Control must coordinate finger motions to achieve desired object rotation while maintaining rolling without slip.

Pivoting uses gravity or contact forces to rotate an object about one contact point. For example, grasping a long object at one end and tilting it to rotate about the grasp point. Pivoting is mechanically simple but limits reorientation to specific axes and requires managing dynamic effects.

Pushing and sliding intentionally allow controlled slip to reposition objects. Rather than preventing all slip through force closure, the controller permits and exploits slip in controlled directions. This can be more efficient than maintaining rigid grasps but requires careful force regulation and slip monitoring through tactile feedback.

Coordinating multiple fingers for in-hand manipulation requires solving a coupled control problem. Each finger's motion affects the object pose, and desired object motion must be distributed among fingers. The manipulation Jacobian relates finger velocities to object velocities, analogous to the robot arm Jacobian relating joint velocities to end-effector velocities.

### Bi-Manual Coordination

Many tasks benefit from or require using both hands: carrying large objects, assembly operations with one hand positioning and one hand fastening, and cooperative manipulation where hands work together. Bi-manual coordination extends single-hand manipulation with additional complexity from coordinating two kinematic chains.

Task decomposition separates bi-manual tasks into roles for each arm. Symmetric tasks divide the task evenly (both hands lifting opposite ends of a table). Asymmetric tasks assign different roles (one hand holds a jar while the other opens the lid). Identifying appropriate decomposition simplifies planning and control.

Relative motion matters more than absolute motion in many bi-manual tasks. When two hands carry an object together, the object pose depends on the relative positions and orientations of the hands, not their absolute positions. Control formulations using relative coordinates naturally capture these constraints.

The bi-manual manipulation Jacobian maps joint velocities of both arms to object velocity. It combines the individual arm Jacobians accounting for how each hand's motion contributes to object motion. Coordinated control uses this combined Jacobian to achieve desired object velocities through appropriate joint commands to both arms.

Force distribution between hands requires careful consideration. When both hands grasp an object, internal forces can occur: the hands push against each other through the object without changing the object's net force or torque. These internal forces should be minimized for efficiency but sufficient to maintain grasp stability.

The grasp force optimization problem for bi-manual manipulation seeks to minimize internal forces while ensuring force closure and satisfying task requirements. The solution typically has a null space: many force distributions achieve the same net object wrench. Optimization selects among these based on secondary criteria like minimizing total force or balancing load between hands.

Bi-manual assembly operations require precise coordination. Peg-in-hole insertion with two hands might use one hand to position the peg and the other to provide stabilizing forces or guide the base. Success requires aligning multiple degrees of freedom within tight tolerances while managing contact forces.

Cooperative manipulation where two robots (or one dual-arm robot) work together extends bi-manual concepts. The robots must communicate their states and intentions, coordinate their motions through shared world models, and distribute tasks effectively. Decentralized control allows each robot to respond quickly to local information while coordination protocols ensure coherent joint behavior.

### Dexterous Manipulation Challenges

Despite advances in hand design, planning algorithms, and control techniques, dexterous manipulation remains significantly more challenging than human manipulation suggests it should be. Understanding these challenges guides research priorities and practical system design.

Modeling uncertainty limits grasp planning accuracy. Object properties (shape, mass distribution, friction coefficient, compliance) are never perfectly known. Small errors can cause planned force-closure grasps to fail or require more grip force than expected. Robust planning must account for uncertainty through conservative force margins or adaptive approaches that adjust based on feedback.

Contact dynamics present both theoretical and practical difficulties. Analytical contact models make simplifying assumptions (rigid bodies, Coulomb friction, point contacts) that real contacts violate. Actual contacts involve deformation, complex friction behavior, and distributed contact patches. Simulation struggles to accurately predict contact behavior, and control must handle this model mismatch.

High-dimensional control spaces challenge real-time manipulation control. A hand with 20 degrees of freedom controlled at 100 Hz generates 2000 control outputs per second, each potentially affecting grasp stability. Computing optimal or even feasible controls in real-time requires efficient algorithms and simplified models.

Perception limitations hinder grasp planning and execution. Vision occlusions hide contact surfaces. Tactile sensing provides only local information at contacts. Estimating object pose and properties from limited observations introduces uncertainty. Active perception strategies that move sensors to gain better views can help but require coordination with manipulation goals.

Generalization across object classes remains difficult. Humans effortlessly transfer manipulation skills from one object to novel similar objects. Robots typically require explicit planning for each object or extensive training data. Learning representations that capture manipulation-relevant object properties and enable effective generalization is an active research area.

Real-time adaptation distinguishes robust from brittle manipulation. Disturbances, modeling errors, and unexpected events constantly arise. Systems that can detect problems (through tactile and visual feedback), diagnose causes, and adjust plans or control strategies online exhibit far better practical performance than purely feedforward systems.

## Practical Understanding

### Hand Design Trade-offs in Practice

Designing a robotic hand requires balancing numerous competing objectives. High dexterity demands many actuated degrees of freedom, but each additional motor adds weight, cost, and control complexity. Underactuation reduces these burdens but limits manipulation capability.

The finger length ratio affects grasp capability. Longer fingers reach around larger objects and achieve better form closure on irregular shapes. Shorter fingers are lighter, faster, and fit in tighter spaces. Human finger proportions provide a reasonable default, but task-specific optimization might suggest different ratios.

Thumb opposition angle determines the types of grasps possible. A thumb opposing at 90 degrees to the fingers enables strong pinch grasps and power grasps. Different angles optimize different grasp types. Some designs include an additional thumb degree of freedom to adjust opposition angle based on the task.

Actuator placement involves choosing between motors in the hand (direct drive) or in the forearm with tendon transmission. Hand-mounted motors simplify kinematics but add weight at the end of the arm, reducing payload capacity and increasing inertia. Forearm motors with tendons reduce hand weight but introduce compliance, friction, and tendon routing complexity.

Tendon transmission provides smooth, compliant force transmission and enables underactuation through differential mechanisms. However, tendons stretch under load, creating position errors, and can break or slip. Routing tendons through multiple joints while avoiding interference with other components requires careful mechanical design.

Linkage-based underactuation uses mechanical linkages to couple multiple joints. As one motor drives the linkage, multiple joints move in coordinated patterns determined by the linkage geometry. Different linkage designs produce different coupling behaviors. Adjusting linkage parameters tunes the hand's automatic adaptation to object shape.

Compliant fingertips improve grasping robustness. Soft materials conform to object surfaces, increasing contact area and friction. They absorb position errors and impact forces during contact. However, compliance complicates position control and force sensing. Hybrid designs use compliant coverings over rigid cores, balancing these trade-offs.

### Computing Force Closure

Verifying force closure for a proposed grasp requires checking whether contact forces can balance arbitrary external wrenches. This involves geometric and linear algebra computations on the grasp configuration.

The grasp matrix G maps contact forces to object wrenches. Each column of G corresponds to one contact and describes the wrench that contact exerts per unit contact force. For a 3D point contact with friction at position p with contact normal n, the column includes the force components (within the friction cone) and torques (from p × force).

Constructing the grasp matrix proceeds systematically:

1. For each contact i, determine its position p_i and contact normal n_i
2. Define the friction cone based on friction coefficient mu
3. Represent forces within the cone using a basis (e.g., normal direction plus tangential directions)
4. Compute the wrench each basis force generates: [force; p × force]
5. Assemble all basis wrenches into columns of G

The grasp achieves force closure if the origin lies in the interior of the convex hull of G's columns (and their negatives, since contacts can push in any direction within their friction cone). Checking this condition involves solving a linear programming problem or using computational geometry algorithms.

A simpler necessary condition checks G's rank: for 3D objects, G must have full rank 6, indicating the contacts span the 6D wrench space. This is necessary but not sufficient for force closure—the contacts must also satisfy geometric conditions ensuring positive combinations can generate any wrench.

For planar grasping (2D objects), the grasp matrix is 3×n for n contacts (force in x, y and torque about z). Force closure requires rank 3 and geometric conditions on contact locations. A common rule: three or more contacts with friction coefficients above a minimum threshold, with contact normals not intersecting at a common point.

Practically implementing force closure checking requires numerical stability considerations. Near-grazing contacts (nearly tangent to object surface) create nearly-degenerate grasp matrices. Small numerical errors can incorrectly classify nearly-force-closure grasps. Robust implementations use tolerances and condition number checks.

### Grasp Quality Evaluation

Once force closure is verified, quantifying grasp quality enables comparing alternative grasps. Computing quality metrics involves optimization over contact forces and disturbance wrenches.

For epsilon quality, the algorithm iterates over many disturbance directions (sampled uniformly on the unit sphere in wrench space). For each direction, solve an optimization problem:

```
maximize: magnitude
subject to:
  - Contact forces balance disturbance: G * f = magnitude * direction
  - Contact forces within friction cones: |f_tangential| ≤ mu * f_normal
  - Contact forces non-negative normal components: f_normal ≥ 0
```

The minimum magnitude across all directions gives epsilon. Larger epsilon indicates the grasp resists larger disturbances.

Computing Ferrari-Canny quality requires finding the largest ball centered at the origin within the grasp wrench space. The radius equals the minimum distance from the origin to the boundary of the convex hull of achievable wrenches. This can be computed by solving linear programs for many wrench directions, finding which boundary is closest.

Volume-based metrics integrate over the grasp wrench space, computing its volume. For low-dimensional spaces, numerical integration or Monte Carlo sampling estimates the volume. Alternatively, convex hull algorithms can compute exact volumes.

Manipulability for grasping uses the grasp Jacobian J_g relating finger velocities to object velocities. The manipulability measure is sqrt(det(J_g * J_g^T)), analogous to robot arm manipulability. Large values indicate the grasped object can be moved easily in all directions.

Weighted combinations of metrics allow multi-criteria optimization:

```
quality = w1 * epsilon + w2 * manipulability - w3 * force_magnitude
```

Weights encode task priorities. Weights should be normalized to account for different metric scales. Multi-objective optimization can identify the Pareto frontier of grasps, presenting trade-offs for designer selection.

### Implementing Grasp Planning

A practical grasp planning system integrates object perception, candidate generation, evaluation, and execution. The pipeline typically follows these stages:

Object segmentation and pose estimation uses vision to identify the object to be grasped and estimate its 3D pose. Point clouds from depth cameras or stereo vision provide geometric information. Object recognition identifies the object class, retrieving shape models from a database.

Candidate grasp generation samples many possible grasps. For database approaches, retrieve pre-computed grasps for this object class and transform them to the current object pose. For sampling approaches, randomly select contact points on the object surface, construct hand configurations achieving these contacts, and check collisions.

A typical sampling iteration:

1. Sample a point on the object surface as the first contact
2. Sample the hand approach direction (often along the surface normal)
3. Sample hand orientation about the approach direction
4. Position the hand to achieve the first contact
5. Close fingers according to the hand's underactuation or coupling
6. Record final finger positions as contact points
7. Check for collisions between hand and object or environment

Grasp evaluation computes quality metrics for each candidate. This is the computational bottleneck; evaluating thousands of candidates requires efficient implementation. Parallelization across CPU cores or GPU acceleration can dramatically speed evaluation.

Filtering removes infeasible grasps: those with insufficient quality, kinematic unreachability, or collisions. A hierarchical filtering approach first applies fast approximate checks to eliminate obviously bad grasps, then applies expensive exact checks only to promising candidates.

Ranking orders the remaining grasps by quality. The highest-quality grasp is selected for execution. Some systems maintain a ranked list and attempt grasps in order if the first fails.

Execution planning computes a motion plan from the current arm configuration to the pre-grasp pose (hand positioned near the object but not contacting), then to the grasp pose (fingers achieving contact). Motion planning uses techniques covered in Chapter 14 to avoid obstacles.

Grasp execution follows the planned motion until contact. Force/torque sensing or tactile feedback detects contact. The controller transitions from position control to force control, applying desired grip forces. Feedback monitors contact stability, adjusting forces if slip is detected.

### Force Control for Manipulation

Once a grasp is established, maintaining appropriate contact forces ensures stability without damaging objects. Force control adjusts motor commands based on sensed forces, closing the feedback loop around force rather than position.

Impedance control specifies a dynamic relationship between force and position: F = K * (x - x_d), where K is stiffness. The controller acts like a spring connecting the actual position x to the desired position x_d. When external forces push the hand, it complies according to the stiffness. High stiffness resists disturbances firmly; low stiffness provides compliant behavior.

Implementing impedance control requires force sensing and position control. Measure the contact force F. Compute the desired position x_d based on the commanded force and stiffness: x_d = x - K^(-1) * F. Command the position controller to move to x_d. As forces change, x_d adjusts, creating the desired force-position relationship.

Hybrid position/force control separates task space into position-controlled and force-controlled directions. For example, when grasping a box: control position in the direction parallel to the surface (sliding along it) and control force in the normal direction (pressing against it). The controller switches between position and force control based on the task space direction.

The selection matrix S determines which directions use force control (S_f) and which use position control (S_p). Typically S_f and S_p are diagonal matrices with ones in controlled directions and zeros elsewhere, and S_p + S_f = I.

Grasp force optimization distributes contact forces among multiple fingers to achieve desired net object wrench while minimizing effort. The optimization problem:

```
minimize: sum of squared contact forces
subject to:
  - Net object wrench equals desired: G * f = w_desired
  - Forces within friction cones
  - Non-negative normal forces
```

The solution balances load across fingers and uses friction efficiently. For redundant grasps (more contacts than necessary), the null space allows adjusting internal forces without changing net wrench.

Tactile servoing uses tactile feedback for fine manipulation. The controller monitors contact force magnitudes and locations, adjusting finger positions to achieve desired contact patterns. This enables tasks like sliding along an edge (maintaining constant edge contact) or surface exploration (scanning fingers across a surface).

### In-Hand Manipulation Implementation

Repositioning a grasped object without releasing it requires coordinated finger motion and careful force management. The implementation combines planning (identifying intermediate configurations) and control (executing motion while maintaining stability).

Finger gaiting plans a sequence of intermediate grasps. Each intermediate grasp must satisfy force closure with a subset of fingers while others reposition. The planner searches the graph of stable grasps, finding a path from initial to goal configuration.

Graph construction represents grasps as nodes and single-finger motions as edges. A grasp is a node if it achieves force closure. An edge exists between two grasps if one finger can move from one configuration to another while the remaining fingers maintain force closure. Graph search algorithms (A*, Dijkstra) find shortest paths through this graph.

Executing finger gaiting follows the planned sequence. At each step:

1. Verify remaining fingers achieve force closure
2. Increase forces at stable contacts to provide larger safety margin
3. Command one finger to release (reduce force to zero)
4. Move released finger to new position
5. Establish new contact and increase force
6. Decrease forces at remaining contacts to nominal levels

Force sensing and tactile feedback monitor stability throughout. If slip is detected during a transition, abort the current step and increase forces at remaining contacts to restore stability.

Rolling manipulation requires coordinating finger velocities to achieve desired object rotation without slip. The rolling constraint couples object angular velocity omega to finger velocities: v_finger = omega × r, where r is the vector from the rotation axis to the contact point.

Computing required finger velocities:

1. Specify desired object angular velocity omega
2. For each finger, compute the contact point's velocity from the rolling constraint
3. Use the hand's inverse kinematics to find joint velocities achieving these finger velocities
4. Send joint velocity commands to the motors

Monitoring actual contact motion through tactile sensing detects slip violations. If actual motion deviates from the rolling constraint (slip occurs), adjust finger forces to increase friction or modify the rotation rate.

### Bi-Manual Grasp Planning and Control

Coordinating two hands to manipulate a single object extends single-hand techniques but introduces coupling between the arms. Both planning and control must account for this coupling.

Object modeling for bi-manual grasps includes attachment points for both hands. The relative configuration of hands determines object pose. If hand 1 is at T_1 and hand 2 at T_2, and the object-to-hand transforms are T_o1 and T_o2, then the object pose is T_obj = T_1 * T_o1 = T_2 * T_o2. Consistency requires these to match.

Grasp planning for bi-manual manipulation selects contact points for both hands simultaneously. The combined grasp must achieve force closure considering all contacts from both hands. Quality metrics evaluate the entire bi-manual grasp as a single system.

Sampling-based planning generates candidate bi-manual grasps by:

1. Sample grasp for hand 1 on object
2. Sample grasp for hand 2 on object
3. Check geometric compatibility: hands don't collide with each other
4. Verify combined force closure from all contacts
5. Compute quality metrics for the combined grasp
6. Check inverse kinematics feasibility for both arms

Coordinated motion planning computes trajectories for both arms that avoid collisions with each other, the object, and the environment. The combined configuration space has dimension n_left + n_right (sum of both arm DOFs). Motion planners search this high-dimensional space for collision-free paths.

Prioritization simplifies coordination: designate one arm as primary (performs the main task) and the other as secondary (assists). Plan the primary arm's motion first, treating it as an additional obstacle for secondary arm planning. This reduces complexity but may miss solutions requiring true coordination.

Controlling bi-manual manipulation uses the combined manipulation Jacobian J_bi = [J_left | J_right], mapping both arms' joint velocities to object velocity. Desired object velocity v_obj is achieved by joint velocities q_dot = J_bi^+ * v_obj, where J_bi^+ is the pseudo-inverse.

Force distribution optimizes internal forces:

```
minimize: |f_internal|^2
subject to:
  - Net object wrench: f_left + f_right = f_desired
  - Force closure at each hand
  - Force limits at each contact
```

The solution minimizes squeezing forces between hands while maintaining grasp stability and achieving desired object wrench.

### Motion Planning with MoveIt 2

MoveIt 2 provides a comprehensive framework for manipulation motion planning, integrating perception, planning algorithms, control, and execution. Understanding its conceptual architecture enables effective use for humanoid manipulation.

The planning scene represents the world model: robot configuration, object locations, and collision geometry. Perception updates the planning scene from sensor data (cameras, depth sensors). Octomap representations efficiently encode 3D occupied space from point clouds.

Planning requests specify motion goals: target end-effector pose, joint configuration, or Cartesian path. Constraints include collision avoidance, joint limits, and task-specific requirements (maintain upright orientation, keep object level).

Motion planners search configuration space for collision-free paths. MoveIt 2 includes multiple planners: RRT (Rapidly-exploring Random Trees), RRT-Connect, PRM (Probabilistic Roadmaps), and OMPL (Open Motion Planning Library) variants. Different planners have different strengths for various problem characteristics.

RRT and variants grow trees from the start configuration, randomly sampling new configurations and connecting them if collision-free. RRT-Connect grows trees from both start and goal, connecting them when they meet. These planners work well for high-dimensional spaces and complex obstacles.

Trajectory processing smooths and optimizes planned paths. Initial paths from sampling-based planners often contain unnecessary waypoints and jerky motion. Time parameterization adds velocity profiles respecting speed and acceleration limits. Smoothing algorithms adjust waypoints to reduce path length and improve motion quality.

Grasping with MoveIt 2 uses the MoveIt Task Constructor framework. Task decomposition breaks manipulation into stages: approach, grasp, retreat, move, release. Each stage specifies constraints and goals. The framework searches for plans satisfying all stages, handling the coupling between stages (grasp pose affects retreat direction).

Pick and place pipelines in MoveIt 2:

1. Perceive object location and pose
2. Generate candidate grasps using grasp planning
3. For each grasp, plan approach (move arm to pre-grasp pose)
4. Plan grasp motion (close fingers to achieve contact)
5. Plan retreat (lift object)
6. Plan transport (move to place location)
7. Plan place (lower object)
8. Plan release (open fingers)
9. Execute highest-quality complete plan

Execution monitoring tracks plan progress and handles failures. If unexpected collisions occur or the object is not grasped successfully, the system can replan or try alternative grasps.

## Conceptual Diagrams

### Anthropomorphic Hand Structure

```
Side view of humanoid hand:

                    DIP (Distal Interphalangeal)
                    |
                    PIP (Proximal Interphalangeal)
                    |
                    MCP (Metacarpophalangeal)
                    |
        Fingertip---O
                    |
                ---O
                    |
                ---O
                    |
            =========== Palm

Four fingers (Index, Middle, Ring, Pinky): 3 joints each (MCP, PIP, DIP)
Thumb: 3 joints with different orientation (enables opposition)

Typical DOF count:
- Fully actuated: 15-20 DOF
- Underactuated: 4-8 actuators controlling 12-20 joints

Sensors:
- Joint encoders: measure finger positions
- Tactile: palm and fingertips (force, pressure, slip)
- F/T sensor: wrist (overall hand loading)
```

### Grasp Taxonomy

```
POWER GRASPS (whole hand + palm contact):

Cylindrical Grasp:       Spherical Grasp:
    ||||||                   ____
    ||obj||                 / obj \
    ||||||                  \ __  /
   =======palm             =======palm
   (wrap around cylinder)   (fingers distributed around sphere)


PRECISION GRASPS (fingertip contact, no palm):

Pinch Grasp:             Tripod Grasp:
   Thumb                    Thumb
     |  |Index                |  Index
     |  |                     | /
     [O]                      ||  Middle
    object                   [O]
  (thumb opposes            object
   index finger)        (three-finger grip)


INTERMEDIATE GRASPS:

Lateral Grasp:
   Thumb-||
        ||object  Index finger
   (thumb presses against side of index)
```

### Force Closure Concept

```
2D Example - Three-Finger Grasp:

         Finger 2
            |
            v
           [O]
          /   \
         v     v
    Finger 1  Finger 3

Each finger pushes within its friction cone:

    Normal force: perpendicular to surface
    Friction cone: angle depends on friction coefficient μ

         / | \  <-- Friction cone (angle = atan(μ))
        /  |  \
       /   v   \  Contact normal
      -----------  Object surface

FORCE CLOSURE CHECK:
1. Can contacts generate force in any direction? YES
2. Can contacts generate torque in any direction? YES
3. Forces within friction cones? YES

Result: Force closure achieved


FAILURE Example - Two-Finger Grasp (no friction):

    Finger 1    Finger 2
        |           |
        v           v
       [============]

Cannot resist horizontal forces → NOT force closure


Form Closure (geometry prevents motion):

    /|  Object  |\
   / |   ___    | \
  F1 |  |   |   | F2
     |  |___|   |
      \ |  | /
        \|_|/
         F3

Geometric constraint prevents motion even without friction.
```

### Grasp Quality Metrics

```
EPSILON QUALITY:

Disturbance Wrench Space (3D for planar, 6D for spatial):

         F_y
          ^
          |   Worst direction (smallest resistance)
          |  /
          | /  epsilon = min distance to failure
          |/
    ------O------> F_x
         /|
        / |
     Other directions (larger resistance)

Grasp can resist disturbances within epsilon ball.
Larger epsilon = better quality.


FERRARI-CANNY METRIC:

Grasp Wrench Space (wrenches grasp can exert):

           Wrench_y
               ^
              /|\
             / | \
            /  |  \   <-- Convex hull of achievable wrenches
           /   O   \      (O = origin)
          /    .    \
         /     .     \
        /      .radius\
       /              \
      -----------------> Wrench_x

Ferrari-Canny = radius of largest ball centered at origin
                inside wrench space
```

### Tactile Sensing Applications

```
SLIP DETECTION:

Time series of tactile sensor readings:

Force        /\    /\    /\   <-- Vibrations indicate slip
  ^         /  \  /  \  /  \
  |   _____/    \/    \/    \_____
  |________________________> time
      Contact   Slip starts    Stable again
      stable                   (increased force)

Response: Increase grip force upon detecting slip


FORCE CONTROL:

Desired force: F_d = 5N
Measured force: F_m (from tactile sensors)

Error: e = F_d - F_m

Control law: motor_command = motor_command + K_p * e

   Measured
     F_m
      ^     Target F_d
      |    -----------
      |   /
      |  /  (approaches target)
      | /
      |/____________> time


SHAPE EXPLORATION:

Move finger along surface, record tactile readings:

    Finger path
    ------>
    ┌─────┐  Tactile readings vary with surface geometry
    │     │
    │ Obj │  High pressure on flat surfaces
    │     │  Pressure changes at edges
    └─────┘

Build tactile map of object shape.
```

### Grasp Planning Pipeline

```
INPUT: Object point cloud + pose
   |
   v
+----------------------+
| Generate Candidates  |  Sample contact points
|                      |  Construct hand configs
+----------------------+
   |
   | (1000s of candidates)
   v
+----------------------+
| Filter: Fast Checks  |  Collision detection
|                      |  Basic reachability
+----------------------+
   |
   | (100s remaining)
   v
+----------------------+
| Evaluate Quality     |  Force closure
|                      |  Epsilon/Ferrari-Canny
|                      |  Manipulability
+----------------------+
   |
   | (10s of high-quality grasps)
   v
+----------------------+
| Rank and Select      |  Sort by quality
|                      |  Select best
+----------------------+
   |
   | (selected grasp)
   v
+----------------------+
| Motion Planning      |  Plan approach
|                      |  Plan grasp closure
+----------------------+
   |
   v
OUTPUT: Grasp + motion plan
```

### In-Hand Manipulation - Finger Gaiting

```
Reorient object by walking fingers:

Initial Grasp:           Intermediate:              Final Grasp:
   F1  F2  F3              F1     F3                 F1  F2  F3
    |   |   |              |      |                   |   |   |
    [=======]              [======]                   [=======]
                          F2 repositions              (rotated)
                            (moving)

Sequence:
1. F1, F2, F3 achieve force closure
2. Release F2, maintain force closure with F1, F3
3. Move F2 to new position
4. Re-establish contact with F2
5. Repeat with different fingers to continue rotation

Requirements:
- Remaining fingers maintain force closure during transitions
- Force margins prevent slip during reconfiguration
- Planned path through stable grasp configurations
```

### Bi-Manual Manipulation

```
Top view - Two hands grasping box:

    Left Hand              Right Hand
         |                     |
         v                     v
    +----|-------OBJECT--------|----+
    |    *                     *    |
    |                               |
    |          [CoM]                |
    |                               |
    +-------------------------------+

Forces:
- Each hand exerts grasp forces on object
- Internal forces: hands squeeze object
- External force: net force on object (lift, move)

Control objectives:
1. Net force: move object as desired
2. Internal force: maintain grasp stability, minimize squeezing
3. Coordination: hands move together (object rigid)

Jacobian relationship:
v_obj = J_left * q_dot_left = J_right * q_dot_right

Combined: v_obj = [J_left | J_right] * [q_dot_left; q_dot_right]
```

### MoveIt 2 Architecture

```
PLANNING SCENE (world model)
+-----------------------------------+
| Robot state (joint angles)        |
| Object locations                  |
| Collision geometry                |
| Attached objects (grasped items)  |
+-----------------------------------+
           |
           | (updated from perception)
           v
+-----------------------+
| MOTION PLANNING       |
+-----------------------+
| Goal: target pose     |
| Constraints:          |
| - Collision avoidance |
| - Joint limits        |
| - Cartesian path      |
+-----------------------+
           |
           | (RRT, RRT-Connect, PRM, etc.)
           v
+-----------------------+
| TRAJECTORY            |
| PROCESSING            |
+-----------------------+
| - Time parameterization|
| - Smoothing           |
| - Optimization        |
+-----------------------+
           |
           v
+-----------------------+
| EXECUTION             |
+-----------------------+
| - Send to controllers |
| - Monitor progress    |
| - Handle failures     |
+-----------------------+
           |
           v
      Robot Motion
```

### Pick and Place Sequence

```
PICK AND PLACE STAGES:

1. APPROACH
   Robot arm -----> [Object]
   (move to pre-grasp pose)

2. GRASP
   Hand -----> [Object]
   (close fingers, establish contact)

3. RETREAT
   Robot arm /\ with [Object]
   (lift object)

4. TRANSPORT
   Robot arm -----------> with [Object]
   (move to place location)

5. PLACE
   Robot arm \/ with [Object]
   (lower object to surface)

6. RELEASE
   Hand ----X [Object]
   (open fingers, release)

7. WITHDRAW
   Robot arm <----- from [Object]
   (move away)

Each stage has:
- Constraints (collision avoidance, force limits)
- Goals (target pose/configuration)
- Success criteria (force sensing, vision confirmation)
```

## Knowledge Checkpoint

Test your understanding of manipulation and grasping:

1. **Hand Design**: Compare fully-actuated and underactuated hand designs. What advantages does each provide, and for what applications would you choose each design?

2. **Grasp Taxonomy**: Explain the difference between power grasps and precision grasps. Provide examples of tasks that require each type and explain why.

3. **Force Closure**: A three-fingered hand grasps a planar object with point contacts. What geometric conditions must the contact points satisfy to achieve force closure with friction? Why is friction necessary?

4. **Form vs Force Closure**: Explain the difference between form closure and force closure. Which is more commonly achieved by robotic hands, and why?

5. **Friction Cone**: Draw a friction cone for a contact with friction coefficient mu = 0.5. What does this cone represent, and how does it constrain the contact force?

6. **Grasp Quality**: Two grasps both achieve force closure on the same object. One has epsilon quality of 2.0 N, the other has epsilon quality of 5.0 N. What does this difference mean in practical terms?

7. **Tactile Sensing**: Explain how slip detection works using high-frequency tactile sensing. What changes in the tactile signal indicate slip, and how should the controller respond?

8. **Grasp Planning**: Describe the trade-offs between analytical grasp planning (using geometric rules) and sampling-based grasp planning (randomly generating and evaluating candidates).

9. **In-Hand Manipulation**: During finger gaiting, why must the remaining fingers maintain force closure while one finger repositions? What happens if this condition is violated?

10. **Impedance Control**: Explain how impedance control differs from pure position control or pure force control. When is impedance control preferable for manipulation tasks?

11. **Bi-Manual Coordination**: When two hands grasp a rigid object, internal forces can arise without changing the net object wrench. Explain what internal forces are and why minimizing them is desirable.

12. **Manipulability**: The manipulability measure for a grasp quantifies how easily the grasped object can be moved. What factors contribute to high manipulability? How does manipulability relate to the grasp Jacobian?

13. **Contact Modeling**: Real contacts between robotic fingers and objects violate the point-contact assumption used in many grasp models. Name three ways real contacts differ from idealized point contacts and how these differences affect grasp planning.

14. **Grasp Execution**: During grasp execution, vision initially guides the hand toward the object, but tactile feedback becomes more important after contact. Explain why this sensory transition occurs and what each modality contributes.

## Chapter Summary

This chapter explored the principles and techniques enabling humanoid robots to grasp and manipulate objects. We began with anthropomorphic hand design, examining the mechanical structures, actuation strategies, and sensor integration that provide dexterous manipulation capability. The fundamental trade-off between full actuation (maximum control authority) and underactuation (practical simplicity and robustness) shapes hand design choices.

Grasp taxonomies organize the diverse ways hands interact with objects. Power grasps use the entire hand for stable, strong grips. Precision grasps use fingertips for fine manipulation control. Understanding this taxonomy helps select appropriate grasps for different tasks and objects.

Force closure provides the mathematical foundation for grasp stability. A grasp achieves force closure when contact forces can resist arbitrary external wrenches. Form closure, where geometry alone prevents motion, represents the ideal but is difficult to achieve practically. Most robotic grasps rely on force closure with friction.

Grasp quality metrics quantify how good force-closure grasps are. Epsilon quality measures robustness to disturbances. Ferrari-Canny metric characterizes uniform wrench capability. Manipulability indicates ease of object motion. These metrics guide grasp selection, preferring grasps that better serve task requirements.

Tactile sensing closes the feedback loop for manipulation. Force sensors detect contact forces, enabling force control. Slip detection triggers reactive grasp adjustment. Multi-modal tactile sensors provide rich information about contact state. Integration of vision and touch provides robust perception throughout manipulation.

Grasp planning algorithms select finger placements and contact forces. Analytical methods use geometric rules for simple shapes. Sampling-based approaches generate and evaluate many candidates for complex objects. Learning-based methods predict grasp success from training data. All approaches must verify kinematic reachability and collision avoidance.

In-hand manipulation enables repositioning grasped objects without releasing. Finger gaiting walks fingers around objects. Rolling contacts achieve reorientation through coordinated finger motion. These advanced techniques require carefully maintaining force closure through state transitions.

Bi-manual manipulation coordinates two arms to manipulate single objects or perform assembly tasks. The coupled kinematics require coordinated motion planning. Force distribution must balance load between hands while minimizing internal forces. Task decomposition simplifies planning by assigning appropriate roles to each arm.

Practical manipulation faces numerous challenges: modeling uncertainty, contact dynamics complexity, high-dimensional control spaces, perception limitations, and generalization across object classes. Robust systems combine feedforward planning with feedback adaptation, using vision and touch to detect and correct errors.

MoveIt 2 provides a comprehensive framework integrating planning, perception, and control for manipulation. Its modular architecture enables using different planners, trajectory processors, and controllers while handling common concerns like collision avoidance and constraint satisfaction.

The concepts developed in this chapter—force closure, quality metrics, tactile feedback, and coordinated control—form the foundation for capable manipulation systems. As humanoid robots take on increasingly complex tasks, robust grasping and dexterous manipulation become essential capabilities.

## Further Reading

### Foundational Textbooks

1. Murray, R. M., Li, Z., & Sastry, S. S. (1994). "A Mathematical Introduction to Robotic Manipulation." CRC Press.
   - Rigorous mathematical treatment of kinematics, contact models, and force closure theory.

2. Lynch, K. M., & Park, F. C. (2017). "Modern Robotics: Mechanics, Planning, and Control." Cambridge University Press.
   - Comprehensive coverage including manipulation, grasping, and motion planning with clear explanations.

3. Siciliano, B., & Khatib, O. (Eds.). (2016). "Springer Handbook of Robotics" (2nd ed.). Springer.
   - Extensive chapters on grasping, dexterous manipulation, and haptics by leading researchers.

### Grasp Analysis and Planning

4. Prattichizzo, D., & Trinkle, J. C. (2016). "Grasping." In Springer Handbook of Robotics (pp. 955-988). Springer.
   - Comprehensive overview of grasp theory, including force closure, quality metrics, and planning algorithms.

5. Sahbani, A., El-Khoury, S., & Bidaud, P. (2012). "An Overview of 3D Object Grasp Synthesis Algorithms." Robotics and Autonomous Systems, 60(3), 326-336.
   - Survey of grasp planning approaches with taxonomy and comparative analysis.

6. Miller, A. T., & Allen, P. K. (2004). "Graspit! A Versatile Simulator for Robotic Grasping." IEEE Robotics & Automation Magazine, 11(4), 110-122.
   - Description of influential grasp simulation and planning tool.

### Hand Design

7. Piazza, C., Grioli, G., Catalano, M. G., & Bicchi, A. (2019). "A Century of Robotic Hands." Annual Review of Control, Robotics, and Autonomous Systems, 2, 1-32.
   - Historical survey of robotic hand designs with analysis of design principles and trends.

8. Dollar, A. M., & Howe, R. D. (2010). "The Highly Adaptive SDM Hand: Design and Performance Evaluation." International Journal of Robotics Research, 29(5), 585-597.
   - Detailed analysis of underactuated hand design principles with experimental validation.

### Tactile Sensing

9. Dahiya, R. S., Metta, G., Valle, M., & Sandini, G. (2010). "Tactile Sensing—From Humans to Humanoids." IEEE Transactions on Robotics, 26(1), 1-20.
   - Comprehensive review of tactile sensor technologies and their application to robotics.

10. Wettels, N., Santos, V. J., Johansson, R. S., & Loeb, G. E. (2008). "Biomimetic Tactile Sensor Array." Advanced Robotics, 22(8), 829-849.
    - BioTac sensor design and capabilities for rich tactile perception.

### Dexterous Manipulation

11. Okamura, A. M., Smaby, N., & Cutkosky, M. R. (2000). "An Overview of Dexterous Manipulation." Proceedings of IEEE International Conference on Robotics and Automation.
    - Survey of in-hand manipulation techniques and challenges.

12. Rus, D. (1999). "In-Hand Dexterous Manipulation of Piecewise-Smooth 3-D Objects." International Journal of Robotics Research, 18(4), 355-381.
    - Theoretical framework and algorithms for finger gaiting and reorientation.

### Learning-Based Grasping

13. Bohg, J., Morales, A., Asfour, T., & Kragic, D. (2014). "Data-Driven Grasp Synthesis—A Survey." IEEE Transactions on Robotics, 30(2), 289-309.
    - Survey of learning and data-driven approaches to grasp planning.

14. Levine, S., Pastor, P., Krizhevsky, A., Ibarz, J., & Quillen, D. (2018). "Learning Hand-Eye Coordination for Robotic Grasping with Deep Learning and Large-Scale Data Collection." International Journal of Robotics Research, 37(4-5), 421-436.
    - Deep learning approach to grasp planning with large-scale robot data.

### Bi-Manual Manipulation

15. Smith, C., Karayiannidis, Y., Nalpantidis, L., Gratal, X., Qi, P., Dimarogonas, D. V., & Kragic, D. (2012). "Dual Arm Manipulation—A Survey." Robotics and Autonomous Systems, 60(10), 1340-1353.
    - Comprehensive survey of bi-manual and dual-arm manipulation techniques.

### MoveIt 2 and Motion Planning

16. MoveIt 2 Documentation: https://moveit.ros.org/
    - Official documentation, tutorials, and API reference for the MoveIt 2 framework.

17. Chitta, S., Sucan, I., & Cousins, S. (2012). "MoveIt! [ROS Topics]." IEEE Robotics & Automation Magazine, 19(1), 18-19.
    - Overview of the MoveIt framework architecture and capabilities.

18. Sucan, I. A., Moll, M., & Kavraki, L. E. (2012). "The Open Motion Planning Library." IEEE Robotics & Automation Magazine, 19(4), 72-82.
    - Description of OMPL, the motion planning library underlying MoveIt.

### Practical Implementation

19. Ciocarlie, M., Lackner, C., & Allen, P. (2007). "Soft Finger Model with Adaptive Contact Geometry for Grasping and Manipulation Tasks." Second Joint EuroHaptics Conference and Symposium on Haptic Interfaces.
    - Practical contact modeling for compliant fingertips.

20. Hsiao, K., Kaelbling, L. P., & Lozano-Pérez, T. (2010). "Task-Driven Tactile Exploration." Robotics: Science and Systems.
    - Using tactile feedback to guide manipulation and exploration.

## Looking Ahead

With manipulation capabilities established, Chapter 14 examines natural human-robot interaction—how humanoids communicate and collaborate with humans through gestures, gaze, speech, and safe physical contact. This final chapter integrates locomotion and manipulation with social and interactive capabilities.

Natural interaction requires anthropomorphic design extending beyond functional capability to include social signals. Gesture recognition and generation enable non-verbal communication. Gaze direction indicates attention and intention. Facial expressions (for robots equipped with expressive faces) convey emotional state and social engagement.

The manipulation techniques developed in this chapter directly support interactive tasks. Compliant control, introduced for gentle object handling, becomes essential for safe physical human-robot interaction. Force sensing and tactile feedback, used for grasp stability, enable detecting human contact and responding appropriately. Motion planning with collision avoidance extends to predicting and accommodating human motion.

Safety considerations become paramount when robots work alongside humans. ISO standards define safety requirements for collaborative robots. Collision detection must be fast and reliable. Control systems must limit forces and velocities to prevent injury. Understanding these constraints shapes interaction design.

The integration of perception, cognition, and control reaches its fullest expression in natural human-robot interaction. Vision tracks human position and recognizes gestures. Planning anticipates human intentions and coordinates robot actions. Control executes motions that are both effective and socially appropriate.

Chapter 14 will explore proxemics (spatial relationships), multi-modal interaction (combining speech, gesture, and gaze), compliant control for safe contact, and the standards and best practices for collaborative robotics. The goal is humanoid systems that work seamlessly alongside humans, understanding social cues and responding in natural, safe, and effective ways.

