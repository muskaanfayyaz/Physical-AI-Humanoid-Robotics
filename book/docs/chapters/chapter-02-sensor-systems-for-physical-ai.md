# Chapter 2: Sensor Systems for Physical AI

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand different sensor modalities for robotics (vision, inertial, force)
- Configure and calibrate LiDAR, depth cameras, and IMUs
- Implement sensor data acquisition and processing pipelines
- Apply sensor fusion techniques for robust perception
- Troubleshoot common sensor integration issues

## Introduction

In the physical world, perception is paramount. A humanoid robot without sensors is blind, deaf, and numb—unable to navigate, manipulate objects, or respond to its environment. Sensors are the interface between the digital brain and the physical world, converting light, sound, motion, and force into data that algorithms can process.

This chapter explores the sensor systems that enable Physical AI. We examine vision sensors (cameras, LiDAR, depth sensors), inertial sensors (IMUs, gyroscopes), and force sensors, understanding how each modality provides unique information about the robot and its environment. We also discuss sensor fusion—the art of combining multiple sensor streams to achieve robust, reliable perception.

## Vision Systems

### Cameras: The Foundation of Robot Vision

Standard cameras capture two-dimensional projections of three-dimensional scenes. A camera sensor consists of millions of photosensitive elements (pixels) that measure light intensity and color.

**Key Camera Characteristics:**

**Resolution:** The number of pixels (e.g., 1920x1080, 4K). Higher resolution provides more detail but requires more processing.

**Frame Rate:** Images captured per second (e.g., 30 fps, 60 fps). Higher frame rates enable tracking fast motion.

**Field of View (FOV):** The angular extent of the scene captured. Wide FOV provides situational awareness; narrow FOV gives detail.

**Dynamic Range:** The ratio between brightest and darkest intensities the sensor can capture. High dynamic range (HDR) handles challenging lighting.

**Color vs. Monochrome:** Color cameras provide RGB information; monochrome cameras offer higher sensitivity in low light.

**Camera Limitations:**

Cameras project 3D scenes onto 2D images, losing depth information. Determining how far away an object is requires additional techniques like:
- Stereo vision (using two cameras)
- Structure from motion (using camera movement)
- Depth sensors (adding dedicated depth measurement)

Cameras also struggle with:
- Low light conditions
- High-contrast scenes (bright sunlight and deep shadows)
- Reflective or transparent surfaces
- Motion blur during fast movement

### Depth Cameras: Adding the Third Dimension

Depth cameras measure distance to surfaces, providing a depth map where each pixel contains both color and distance information (RGB-D data).

**Structured Light Depth Cameras:**

These systems project a known pattern (often infrared) onto the scene and measure how the pattern deforms. The deformation reveals surface geometry.

Process:
1. Projector emits infrared pattern
2. IR camera captures reflected pattern
3. Algorithm compares captured pattern to reference
4. Depth is calculated from pattern distortion

Advantages:
- Works indoors
- Provides dense depth maps
- Moderate cost

Limitations:
- Fails in bright sunlight (IR interference)
- Limited range (typically 0.5-5 meters)
- Struggles with reflective or absorptive surfaces

**Time-of-Flight (ToF) Cameras:**

ToF cameras emit light pulses and measure the time for reflection to return. Distance = (speed of light × time) / 2.

Process:
1. Emit modulated light pulse
2. Measure phase shift of returned light
3. Calculate distance from phase difference
4. Generate depth map

Advantages:
- Fast depth acquisition
- Works at various ranges
- Less sensitive to textures

Limitations:
- Lower resolution than structured light
- Susceptible to multi-path interference
- Higher power consumption

**Stereo Cameras:**

Stereo systems use two cameras (like human eyes) to compute depth through triangulation. The disparity (difference in position) of features between left and right images reveals depth.

Process:
1. Capture images from two cameras
2. Identify corresponding features in both images
3. Calculate disparity (horizontal offset)
4. Compute depth from disparity and camera baseline

Advantages:
- Works outdoors (no active illumination)
- Longer range than structured light
- Passive sensing (low power)

Limitations:
- Requires textured surfaces
- Computationally intensive
- Fails in textureless regions

**Intel RealSense D435i:**

The RealSense D435i is a popular depth camera for robotics, combining:
- Stereo depth cameras
- RGB camera
- Infrared projector (for texture assistance)
- IMU (Inertial Measurement Unit)

This combination provides RGB-D data plus inertial information for motion tracking—ideal for Visual SLAM and object manipulation tasks.

### LiDAR: Precision Distance Measurement

LiDAR (Light Detection and Ranging) uses laser pulses to measure distances with high precision. Unlike cameras, LiDAR directly measures distance rather than inferring it.

**How LiDAR Works:**

1. Emit laser pulse in a specific direction
2. Measure time until reflection returns
3. Calculate distance: d = (c × t) / 2 where c is speed of light
4. Rotate or scan to cover field of view
5. Generate point cloud (set of 3D points)

**LiDAR Types:**

**Mechanical Spinning LiDAR:**
- Rotating mirror or entire sensor unit
- Provides 360-degree coverage
- Used in autonomous vehicles
- Example: Velodyne sensors

**Solid-State LiDAR:**
- No moving parts (uses phase arrays or MEMS mirrors)
- More reliable, compact
- Limited field of view
- Increasingly common in robotics

**LiDAR Characteristics:**

**Range:** From a few meters to hundreds of meters depending on sensor

**Angular Resolution:** Density of measurements (e.g., 0.1-degree spacing)

**Scanning Rate:** How fast it completes a full scan (e.g., 10 Hz)

**Number of Beams:** Multi-beam LiDAR has vertical layers (e.g., 16, 32, 64 beams)

**Accuracy:** Typically centimeter-level precision

**LiDAR Advantages:**
- Excellent range accuracy
- Works in various lighting conditions
- Provides dense 3D point clouds
- Unaffected by texture or color

**LiDAR Limitations:**
- Higher cost than cameras
- Generates massive data volumes
- Struggles with reflective or transparent surfaces
- Cannot capture color/texture information

### Comparing Vision Sensors

| Sensor Type | Range | Resolution | Sunlight | Cost | 3D Data | Color |
|-------------|-------|------------|----------|------|---------|-------|
| Camera | Far | High | Good | Low | No* | Yes |
| Depth Camera | 0.5-5m | Medium | Poor | Medium | Yes | Yes |
| LiDAR | 0-100m+ | Medium | Good | High | Yes | No |

*Cameras can provide 3D through stereo or SfM, but not directly.

## Inertial and Motion Sensors

### Inertial Measurement Units (IMUs)

An IMU measures acceleration and rotational velocity—critical for understanding a robot's motion and orientation. Modern IMUs combine multiple sensor types:

**Accelerometers:** Measure linear acceleration in three axes (x, y, z). When stationary, they measure gravity, revealing the "down" direction.

**Gyroscopes:** Measure angular velocity (rotation rate) around three axes. Integration of gyroscope data gives orientation changes.

**Magnetometers:** Measure magnetic field strength, providing absolute heading (compass direction).

**How IMUs Enable Robot Perception:**

**Orientation Estimation:** Combining accelerometer and gyroscope data provides the robot's orientation (roll, pitch, yaw) relative to gravity and magnetic north.

**Motion Detection:** Accelerometers detect when the robot starts or stops moving, useful for triggering actions.

**Vibration Analysis:** High-frequency IMU data can detect motor issues or terrain roughness.

**Sensor Fusion:** IMUs complement vision sensors. When cameras cannot determine motion (e.g., in textureless environments), IMUs provide velocity and orientation estimates.

**IMU Challenges:**

**Drift:** Gyroscopes accumulate error over time when integrated to obtain orientation. A small bias in angular velocity compounds into large orientation errors.

**Noise:** Accelerometers are noisy, especially during robot motion. Filtering is essential.

**Calibration:** IMUs require careful calibration to remove biases and correct for sensor imperfections.

**Mounting:** IMU placement affects measurements. Ideally mounted at the robot's center of mass.

**Example: RealSense D435i IMU:**

The integrated IMU in the D435i provides:
- 3-axis accelerometer
- 3-axis gyroscope
- Time-synchronized with camera frames
- Enables Visual-Inertial SLAM (combining vision and inertial data)

### Force and Torque Sensors

Force/torque sensors measure mechanical interaction between the robot and environment. These sensors are critical for manipulation and safe physical interaction.

**Force Sensors:**

Measure force applied in different directions (typically 3-axis: Fx, Fy, Fz). Common in:
- Feet (for balance and ground contact detection)
- Fingertips (for grasp force control)
- Joints (for detecting external forces)

**Torque Sensors:**

Measure rotational forces around axes (typically 3-axis: Tx, Ty, Tz). Used in:
- Joints (for detecting interaction torques)
- Wrists (for tool force feedback)

**Six-Axis Force/Torque Sensors:**

Combine force and torque measurement in a single sensor, providing complete information about contact interactions. Mounted at the wrist, these sensors enable:
- Compliant control (adjusting to external forces)
- Tool force feedback
- Assembly task monitoring
- Collision detection

**Applications in Humanoid Robotics:**

**Grasp Force Control:** Adjusting grip strength based on object properties—firm for heavy objects, gentle for fragile items.

**Balance Control:** Foot force sensors measure ground contact and weight distribution, essential for bipedal balance.

**Compliant Interaction:** Detecting and responding to external forces, enabling safe human-robot collaboration.

**Object Property Estimation:** Inferring object weight and friction properties from manipulation forces.

## Sensor Fusion: Combining Multiple Modalities

No single sensor provides complete, reliable information. Sensor fusion combines multiple sensors to achieve robust perception that exceeds any individual sensor's capability.

### Why Sensor Fusion Matters

Different sensors have complementary strengths and weaknesses:

**Cameras:** Provide rich visual information but lose depth and struggle in poor lighting.

**LiDAR:** Provides accurate depth but no color/texture and has blind spots.

**IMUs:** Track motion reliably but drift over time.

**Force Sensors:** Detect contact but provide no distance information.

By combining sensors, we can:
- Fill in gaps where individual sensors fail
- Cross-validate measurements to detect errors
- Improve accuracy through redundancy
- Operate in diverse conditions

### Sensor Fusion Architectures

**Complementary Fusion:**

Different sensors measure different properties. Example: RGB camera provides color, depth camera provides distance. Combining them produces RGB-D data containing both.

**Competitive Fusion:**

Multiple sensors measure the same property. The fusion algorithm weights or selects the most reliable measurement. Example: Combining two distance measurements from LiDAR and stereo vision.

**Cooperative Fusion:**

Sensors work together to extract information impossible from individual sensors. Example: Visual-Inertial Odometry combines camera images and IMU data to track motion more accurately than either sensor alone.

### Common Fusion Techniques

**Kalman Filtering:**

The Kalman filter is a mathematical framework for optimally combining measurements with different uncertainties. It maintains:
- State estimate (e.g., robot position and velocity)
- Uncertainty estimate (how confident we are)

Process:
1. Predict: Use motion model to predict next state
2. Update: Incorporate new sensor measurement
3. Fuse: Optimally combine prediction and measurement based on uncertainties

Kalman filters are widely used for IMU-based orientation estimation, fusing accelerometer and gyroscope data.

**Particle Filters:**

Particle filters represent uncertainty through a set of hypotheses (particles). Each particle is a possible state. The algorithm:
1. Propagates particles forward using motion model
2. Weights particles based on sensor likelihood
3. Resamples to focus on high-probability regions

Useful when uncertainty is non-Gaussian or multi-modal.

**Complementary Filters:**

Simpler than Kalman filters, complementary filters combine high-frequency and low-frequency sensor data. Example for orientation:
- Gyroscope provides accurate short-term rotation (high-pass filter)
- Accelerometer provides absolute tilt reference (low-pass filter)
- Combine: orientation = α × (orientation + gyro × dt) + (1-α) × accel_orientation

Fast, efficient, and widely used in embedded systems.

### Visual-Inertial Fusion

Combining cameras and IMUs exemplifies cooperative fusion:

**Camera Strengths:**
- Absolute position information (when features are visible)
- No drift in feature tracking
- Rich environmental detail

**Camera Weaknesses:**
- Fails in textureless environments
- Cannot measure motion directly
- Affected by lighting, blur, occlusions

**IMU Strengths:**
- High-frequency motion measurement
- Works in any visual condition
- Measures rotation directly

**IMU Weaknesses:**
- Drifts over time
- No absolute position information
- Noisy acceleration data

**Visual-Inertial Odometry (VIO):**

VIO algorithms fuse camera and IMU data to track robot motion. The IMU provides continuous motion estimates between camera frames, while the camera corrects accumulated IMU drift. This combination:
- Operates at high frequency (IMU rate: 200-1000 Hz)
- Maintains accuracy over time (camera corrections)
- Works through brief visual occlusions (IMU carries state forward)
- Provides both position and orientation

This makes VIO ideal for robot navigation, drone flight, and augmented reality applications.

## Calibration: The Foundation of Accurate Sensing

Sensors are imperfect. Manufacturing tolerances, environmental effects, and physical misalignments introduce errors. Calibration is the process of characterizing and correcting these imperfections.

### Camera Calibration

Cameras have intrinsic parameters (internal properties) and extrinsic parameters (position/orientation in space).

**Intrinsic Parameters:**
- Focal length (zoom level)
- Principal point (optical center)
- Lens distortion (radial and tangential)

**Calibration Process:**
1. Capture images of a known pattern (checkerboard)
2. Detect pattern corners in images
3. Solve for camera parameters that best explain observations
4. Store calibration matrix and distortion coefficients

Proper calibration is essential for accurate depth estimation, 3D reconstruction, and visual servoing.

### IMU Calibration

IMUs have biases, scale factors, and axis misalignments that must be corrected.

**Accelerometer Calibration:**
- Collect data in multiple static orientations
- Solve for bias (zero-acceleration offset)
- Solve for scale factors (sensitivity per axis)
- Validate that magnitude equals gravity when stationary

**Gyroscope Calibration:**
- Collect data while stationary
- Measure bias (angular velocity when not rotating)
- Optionally measure scale factors using turntable

**Magnetometer Calibration:**
- Rotate sensor through full 3D space
- Fit ellipsoid to measurements
- Correct for hard iron (constant offset) and soft iron (scale/rotation) effects

### Stereo Camera Calibration

Stereo systems require calibrating:
- Each camera individually (intrinsic parameters)
- Relative position and orientation between cameras (extrinsic parameters)

The baseline (distance between cameras) and relative orientation determine depth accuracy. Even small calibration errors significantly degrade depth estimates.

### Multi-Sensor Calibration

When combining multiple sensor types (camera + LiDAR, camera + IMU), we must calibrate their relative positions and orientations.

**Hand-Eye Calibration:** Determining the transformation between a camera and a robot manipulator.

**Camera-IMU Calibration:** Finding the rigid transformation between camera and IMU frames, essential for VIO.

**LiDAR-Camera Calibration:** Aligning LiDAR point clouds with camera images for sensor fusion.

Specialized calibration routines and patterns (checkerboards, AprilTags, LiDAR-reflective targets) facilitate these calibrations.

## Data Synchronization and Processing

### Temporal Synchronization

Sensors operate at different rates:
- Cameras: 30-60 Hz
- LiDAR: 10-20 Hz
- IMU: 200-1000 Hz
- Force sensors: 100-1000 Hz

For sensor fusion, measurements must be time-aligned. Strategies include:

**Hardware Synchronization:** Triggering sensors from a common clock signal.

**Timestamp Alignment:** Recording accurate timestamps for each measurement and interpolating to common times.

**Buffering and Interpolation:** Maintaining short-term histories of each sensor and interpolating to align measurements temporally.

### Data Processing Pipelines

Raw sensor data requires processing before use:

**Image Processing:**
1. Undistort images using calibration
2. Convert color spaces if needed
3. Apply filters (noise reduction, edge detection)
4. Extract features or run neural networks

**Point Cloud Processing:**
1. Filter noise and outliers
2. Downsample to reduce data volume
3. Transform to common coordinate frame
4. Segment into objects or surfaces

**IMU Processing:**
1. Remove biases using calibration
2. Apply complementary or Kalman filtering
3. Integrate to obtain orientation
4. Fuse with other sensors

Efficient processing is critical—sensors generate megabytes per second. Real-time processing requires optimized algorithms and often GPU acceleration.

## Conceptual Diagrams

### Diagram 1: Sensor Modalities and Information

```
Robot Sensor Suite:

Vision Sensors:
┌──────────────┬────────────────┬───────────────┐
│   Camera     │ Depth Camera   │    LiDAR      │
├──────────────┼────────────────┼───────────────┤
│ 2D Image     │ RGB-D Data     │ Point Cloud   │
│ Color/Texture│ Color + Depth  │ 3D Position   │
│ No Depth*    │ Limited Range  │ No Color      │
│ Low Cost     │ Medium Cost    │ High Cost     │
└──────────────┴────────────────┴───────────────┘

Inertial Sensors:
┌──────────────┬────────────────┬───────────────┐
│ Accelerometer│  Gyroscope     │ Magnetometer  │
├──────────────┼────────────────┼───────────────┤
│ Linear Accel │ Angular Velocity│ Magnetic Field│
│ Gravity Dir  │ Rotation Rate  │ Heading       │
│ Drifts (Pos) │ Drifts (Orient)│ Local Disturb │
└──────────────┴────────────────┴───────────────┘

Force Sensors:
┌──────────────┬────────────────┐
│ Force Sensor │ Torque Sensor  │
├──────────────┼────────────────┤
│ Linear Force │ Rotational Force│
│ Contact Detect│ Joint Loading  │
│ Grasp Control│ Compliance     │
└──────────────┴────────────────┘
```

### Diagram 2: Sensor Fusion for Robust Perception

```
Sensor Fusion Architecture:

Individual Sensors:
┌─────────┐  ┌─────────┐  ┌─────────┐
│ Camera  │  │ LiDAR   │  │  IMU    │
└────┬────┘  └────┬────┘  └────┬────┘
     │            │            │
     └────────┬───┴────────────┘
              │
        ┌─────▼──────┐
        │   Sensor   │
        │   Fusion   │
        │  Algorithm │
        └─────┬──────┘
              │
      ┌───────▼────────┐
      │ Fused Estimate │
      │  - Position    │
      │  - Orientation │
      │  - Velocity    │
      │  - Environment │
      └────────────────┘
              │
         (More reliable than
          any single sensor)
```

### Diagram 3: Visual-Inertial Odometry

```
Visual-Inertial Odometry (VIO):

Camera Path:
[Images] → [Feature Tracking] → [Visual Odometry]
                                        ↓
                                  [Position/Orient
                                   from Features]
                                        ↓
                                        ↓
IMU Path:                               ↓
[IMU Data] → [Integration] → [Motion Prediction]
                                        ↓
                                        ↓
                            ┌───────────▼──────────┐
                            │  Extended Kalman     │
                            │  Filter (Fusion)     │
                            └───────────┬──────────┘
                                        │
                              ┌─────────▼──────────┐
                              │  Accurate Pose     │
                              │  - No drift        │
                              │  - High frequency  │
                              │  - Robust          │
                              └────────────────────┘
```

## Key Concepts Summary

### RGB-D Sensing
Combination of color (RGB) and depth (D) information, providing both appearance and geometry of the environment.

### Point Cloud
Set of 3D points representing surfaces in space, typically generated by LiDAR or depth cameras.

### Inertial Measurement Unit (IMU)
Sensor combining accelerometers and gyroscopes to measure linear acceleration and angular velocity.

### Sensor Fusion
Process of combining data from multiple sensors to produce more accurate and reliable estimates than any single sensor.

### Calibration
Process of determining sensor parameters and correcting systematic errors to ensure accurate measurements.

### Visual-Inertial Odometry (VIO)
Technique combining camera images and IMU data to track motion, leveraging strengths of both modalities.

### Sensor Noise
Random variations in sensor measurements caused by electrical interference, quantization, and physical limitations.

### Sensor Drift
Gradual accumulation of error over time, particularly problematic in gyroscopes and accelerometers.

## Knowledge Checkpoint

Test your understanding of this chapter's concepts:

1. **Sensor Selection:**
   - A humanoid robot must navigate a warehouse with varying lighting (bright sunlight near windows, dark aisles). Which sensors would you select and why?
   - For a manipulation task requiring grasping fragile objects, what sensor modalities are essential?
   - Compare the suitability of depth cameras vs. LiDAR for indoor navigation in a home environment.

2. **Sensor Fusion:**
   - Explain why combining cameras and IMUs produces better motion tracking than using either sensor alone.
   - A robot's LiDAR detects an obstacle at 2.5 meters, but the stereo camera estimates 2.8 meters. How would you resolve this discrepancy?
   - Describe a scenario where sensor fusion would fail if sensors are not properly time-synchronized.

3. **Calibration:**
   - Why is camera calibration necessary before using visual information for depth estimation or robot control?
   - An IMU shows a constant gyroscope reading of 0.5 degrees/second when the robot is stationary. What is this error called and how would you correct it?
   - Explain why the relative position between cameras in a stereo system must be calibrated accurately.

4. **Application Design:**
   - Design a sensor suite for a bipedal humanoid that must navigate stairs, avoid obstacles, and pick up objects. Justify each sensor choice.
   - A depth camera works well indoors but fails outdoors in sunlight. Propose a solution using sensor fusion.
   - For a robot that must detect when it collides with objects during navigation, what sensor modality is most appropriate?

## Chapter Summary

This chapter explored the sensor systems that enable Physical AI. Key takeaways include:

**Vision Sensors:** Cameras provide rich visual information but lose depth in single images. Depth cameras (structured light, ToF, stereo) add the third dimension but have range and environmental limitations. LiDAR offers precise distance measurement and works in varied lighting but cannot capture color or texture.

**Inertial Sensors:** IMUs combining accelerometers and gyroscopes measure motion and orientation. They provide high-frequency state estimates essential for control but suffer from drift. Magnetometers add absolute heading reference.

**Force Sensors:** Measure mechanical interaction between robot and environment. Essential for manipulation, balance control, and safe physical interaction. Enable compliant control and grasp force regulation.

**Sensor Fusion:** Combining multiple sensor modalities produces robust perception exceeding any single sensor's capability. Techniques like Kalman filtering, complementary filtering, and particle filtering optimally combine measurements with different characteristics and uncertainties.

**Calibration:** All sensors have imperfections requiring calibration. Camera calibration corrects lens distortion and determines intrinsic parameters. IMU calibration removes biases and scale errors. Multi-sensor calibration establishes spatial relationships between sensors.

**Data Processing:** Raw sensor data requires filtering, transformation, and synchronization. Real-time processing demands efficient algorithms and often GPU acceleration. Proper time synchronization is critical for fusion.

The sensors covered in this chapter form the perceptual foundation for all subsequent topics. As we progress to ROS 2 in the next chapter, you will learn how to integrate these sensors into a coherent robotic system.

## Further Reading

**Books:**
- "Computer Vision: Algorithms and Applications" by Richard Szeliski
- "Inertial Navigation Systems" by Paul D. Groves
- "Probabilistic Robotics" by Thrun, Burgard, and Fox (Chapters on Sensors)

**Papers:**
- "A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry"
- "LiDAR-Camera Calibration: A Review"
- "VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator"

**Technical Documentation:**
- Intel RealSense D435i documentation and datasheets
- Velodyne LiDAR technical specifications
- IMU calibration procedures (Bosch BMI088, InvenSense ICM-20948)

**Tutorials and Code:**
- OpenCV camera calibration tutorials
- ROS sensor driver documentation
- Kalman filter implementations

## Looking Ahead

With understanding of how robots perceive their environment, we now turn to ROS 2—the Robot Operating System. ROS 2 provides the middleware that connects sensors, planning algorithms, and motor controllers into a coherent system. In the next three chapters, we will master ROS 2 fundamentals, learn to build complex multi-node systems, and apply these skills specifically to humanoid robots.
