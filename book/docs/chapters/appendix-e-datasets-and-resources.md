# Appendix E: Datasets and Resources

This appendix catalogs publicly available datasets, pre-trained models, 3D assets, and community resources for Physical AI and humanoid robotics research and development.

## E.1 Publicly Available Robot Datasets

### E.1.1 Manipulation Datasets

**YCB Object and Model Set**

- **Description:** 77 household objects with high-quality 3D meshes, texture maps, and physical properties
- **Content:** Kitchen items, tools, food packages, toys
- **Formats:** OBJ, STL meshes; texture images; physics parameters
- **Use Cases:** Grasping, manipulation planning, object recognition
- **Size:** ~5 GB (full dataset)
- **Access:** http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/
- **License:** Creative Commons Attribution 4.0

**YCB-Video Dataset**

- **Description:** RGB-D video sequences of YCB objects in real scenes
- **Content:** 92 video sequences, 133,827 frames
- **Annotations:** 6D object poses, segmentation masks
- **Use Cases:** 6D pose estimation, object tracking
- **Size:** ~12 GB
- **Access:** https://rse-lab.cs.washington.edu/projects/posecnn/
- **Citation:** Xiang et al., "PoseCNN: A Convolutional Neural Network for 6D Object Pose Estimation", RSS 2018

**Google Scanned Objects**

- **Description:** High-quality 3D scans of household objects
- **Content:** 1,030 objects scanned with structure-from-motion
- **Formats:** OBJ meshes with textures
- **Quality:** Watertight meshes, photorealistic textures
- **Use Cases:** Simulation, synthetic data generation
- **Size:** ~8 GB
- **Access:** https://app.ignitionrobotics.org/GoogleResearch/fuel/collections/Google%20Scanned%20Objects
- **License:** Creative Commons BY 4.0

**ACRONYM Grasping Dataset**

- **Description:** 17.7M parallel-jaw grasps on 8,872 ShapeNet objects
- **Content:** Grasp poses, success predictions, object meshes
- **Format:** HDF5 files with grasp data
- **Use Cases:** Grasp synthesis, learning-based grasping
- **Size:** ~6 GB
- **Access:** https://sites.google.com/nvidia.com/graspdataset
- **Citation:** Eppner et al., "ACRONYM: A Large-Scale Grasp Dataset", ICRA 2021

**Columbia Grasp Database**

- **Description:** Grasps for household objects using various grippers
- **Content:** 287 objects, multiple gripper configurations
- **Annotations:** Grasp quality metrics, success rates
- **Use Cases:** Grasp planning, gripper design evaluation
- **Access:** https://grasping.cs.columbia.edu/
- **Citation:** Goldfeder et al., "The Columbia Grasp Database", ICRA 2009

### E.1.2 Navigation Datasets

**TUM RGB-D Dataset**

- **Description:** RGB-D sequences for visual odometry and SLAM evaluation
- **Content:** 41 sequences in office/home environments
- **Sensors:** Microsoft Kinect (640×480 RGB-D at 30 Hz)
- **Ground Truth:** Motion capture system (high precision)
- **Use Cases:** Visual SLAM, RGB-D odometry, depth estimation
- **Size:** ~34 GB (full dataset)
- **Access:** https://vision.in.tum.de/data/datasets/rgbd-dataset
- **Citation:** Sturm et al., "A Benchmark for RGB-D SLAM Evaluation", IROS 2012

**EuRoC MAV Dataset**

- **Description:** Visual-inertial datasets from micro aerial vehicle
- **Content:** 11 sequences in machine hall and room environments
- **Sensors:** Stereo cameras (20 Hz), IMU (200 Hz)
- **Ground Truth:** Laser tracker and motion capture
- **Use Cases:** Visual-inertial odometry, SLAM
- **Size:** ~20 GB
- **Access:** https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
- **Citation:** Burri et al., "The EuRoC MAV Dataset", IJRR 2016

**KITTI Dataset**

- **Description:** Autonomous driving datasets with lidar, cameras, GPS/IMU
- **Content:**
  - Odometry: 22 stereo sequences with ground truth
  - 3D Object Detection: 15K annotated images
  - Tracking: 50 sequences with object trajectories
- **Sensors:** Velodyne lidar, stereo cameras, GPS/IMU
- **Use Cases:** Visual odometry, 3D detection, tracking, mapping
- **Size:** ~200 GB (varies by task)
- **Access:** http://www.cvlibs.net/datasets/kitti/
- **Citation:** Geiger et al., "Vision meets Robotics: The KITTI Dataset", IJRR 2013

**NCLT Dataset (North Campus Long-Term)**

- **Description:** Long-term autonomous navigation dataset
- **Content:** 27 sessions over 15 months, same route
- **Sensors:** Velodyne lidar, cameras, IMU, GPS
- **Unique Feature:** Seasonal and lighting variations
- **Use Cases:** Long-term SLAM, place recognition, change detection
- **Size:** ~3.6 TB (full dataset)
- **Access:** http://robots.engin.umich.edu/nclt/
- **Citation:** Carlevaris-Bianco et al., "University of Michigan NCLT Dataset", IJRR 2016

### E.1.3 Human Motion Datasets

**CMU Graphics Lab Motion Capture Database**

- **Description:** Largest free motion capture dataset
- **Content:** 2,605 motion sequences, 144 subjects
- **Categories:** Walking, running, sports, interaction, dance, martial arts
- **Format:** BVH (motion capture), C3D (marker trajectories)
- **Use Cases:** Human pose estimation, motion retargeting, animation
- **Size:** ~2 GB
- **Access:** http://mocap.cs.cmu.edu/
- **License:** Free for research and commercial use

**Human3.6M**

- **Description:** Large-scale 3D human pose dataset
- **Content:** 3.6M video frames, 11 subjects, 17 scenarios
- **Sensors:** 4 RGB cameras, motion capture (ground truth)
- **Annotations:** 3D joint positions, body part segmentation
- **Use Cases:** 3D human pose estimation, action recognition
- **Size:** ~100 GB
- **Access:** http://vision.imar.ro/human3.6m/ (registration required)
- **Citation:** Ionescu et al., "Human3.6M: Large Scale Datasets for 3D Human Sensing", PAMI 2014

**AMASS (Archive of Motion Capture as Surface Shapes)**

- **Description:** Unified motion capture dataset with SMPL body model
- **Content:** 40+ hours, 300+ subjects, 11,000+ motions
- **Format:** SMPL parameters (shape and pose)
- **Sources:** Consolidated from 15 motion capture datasets
- **Use Cases:** Motion synthesis, human modeling, physics simulation
- **Size:** ~24 GB
- **Access:** https://amass.is.tue.mpg.de/
- **Citation:** Mahmood et al., "AMASS: Archive of Motion Capture as Surface Shapes", ICCV 2019

**HumanEva Dataset**

- **Description:** Synchronized video and motion capture for pose estimation
- **Content:** 7 calibrated video sequences, 4 subjects
- **Actions:** Walking, jogging, gestures, throwing, boxing
- **Ground Truth:** Motion capture system
- **Use Cases:** 2D/3D pose estimation benchmarking
- **Access:** http://humaneva.is.tue.mpg.de/
- **Citation:** Sigal et al., "HumanEva: Synchronized Video and Motion Capture Dataset", IJCV 2010

### E.1.4 Grasping Datasets

**Dex-Net 1.0, 2.0, 3.0, 4.0**

- **Description:** Synthetic datasets for robot grasping
- **Content:**
  - Dex-Net 1.0: 10M point clouds, 2.5M grasps
  - Dex-Net 2.0: 6.7M point clouds, parallel jaw grasps
  - Dex-Net 3.0: Suction cup grasping
  - Dex-Net 4.0: Ambidextrous grasping (parallel + suction)
- **Use Cases:** Deep learning for grasp planning
- **Access:** https://berkeleyautomation.github.io/dex-net/
- **Citation:** Mahler et al., "Dex-Net 2.0: Deep Learning to Plan Robust Grasps", RSS 2017

**PartNet-Mobility**

- **Description:** Articulated object dataset with motion annotations
- **Content:** 2,346 3D objects with moving parts
- **Annotations:** Part segmentation, joint parameters, motion ranges
- **Categories:** Cabinets, doors, drawers, appliances
- **Use Cases:** Articulated object manipulation, affordance learning
- **Size:** ~4 GB
- **Access:** https://sapien.ucsd.edu/
- **Citation:** Xiang et al., "SAPIEN: A SimulAted Part-based Interactive ENvironment", CVPR 2020

**ContactDB**

- **Description:** Contact patterns during human grasping
- **Content:** 50 household objects, 375 grasp demonstrations
- **Sensors:** Thermal camera to detect contact areas
- **Annotations:** Contact maps, 3D hand poses, forces
- **Use Cases:** Human grasp analysis, contact-rich manipulation
- **Access:** https://contactdb.cc.gatech.edu/
- **Citation:** Brahmbhatt et al., "ContactDB: Analyzing and Predicting Grasp Contact", CVPR 2019

---

## E.2 Pre-trained Models and Checkpoints

### E.2.1 Object Detection Models

**YOLO (You Only Look Once) Series**

| Model | Input Size | mAP | Speed (FPS) | Use Case | Download |
|-------|------------|-----|-------------|----------|----------|
| YOLOv5s | 640×640 | 37.4 | 140 | Real-time, edge devices | https://github.com/ultralytics/yolov5 |
| YOLOv5m | 640×640 | 45.4 | 85 | Balanced | https://github.com/ultralytics/yolov5 |
| YOLOv8n | 640×640 | 37.3 | 200+ | Ultra-fast | https://github.com/ultralytics/ultralytics |
| YOLOv8s | 640×640 | 44.9 | 130 | Real-time | https://github.com/ultralytics/ultralytics |
| YOLOv8m | 640×640 | 50.2 | 80 | High accuracy | https://github.com/ultralytics/ultralytics |

**Frameworks:** PyTorch, ONNX, TensorRT
**Pre-trained on:** COCO (80 classes)

**Detectron2 Model Zoo**

- **Description:** Facebook AI's object detection framework
- **Models:**
  - Faster R-CNN (R50-FPN, R101-FPN)
  - RetinaNet
  - Mask R-CNN (instance segmentation)
  - Panoptic FPN
- **Backbones:** ResNet-50, ResNet-101, ResNeXt
- **Pre-trained on:** COCO, LVIS, Cityscapes
- **Access:** https://github.com/facebookresearch/detectron2/blob/main/MODEL_ZOO.md
- **Format:** PyTorch checkpoints

**EfficientDet**

- **Description:** Scalable and efficient object detection
- **Variants:** D0 (small) to D7 (large)
- **Performance:** D7 achieves 52.2 mAP on COCO
- **Access:** https://github.com/google/automl/tree/master/efficientdet
- **Format:** TensorFlow, PyTorch (via timm)

### E.2.2 Segmentation Models

**Segment Anything Model (SAM)**

- **Description:** Foundation model for image segmentation
- **Architecture:** Vision Transformer (ViT) based
- **Capabilities:** Zero-shot segmentation, prompt-based
- **Checkpoints:**
  - ViT-H (huge): 2.4B parameters, best quality
  - ViT-L (large): 1.2B parameters, balanced
  - ViT-B (base): 636M parameters, faster
- **Access:** https://github.com/facebookresearch/segment-anything
- **License:** Apache 2.0
- **Format:** PyTorch

**DeepLabV3+ / DeepLabV3**

- **Description:** Semantic segmentation with atrous convolution
- **Backbones:** ResNet-50, ResNet-101, MobileNetV2
- **Pre-trained on:** COCO, Pascal VOC, Cityscapes
- **Access:** TensorFlow Model Garden, PyTorch Hub
- **Use Cases:** Scene understanding, outdoor navigation

**Mask R-CNN**

- **Description:** Instance segmentation (detection + masks)
- **Pre-trained models:** COCO 80 classes
- **Backbones:** ResNet-50-FPN, ResNet-101-FPN
- **Access:** Detectron2, TorchVision model zoo
- **mAP:** ~37-39 (depending on backbone)

**SegFormer**

- **Description:** Transformer-based semantic segmentation
- **Variants:** B0 (small) to B5 (large)
- **Performance:** 84.0 mIoU on ADE20K (B5)
- **Access:** https://github.com/NVlabs/SegFormer
- **Format:** PyTorch

### E.2.3 Pose Estimation Models

**OpenPose**

- **Description:** Real-time multi-person 2D pose estimation
- **Keypoints:**
  - Body: 18 or 25 keypoints
  - Hand: 21 keypoints per hand
  - Face: 70 keypoints
- **Framework:** Caffe, OpenCV DNN
- **Access:** https://github.com/CMU-Perceptual-Computing-Lab/openpose
- **Speed:** ~22 FPS (single person, GPU)

**MediaPipe Pose**

- **Description:** Lightweight pose estimation for mobile/edge
- **Keypoints:** 33 body landmarks (including face and hands)
- **Platform:** Mobile, Web, Desktop
- **Performance:** Real-time on CPU
- **Access:** https://google.github.io/mediapipe/solutions/pose.html
- **License:** Apache 2.0
- **Format:** TFLite

**HRNet (High-Resolution Net)**

- **Description:** State-of-art human pose estimation
- **Variants:** HRNet-W32, HRNet-W48
- **Performance:** 74.9 AP on COCO test-dev (W48)
- **Access:** https://github.com/leoxiaobin/deep-high-resolution-net.pytorch
- **Pre-trained on:** COCO, MPII
- **Format:** PyTorch

**6D Object Pose Models**

| Model | Description | Input | Output | Access |
|-------|-------------|-------|--------|--------|
| PoseCNN | CNN-based 6D pose | RGB-D | 6D pose + confidence | https://rse-lab.cs.washington.edu/projects/posecnn/ |
| DenseFusion | RGB-D fusion for pose | RGB-D | 6D pose | https://github.com/j96w/DenseFusion |
| PVNet | Pixel-wise voting | RGB | 6D pose | https://github.com/zju3dv/pvnet |
| FoundationPose | Foundation model | RGB-D | 6D pose (novel objects) | https://github.com/NVlabs/FoundationPose |

### E.2.4 Speech Recognition Models

**Whisper (OpenAI)**

- **Description:** Robust multilingual speech recognition
- **Variants:**
  - Tiny: 39M params, 32x real-time (CPU)
  - Base: 74M params, 16x real-time
  - Small: 244M params, 6x real-time
  - Medium: 769M params, 2x real-time
  - Large: 1550M params, 1x real-time
- **Languages:** 99 languages
- **Access:** https://github.com/openai/whisper
- **Format:** PyTorch
- **Use Case:** Robot voice commands, transcription

**Wav2Vec 2.0**

- **Description:** Self-supervised speech representation learning
- **Pre-trained models:** Base (95M), Large (317M)
- **Fine-tuned for:** English ASR, multilingual ASR
- **Access:** https://huggingface.co/models?search=wav2vec2
- **Framework:** Transformers (Hugging Face)
- **Use Case:** Custom wake word detection, ASR fine-tuning

**Vosk**

- **Description:** Offline speech recognition toolkit
- **Models:** 20+ languages, small to large variants
- **Size:** 50 MB (small) to 1.8 GB (large)
- **Platform:** Cross-platform (Linux, Windows, macOS, Android, iOS)
- **Access:** https://alphacephei.com/vosk/models
- **License:** Apache 2.0
- **Use Case:** Embedded systems, privacy-focused applications

---

## E.3 3D Model Libraries

### E.3.1 Robot URDF Repositories

**ROS Industrial Robot Support**

- **Description:** URDF models for industrial manipulators
- **Robots:** ABB, FANUC, Universal Robots, KUKA, Motoman
- **Content:** URDF/XACRO files, meshes, MoveIt configs
- **Access:** https://github.com/ros-industrial
- **Format:** URDF, DAE/STL meshes
- **License:** Varies (mostly BSD/Apache)

**Example repositories:**
- Universal Robots: https://github.com/ros-industrial/universal_robot
- ABB: https://github.com/ros-industrial/abb
- FANUC: https://github.com/ros-industrial/fanuc

**TIAGo Robot (PAL Robotics)**

- **Description:** Mobile manipulation platform URDF
- **Variants:** TIAGo Base, TIAGo with arm, TIAGo++
- **Content:** Full URDF, Gazebo simulation
- **Access:** https://github.com/pal-robotics/tiago_robot
- **Use Case:** Research on mobile manipulation

**Clearpath Robotics**

- **Description:** Mobile robot platforms
- **Robots:** Husky, Jackal, Ridgeback, Dingo
- **Content:** URDF, Gazebo worlds, navigation configs
- **Access:** https://github.com/clearpathrobotics
- **Format:** URDF/XACRO, STL/DAE meshes

**Unitree Robotics**

- **Description:** Quadruped and humanoid robots
- **Robots:** Go1, Go2, A1, Aliengo, G1 Humanoid
- **Content:** URDF, simulation setup
- **Access:**
  - https://github.com/unitreerobotics/unitree_ros
  - https://github.com/unitreerobotics/unitree_mujoco
- **Format:** URDF, MuJoCo XML

**Boston Dynamics Spot (Community)**

- **Description:** Community-created Spot URDF
- **Note:** Unofficial, for simulation only
- **Access:** https://github.com/chvmp/spot_ros
- **Format:** URDF

### E.3.2 Environment Models

**Gazebo Model Database**

- **Description:** Official Gazebo model repository
- **Content:** 100+ models (furniture, structures, robots)
- **Categories:** Construction, ground, people, robots, shapes
- **Access:** https://github.com/osrf/gazebo_models
- **Browser:** https://app.gazebosim.org/
- **Format:** SDF, COLLADA meshes

**AWS RoboMaker Small House World**

- **Description:** Residential environment for robot simulation
- **Content:** Furnished house model, Gazebo world
- **Size:** ~200 MB
- **Access:** https://github.com/aws-robotics/aws-robomaker-small-house-world
- **License:** MIT
- **Use Case:** Home service robot testing

**Unity Robotics Hub Environments**

- **Description:** Photorealistic environments for Unity
- **Content:** Warehouse, factory, outdoor scenes
- **Access:** https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example
- **Format:** Unity scenes
- **Use Case:** Synthetic data generation, visualization

**NVIDIA Isaac Sim Assets**

- **Description:** High-quality 3D assets for Isaac Sim
- **Content:** Warehouses, factories, retail, robots
- **Access:** Through Omniverse Nucleus
- **Format:** USD (Universal Scene Description)
- **License:** NVIDIA Omniverse license

### E.3.3 Object Meshes and CAD Files

**ShapeNet**

- **Description:** Large-scale 3D shape repository
- **Content:** 51,300 models, 55 categories
- **Subset:** ShapeNetCore (focus on common objects)
- **Format:** OBJ, MTL (materials)
- **Access:** https://shapenet.org/ (registration required)
- **License:** Varies by model
- **Use Case:** Synthetic data generation, manipulation research

**ModelNet**

- **Description:** CAD model dataset for object recognition
- **Content:**
  - ModelNet10: 4,899 models, 10 categories
  - ModelNet40: 12,311 models, 40 categories
- **Format:** OFF (Object File Format)
- **Use Case:** 3D deep learning, point cloud processing
- **Access:** https://modelnet.cs.princeton.edu/

**3D Warehouse (SketchUp)**

- **Description:** Community 3D model repository
- **Content:** Millions of user-created models
- **Categories:** Architecture, furniture, machinery
- **Format:** SKP (SketchUp), COLLADA export
- **Access:** https://3dwarehouse.sketchup.com/
- **License:** Varies (check individual models)
- **Use Case:** Simulation environments

**Thingiverse**

- **Description:** 3D printable model repository
- **Content:** 2M+ designs (mechanical parts, tools, objects)
- **Format:** STL, OBJ, SCAD
- **Access:** https://www.thingiverse.com/
- **License:** Creative Commons (varies)
- **Use Case:** Robot parts, grippers, custom tools

**GrabCAD**

- **Description:** Professional CAD model library
- **Content:** 4.5M+ CAD models
- **Format:** STEP, STL, SOLIDWORKS, Inventor
- **Access:** https://grabcad.com/library
- **Quality:** Engineering-grade models
- **Use Case:** Robot design, gripper design

---

## E.4 Community Resources and Forums

### E.4.1 ROS Discourse

- **URL:** https://discourse.ros.org/
- **Description:** Official ROS community forum
- **Categories:**
  - General: ROS 2 discussions
  - Next Generation ROS: ROS 2 specific
  - Using ROS: User questions and tutorials
  - ROS Projects: Project showcases
  - Jobs: Career opportunities
- **Activity:** Very active, responses within hours
- **Moderation:** Official ROS team and community moderators

**Best Practices:**
- Search before posting (many common questions answered)
- Provide system info (ROS version, OS, hardware)
- Include error messages and logs
- Tag questions appropriately

### E.4.2 NVIDIA Isaac Forums

- **URL:** https://forums.developer.nvidia.com/c/agx-autonomous-machines/isaac/
- **Description:** Official NVIDIA Isaac support forum
- **Subcategories:**
  - Isaac Sim
  - Isaac ROS
  - Isaac SDK (legacy)
- **Support:** NVIDIA engineers respond regularly
- **Content:** Technical Q&A, bug reports, feature requests

**Related:**
- Omniverse Forums: https://forums.developer.nvidia.com/c/omniverse/
- Jetson Forums: https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/

### E.4.3 GitHub Repositories

**Awesome Robotics**

- **URL:** https://github.com/kiloreux/awesome-robotics
- **Description:** Curated list of robotics resources
- **Content:** Libraries, courses, papers, competitions
- **Topics:** ROS, simulators, vision, planning, control

**Awesome ROS 2**

- **URL:** https://github.com/fkromer/awesome-ros2
- **Description:** ROS 2 specific resources
- **Content:** Packages, tutorials, presentations, books
- **Updates:** Community-maintained, regularly updated

**Awesome Robot Descriptions**

- **URL:** https://github.com/robot-descriptions/awesome-robot-descriptions
- **Description:** Collection of robot URDF/MJCF models
- **Content:** 200+ robot descriptions
- **Format:** URDF, MJCF (MuJoCo)

**Open Robotics GitHub**

- **URL:** https://github.com/osrf (Open Robotics), https://github.com/ros2
- **Content:** Official ROS/Gazebo repositories
- **Examples:**
  - ros2/ros2: ROS 2 meta-repository
  - gazebosim: Gazebo simulation
  - osrf/urdf_tutorial: URDF learning resources

### E.4.4 Discord Communities

**ROS Discord**

- **Invite:** https://discord.gg/ros (check ROS Discourse for current link)
- **Members:** 5,000+
- **Channels:**
  - #ros2-help: Technical support
  - #showcase: Project demonstrations
  - #nav2: Navigation stack
  - #moveit: Motion planning
- **Activity:** Very active, real-time help

**Robotics & AI Discord**

- **Description:** Community for robotics enthusiasts
- **Topics:** Hobbyist and professional robotics
- **Channels:** Hardware, software, projects, careers

**Isaac Sim Community Discord**

- **Access:** Through NVIDIA Developer forums
- **Content:** Isaac Sim users, tips, troubleshooting
- **Activity:** Growing community

### E.4.5 Research Conferences

**Major Robotics Conferences:**

| Conference | Acronym | Focus | Deadline (typical) | Event (typical) |
|------------|---------|-------|-------------------|-----------------|
| International Conference on Robotics and Automation | ICRA | Broad robotics | October | May-June |
| IEEE/RSJ International Conference on Intelligent Robots and Systems | IROS | Intelligent systems | March | September-October |
| Robotics: Science and Systems | RSS | Robotics theory | January | July |
| Conference on Robot Learning | CoRL | Learning for robots | June | November |
| Humanoids | Humanoids | Humanoid robotics | June | November |
| International Conference on Computer Vision | ICCV | Vision (biennial) | March | October |
| Computer Vision and Pattern Recognition | CVPR | Vision | November | June |

**Conference Resources:**

- **Paper archives:** IEEE Xplore, arXiv.org
- **Video presentations:** YouTube channels (e.g., ICRA, RSS)
- **Workshop papers:** Often on conference websites

**Following Conferences:**

- Subscribe to mailing lists for CFPs (Call for Papers)
- Follow on Twitter/X for announcements
- Attend virtually (many offer online participation)
- Review open-access papers on arXiv

---

## E.5 Recommended Reading and Papers

### E.5.1 Classic Robotics Papers

**Kinematics and Control:**

1. **"A Mathematical Introduction to Robotic Manipulation"**
   - Authors: Murray, Li, Sastry
   - Year: 1994
   - Topics: Kinematics, dynamics, control
   - Access: http://www.cds.caltech.edu/~murray/mlswiki/

2. **"Robot Dynamics and Control"**
   - Authors: Spong, Hutchinson, Vidyasagar
   - Year: 1989 (2nd ed. 2006)
   - Topics: Dynamics, trajectory planning, control
   - Classic textbook reference

3. **"Probabilistic Robotics"**
   - Authors: Thrun, Burgard, Fox
   - Year: 2005
   - Topics: Localization, SLAM, Kalman/particle filters
   - Essential for mobile robotics
   - Free online: https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf

**Motion Planning:**

4. **"A Randomized Approach to Robot Path Planning"**
   - Authors: Kavraki et al.
   - Year: 1996
   - Topic: Probabilistic Roadmaps (PRM)
   - Citation: Foundation of sampling-based planning

5. **"Randomized Kinodynamic Planning"**
   - Authors: LaValle, Kuffner
   - Year: 2001
   - Topic: Rapidly-exploring Random Trees (RRT)
   - Impact: Enabled planning for high-DOF robots

**SLAM:**

6. **"Real-Time Appearance-Based Mapping"**
   - Authors: Se, Lowe, Little
   - Year: 2002
   - Topic: Visual SLAM with SIFT features

7. **"ORB-SLAM: A Versatile and Accurate Monocular SLAM System"**
   - Authors: Mur-Artal, Montiel, Tardos
   - Year: 2015
   - Impact: Widely-used visual SLAM
   - Code: https://github.com/raulmur/ORB_SLAM2

### E.5.2 Recent Physical AI Papers

**Simulation and Sim-to-Real:**

1. **"Sim-to-Real Transfer of Robotic Control with Dynamics Randomization"**
   - Authors: Peng et al.
   - Year: 2018, ICRA
   - Topic: Domain randomization for transfer
   - arXiv: https://arxiv.org/abs/1710.06537

2. **"Learning Dexterous In-Hand Manipulation"**
   - Authors: OpenAI et al.
   - Year: 2019, IJRR
   - Topic: Dexterous manipulation with domain randomization
   - arXiv: https://arxiv.org/abs/1808.00177

3. **"Isaac Gym: High Performance GPU-Based Physics Simulation"**
   - Authors: Makoviychuk et al.
   - Year: 2021, NeurIPS
   - Topic: Massively parallel RL training
   - arXiv: https://arxiv.org/abs/2108.10470

**Learning-Based Manipulation:**

4. **"Deep Imitation Learning for Complex Manipulation Tasks from Virtual Reality Teleoperation"**
   - Authors: Zhang et al.
   - Year: 2018, ICRA
   - Topic: VR teleoperation for data collection
   - arXiv: https://arxiv.org/abs/1710.04615

5. **"Learning Synergies between Pushing and Grasping with Self-Supervised Deep Reinforcement Learning"**
   - Authors: Zeng et al.
   - Year: 2018, IROS
   - Topic: Push-grasp learning
   - arXiv: https://arxiv.org/abs/1803.09956

6. **"Transporter Networks: Rearranging the Visual World for Robotic Manipulation"**
   - Authors: Zeng et al.
   - Year: 2021, CoRL
   - Topic: Spatial action representations
   - arXiv: https://arxiv.org/abs/2010.14406

**Humanoid Locomotion:**

7. **"Learning Bipedal Walking On Planned Footsteps For Humanoid Robots"**
   - Authors: Peng et al.
   - Year: 2020, CoRL
   - Topic: Deep RL for humanoid walking
   - arXiv: https://arxiv.org/abs/2011.10928

8. **"Learning Locomotion Skills Using DeepMimic"**
   - Authors: Peng et al.
   - Year: 2018, SIGGRAPH
   - Topic: Motion imitation for locomotion
   - arXiv: https://arxiv.org/abs/1804.02717

9. **"Whole-Body Humanoid Robot Locomotion with Human Reference"**
   - Authors: Radosavovic et al.
   - Year: 2024, arXiv
   - Topic: Human motion retargeting to humanoid
   - arXiv: https://arxiv.org/abs/2402.04436

**Vision-Language-Action Models:**

10. **"RT-1: Robotics Transformer for Real-World Control at Scale"**
    - Authors: Brohan et al. (Google)
    - Year: 2022, RSS
    - Topic: Transformer for robot control
    - arXiv: https://arxiv.org/abs/2212.06817

11. **"RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control"**
    - Authors: Brohan et al. (Google DeepMind)
    - Year: 2023, CoRL
    - Topic: VLA models for robotics
    - arXiv: https://arxiv.org/abs/2307.15818

12. **"Open X-Embodiment: Robotic Learning Datasets and RT-X Models"**
    - Authors: Open X-Embodiment Collaboration
    - Year: 2023, arXiv
    - Topic: Large-scale multi-robot dataset
    - arXiv: https://arxiv.org/abs/2310.08864

### E.5.3 Textbooks and Tutorials

**Foundational Textbooks:**

1. **"Introduction to Robotics: Mechanics and Control" (4th Edition)**
   - Author: John J. Craig
   - Publisher: Pearson, 2017
   - Topics: Kinematics, dynamics, trajectory planning, control
   - Level: Undergraduate

2. **"Robotics, Vision and Control: Fundamental Algorithms in MATLAB" (3rd Edition)**
   - Author: Peter Corke
   - Publisher: Springer, 2023
   - Topics: Complete robotics toolkit with MATLAB code
   - Companion: Robotics Toolbox for MATLAB/Python
   - Access: https://petercorke.com/rvc/

3. **"Modern Robotics: Mechanics, Planning, and Control"**
   - Authors: Kevin Lynch, Frank Park
   - Publisher: Cambridge University Press, 2017
   - Topics: Screw theory, kinematics, dynamics
   - Free: http://modernrobotics.org (videos and book)

4. **"Planning Algorithms"**
   - Author: Steven LaValle
   - Publisher: Cambridge University Press, 2006
   - Topics: Comprehensive motion planning
   - Free: http://planning.cs.uiuc.edu/

**ROS and Practical Guides:**

5. **"Programming Robots with ROS: A Practical Introduction"**
   - Authors: Quigley, Gerkey, Smart
   - Publisher: O'Reilly, 2015
   - Topics: ROS 1 fundamentals (concepts apply to ROS 2)

6. **"A Systematic Approach to Learning Robot Programming with ROS 2"**
   - Authors: Newbury, Bohren, Robinson
   - Publisher: CRC Press, 2024
   - Topics: ROS 2 development from basics to advanced

7. **"ROS 2 Tutorials" (Official)**
   - Access: https://docs.ros.org/en/humble/Tutorials.html
   - Content: Beginner to advanced tutorials
   - Format: Online, free

**Deep Learning for Robotics:**

8. **"Deep Learning for Robot Perception and Cognition"**
   - Authors: Piater et al.
   - Publisher: Academic Press, 2022
   - Topics: Vision, learning, semantic understanding

9. **"Reinforcement Learning for Robotics"** (Online Course)
   - Platform: Coursera, edX, YouTube
   - Instructors: Pieter Abbeel (UC Berkeley), Sergey Levine (UC Berkeley)
   - Topics: Deep RL, policy gradients, sim-to-real

**Hands-On Resources:**

10. **"Practical Robotics in C++"**
    - Author: Lloyd Brombach
    - Year: 2021
    - Topics: ROS, OpenCV, hardware interfacing
    - Code: Extensive examples

11. **"Robot Operating System (ROS) for Absolute Beginners"**
    - Author: Lentin Joseph
    - Publisher: Apress, 2022
    - Topics: ROS fundamentals with projects
    - Code: GitHub examples

### E.5.4 Online Courses and Tutorials

**Coursera:**

- **"Modern Robotics" Specialization** (Northwestern University)
  - Instructor: Kevin Lynch
  - Duration: 6 courses
  - Topics: Kinematics, dynamics, planning, control
  - Certificate: Available

- **"Robotics" Specialization** (University of Pennsylvania)
  - Duration: 5 courses
  - Topics: Aerial, autonomous, perception, estimation, mobility

**edX:**

- **"Robotics MicroMasters" (University of Pennsylvania)**
  - Duration: 4 courses
  - Topics: Kinematics, mobility, perception, estimation, learning

**YouTube Channels:**

- **MATLAB:** Robotics tutorials and examples
- **Articulated Robotics:** Practical ROS 2 tutorials
- **The Construct:** ROS learning platform
- **Jeremy Morgan:** ROS 2 tutorials

**Hands-On Platforms:**

- **The Construct Sim** (https://www.theconstructsim.com/)
  - Online ROS development environment
  - Curated courses and projects
  - Simulation included

- **Robot Ignite Academy**
  - Structured ROS 2 learning paths
  - Simulation-based exercises

---

## Summary

This appendix provided comprehensive resource listings for Physical AI development:

- **Datasets**: Manipulation (YCB, ACRONYM), Navigation (TUM, EuRoC, KITTI), Human Motion (CMU, Human3.6M, AMASS), Grasping (Dex-Net, ContactDB)
- **Pre-trained Models**: Object detection (YOLO, Detectron2), segmentation (SAM, Mask R-CNN), pose estimation (OpenPose, HRNet), speech (Whisper, Vosk)
- **3D Assets**: Robot URDFs (ROS-Industrial, Clearpath, Unitree), environments (Gazebo, AWS, NVIDIA), object meshes (ShapeNet, ModelNet)
- **Community**: Forums (ROS Discourse, NVIDIA), GitHub repositories, Discord servers, research conferences
- **Reading**: Classic papers (Probabilistic Robotics, SLAM), recent work (sim-to-real, VLA models), textbooks (Craig, Lynch, Corke), online courses

These resources enable effective research, development, and continuous learning in robotics. Bookmark key repositories and join active communities to stay current with rapidly evolving Physical AI technologies.

**Recommended Starting Points:**

1. New to ROS 2: Official tutorials + Articulated Robotics YouTube
2. Need datasets: Start with COCO (vision), YCB (manipulation), TUM (SLAM)
3. Pre-trained models: PyTorch Hub, Hugging Face, NVIDIA NGC
4. Community help: ROS Discourse (async), ROS Discord (real-time)
5. Deep dive: Modern Robotics textbook + online course

Stay engaged with conferences (ICRA, IROS, RSS) and follow key researchers on arXiv for latest developments.
