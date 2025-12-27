/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // Main textbook sidebar
  textbookSidebar: [
    {
      type: 'doc',
      id: 'introduction',
      label: 'About This Textbook',
    },

    // Part I: Foundations of Physical AI (Weeks 1-2)
    {
      type: 'category',
      label: 'Part I: Foundations of Physical AI',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'chapters/chapter-01-introduction-to-physical-ai',
          label: 'Chapter 1: Introduction to Physical AI',
        },
        {
          type: 'doc',
          id: 'chapters/chapter-02-sensor-systems-for-physical-ai',
          label: 'Chapter 2: Sensor Systems',
        },
      ],
    },

    // Part II: The Robotic Nervous System (Weeks 3-5)
    {
      type: 'category',
      label: 'Part II: The Robotic Nervous System',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'chapters/chapter-03-introduction-to-ros2',
          label: 'Chapter 3: Introduction to ROS 2',
        },
        {
          type: 'doc',
          id: 'chapters/chapter-04-building-with-ros2',
          label: 'Chapter 4: Building with ROS 2',
        },
        {
          type: 'doc',
          id: 'chapters/chapter-05-ros2-for-humanoid-robots',
          label: 'Chapter 5: ROS 2 for Humanoid Robots',
        },
      ],
    },

    // Part III: The Digital Twin (Weeks 6-7)
    {
      type: 'category',
      label: 'Part III: The Digital Twin',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'chapters/chapter-06-physics-simulation-with-gazebo',
          label: 'Chapter 6: Physics Simulation with Gazebo',
        },
        {
          type: 'doc',
          id: 'chapters/chapter-07-high-fidelity-simulation-with-unity',
          label: 'Chapter 7: High-Fidelity Simulation with Unity',
        },
      ],
    },

    // Part IV: The AI-Robot Brain (Weeks 8-10)
    {
      type: 'category',
      label: 'Part IV: The AI-Robot Brain',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'chapters/chapter-08-nvidia-isaac-platform',
          label: 'Chapter 8: NVIDIA Isaac Platform',
        },
        {
          type: 'doc',
          id: 'chapters/chapter-09-isaac-ros-hardware-accelerated-perception',
          label: 'Chapter 9: Isaac ROS - Hardware-Accelerated Perception',
        },
        {
          type: 'doc',
          id: 'chapters/chapter-10-navigation-and-path-planning',
          label: 'Chapter 10: Navigation and Path Planning',
        },
      ],
    },

    // Part V: Humanoid Robot Development (Weeks 11-12)
    {
      type: 'category',
      label: 'Part V: Humanoid Robot Development',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'chapters/chapter-11-humanoid-robot-kinematics-and-dynamics',
          label: 'Chapter 11: Kinematics and Dynamics',
        },
        {
          type: 'doc',
          id: 'chapters/chapter-12-bipedal-locomotion-and-balance',
          label: 'Chapter 12: Bipedal Locomotion and Balance',
        },
        {
          type: 'doc',
          id: 'chapters/chapter-13-manipulation-and-grasping',
          label: 'Chapter 13: Manipulation and Grasping',
        },
        {
          type: 'doc',
          id: 'chapters/chapter-14-natural-human-robot-interaction',
          label: 'Chapter 14: Natural Human-Robot Interaction',
        },
      ],
    },

    // Part VI: Vision-Language-Action (Week 13)
    {
      type: 'category',
      label: 'Part VI: Vision-Language-Action',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'chapters/chapter-15-conversational-robotics',
          label: 'Chapter 15: Conversational Robotics',
        },
      ],
    },

    // Part VII: Integration and Deployment
    {
      type: 'category',
      label: 'Part VII: Integration and Deployment',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'chapters/chapter-16-sim-to-real-transfer',
          label: 'Chapter 16: Sim-to-Real Transfer',
        },
        {
          type: 'doc',
          id: 'chapters/chapter-17-edge-computing-for-physical-ai',
          label: 'Chapter 17: Edge Computing for Physical AI',
        },
      ],
    },

    // Part VIII: Capstone Project
    {
      type: 'category',
      label: 'Part VIII: Capstone Project',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'chapters/chapter-18-the-autonomous-humanoid',
          label: 'Chapter 18: The Autonomous Humanoid',
        },
      ],
    },

    // Appendices
    {
      type: 'category',
      label: 'Appendices',
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'chapters/appendix-a-hardware-setup-guides',
          label: 'Appendix A: Hardware Setup Guides',
        },
        {
          type: 'doc',
          id: 'chapters/appendix-b-software-installation',
          label: 'Appendix B: Software Installation',
        },
        {
          type: 'doc',
          id: 'chapters/appendix-c-reference-materials',
          label: 'Appendix C: Reference Materials',
        },
        {
          type: 'doc',
          id: 'chapters/appendix-d-mathematical-foundations',
          label: 'Appendix D: Mathematical Foundations',
        },
        {
          type: 'doc',
          id: 'chapters/appendix-e-datasets-and-resources',
          label: 'Appendix E: Datasets and Resources',
        },
      ],
    },
  ],

  // Alternative: Weekly-based sidebar (if preferred)
  weeklySidebar: [
    {
      type: 'doc',
      id: 'introduction',
      label: 'Course Overview',
    },
    {
      type: 'category',
      label: 'Weeks 1-2: Foundations',
      items: [
        'chapters/chapter-01-introduction-to-physical-ai',
        'chapters/chapter-02-sensor-systems-for-physical-ai',
      ],
    },
    {
      type: 'category',
      label: 'Weeks 3-5: ROS 2 Fundamentals',
      items: [
        'chapters/chapter-03-introduction-to-ros2',
        'chapters/chapter-04-building-with-ros2',
        'chapters/chapter-05-ros2-for-humanoid-robots',
      ],
    },
    {
      type: 'category',
      label: 'Weeks 6-7: Simulation',
      items: [
        'chapters/chapter-06-physics-simulation-with-gazebo',
        'chapters/chapter-07-high-fidelity-simulation-with-unity',
      ],
    },
    {
      type: 'category',
      label: 'Weeks 8-10: NVIDIA Isaac Platform',
      items: [
        'chapters/chapter-08-nvidia-isaac-platform',
        'chapters/chapter-09-isaac-ros-hardware-accelerated-perception',
        'chapters/chapter-10-navigation-and-path-planning',
      ],
    },
    {
      type: 'category',
      label: 'Weeks 11-12: Humanoid Development',
      items: [
        'chapters/chapter-11-humanoid-robot-kinematics-and-dynamics',
        'chapters/chapter-12-bipedal-locomotion-and-balance',
        'chapters/chapter-13-manipulation-and-grasping',
        'chapters/chapter-14-natural-human-robot-interaction',
      ],
    },
    {
      type: 'category',
      label: 'Week 13: Conversational AI',
      items: [
        'chapters/chapter-15-conversational-robotics',
      ],
    },
    {
      type: 'category',
      label: 'Final Weeks: Deployment & Capstone',
      items: [
        'chapters/chapter-16-sim-to-real-transfer',
        'chapters/chapter-17-edge-computing-for-physical-ai',
        'chapters/chapter-18-the-autonomous-humanoid',
      ],
    },
    {
      type: 'category',
      label: 'Reference Materials',
      items: [
        'chapters/appendix-a-hardware-setup-guides',
        'chapters/appendix-b-software-installation',
        'chapters/appendix-c-reference-materials',
        'chapters/appendix-d-mathematical-foundations',
        'chapters/appendix-e-datasets-and-resources',
      ],
    },
  ],
};

module.exports = sidebars;
