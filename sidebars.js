// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module1-ros2/chapter1-architecture',
        'module1-ros2/chapter2-python-packages',
        'module1-ros2/chapter3-urdf-modeling',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module2-digital-twin/chapter1-gazebo-intro',
        'module2-digital-twin/chapter2-urdf-sdf-modeling',
        'module2-digital-twin/chapter3-sensor-unity-integration',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module3-nvidia-isaac/chapter1-isaac-sim-intro',
        'module3-nvidia-isaac/chapter2-isaac-ros-tools',
        'module3-nvidia-isaac/chapter3-nav2-bipedal-planning',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module4-vla/chapter1-vla-intro',
        'module4-vla/chapter2-voice-to-action',
        'module4-vla/chapter3-cognitive-planning',
      ],
    },
  ],
};

module.exports = sidebars;