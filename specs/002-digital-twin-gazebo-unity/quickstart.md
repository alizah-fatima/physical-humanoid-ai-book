---
description: "Quickstart guide for Module 2: The Digital Twin (Gazebo & Unity) - Getting started with the content and implementation"
---

# Module 2: The Digital Twin (Gazebo & Unity) - Quickstart Guide

## Overview

This quickstart guide provides a fast path to get started with the Digital Twin module covering Gazebo simulation, URDF/SDF modeling, and Unity integration for humanoid robotics. This guide is designed for users who want to quickly understand and implement the core concepts.

## Prerequisites

### System Requirements
- Ubuntu 20.04/22.04 or equivalent Linux distribution
- ROS 2 Humble Hawksbill installed
- Gazebo Garden or Fortress
- Unity Hub with Unity 2022.3 LTS or newer
- At least 8GB RAM (16GB recommended for complex humanoid models)
- Dedicated GPU with OpenGL 4.5+ support

### Software Dependencies
```bash
# Install Gazebo and ROS 2 integration
sudo apt update
sudo apt install gazebo libgazebo-dev
sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-plugins ros-humble-gazebo-dev

# Install ROS 2 development tools
sudo apt install ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher
```

## Getting Started with Gazebo Simulation

### 1. Launch Basic Gazebo Environment
```bash
# Start Gazebo with default world
gz sim

# Or with a specific world
gz sim -r -v 4 empty.sdf
```

### 2. Create Your First Humanoid Model (URDF)
Create a simple humanoid model in `my_humanoid.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.1"/>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.1"/>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Torso -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.2"/>
  </joint>

  <link name="torso">
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.3"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.3"/>
      <geometry>
        <box size="0.2 0.15 0.6"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.3"/>
      <geometry>
        <box size="0.2 0.15 0.6"/>
      </geometry>
    </collision>
  </link>
</robot>
```

### 3. Load Model into Gazebo
```bash
# Convert URDF to SDF and spawn in Gazebo
gz sdf -p my_humanoid.urdf > my_humanoid.sdf
gz sim -r my_humanoid.sdf
```

## Setting Up Sensor Simulation

### 1. Add LiDAR to Your Robot
Add this to your URDF file:

```xml
<!-- LiDAR sensor -->
<joint name="lidar_joint" type="fixed">
  <parent link="torso"/>
  <child link="lidar_link"/>
  <origin xyz="0.1 0 0.2"/>
</joint>

<link name="lidar_link">
  <inertial>
    <mass value="0.1"/>
    <inertial ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-1.57</min_angle>
          <max_angle>1.57</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
      </range>
    </ray>
    <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

### 2. Add IMU Sensor
```xml
<gazebo reference="torso">
  <sensor name="imu" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <ros>
        <remapping>~/out:=imu/data</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

## Unity Integration Setup

### 1. Install Unity ROS TCP Connector
1. Download the Unity ROS TCP Connector package
2. Import into your Unity project
3. Configure ROS connection settings

### 2. Basic Unity Robot Controller Script
```csharp
using UnityEngine;
using ROS2;

public class RobotController : MonoBehaviour
{
    private ROS2UnityComponent ros2;
    private ISubscription<std_msgs.msg.Float64MultiArray> jointSub;

    void Start()
    {
        ros2 = GetComponent<ROS2UnityComponent>();
        ros2.Init();

        // Subscribe to joint states
        jointSub = ros2.node.CreateSubscription<std_msgs.msg.Float64MultiArray>
            ("/joint_states/position", JointStateCallback);
    }

    void JointStateCallback(std_msgs.msg.Float64MultiArray msg)
    {
        // Update robot joint positions based on ROS messages
        UpdateRobotJoints(msg.data);
    }

    void UpdateRobotJoints(double[] jointPositions)
    {
        // Apply joint positions to Unity robot model
        // Implementation depends on your robot hierarchy
    }
}
```

## Running the Complete Pipeline

### 1. Start ROS 2 Environment
```bash
# Terminal 1: Start ROS 2 daemon
source /opt/ros/humble/setup.bash
ros2 daemon start

# Terminal 2: Launch Gazebo with your robot
ros2 launch my_robot_gazebo my_robot_world.launch.py

# Terminal 3: Visualize in RViz
ros2 run rviz2 rviz2
```

### 2. Connect Unity Visualization
1. Start your Unity project with ROS connection
2. Ensure Unity and Gazebo are synchronized
3. Verify sensor data is flowing correctly

## Basic Humanoid Control Example

### 1. Create Joint Controller
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        self.publisher = self.create_publisher(
            Float64MultiArray, '/joint_commands', 10)

        # Timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)
        self.time = 0.0

    def control_loop(self):
        msg = Float64MultiArray()

        # Simple oscillating joint pattern
        joints = [
            math.sin(self.time),      # hip joint
            math.cos(self.time),      # knee joint
            math.sin(self.time * 0.5) # ankle joint
        ]

        msg.data = joints
        self.publisher.publish(msg)
        self.time += 0.1

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Run the Controller
```bash
chmod +x humanoid_controller.py
ros2 run my_robot_control humanoid_controller.py
```

## Verification Steps

### 1. Check Gazebo Simulation
- Verify robot model loads correctly
- Check physics behavior is stable
- Confirm sensors are publishing data

### 2. Validate Sensor Data
```bash
# Check available topics
ros2 topic list | grep sensor

# Monitor LiDAR data
ros2 topic echo /scan

# Monitor IMU data
ros2 topic echo /imu/data
```

### 3. Unity Connection Test
- Verify Unity connects to ROS network
- Check robot model updates in Unity
- Confirm sensor visualization works

## Troubleshooting Quick Fixes

### Common Gazebo Issues
- **Robot falls through ground**: Check collision geometry and mass properties
- **Slow performance**: Reduce model complexity or adjust physics parameters
- **Sensors not publishing**: Verify plugin configuration and topic names

### Common Unity Issues
- **No connection**: Check ROS network configuration
- **Model not updating**: Verify joint name mapping
- **Synchronization issues**: Check time synchronization

## Next Steps

After completing this quickstart:

1. **Explore Chapter 1**: Deep dive into Gazebo physics simulation
2. **Chapter 2**: Master URDF/SDF modeling for complex humanoids
3. **Chapter 3**: Implement advanced sensor simulation and Unity integration
4. **Build your own humanoid**: Apply learned concepts to create custom models

## Resources

- Gazebo Documentation: http://gazebosim.org/
- ROS 2 Documentation: https://docs.ros.org/
- Unity Integration: Check the textbook chapters for detailed guides
- Sample Code: Available in the textbook examples

This quickstart provides the foundation to explore the comprehensive content in the Digital Twin module, covering all aspects of simulation, modeling, and visualization for humanoid robotics.