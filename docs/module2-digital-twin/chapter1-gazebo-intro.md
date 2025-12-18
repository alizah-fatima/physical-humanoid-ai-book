---
sidebar_position: 1
---

# Chapter 1: Introduction to Gazebo - Physics Simulation for Humanoid Robotics

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the fundamentals of Gazebo simulation environment
- Set up Gazebo for humanoid robotics simulation
- Configure physics properties including gravity and collision detection
- Build custom simulation environments for humanoid robots
- Integrate Gazebo with ROS 2 for realistic robot simulation

## Introduction to Gazebo

Gazebo is a powerful open-source 3D robotics simulator that provides realistic physics simulation, high-quality graphics, and convenient programmatic interfaces. It's widely used in robotics research and development for testing algorithms, robot design, and training AI agents in a safe virtual environment before deployment on real robots.

For humanoid robotics, Gazebo offers:
- Accurate physics simulation with multiple physics engines (ODE, Bullet, Simbody)
- Realistic sensor simulation (cameras, LiDAR, IMUs, force/torque sensors)
- Flexible environment modeling with building editor
- Integration with ROS/ROS 2 through gazebo_ros packages
- Support for complex humanoid models with multiple degrees of freedom

## Installing and Setting Up Gazebo

### System Requirements

Before installing Gazebo, ensure your system meets the requirements:
- Ubuntu 20.04/22.04 or newer (recommended for ROS 2 compatibility)
- Graphics card with OpenGL 2.1+ support
- Minimum 4GB RAM (8GB+ recommended for complex humanoid simulations)
- Multi-core processor for optimal performance

### Installation

For ROS 2 Humble Hawksbill (Ubuntu 22.04):

```bash
sudo apt update
sudo apt install gazebo libgazebo-dev
# For ROS 2 integration
sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-plugins ros-humble-gazebo-dev
```

### Basic Gazebo Launch

To launch Gazebo with an empty world:

```bash
gz sim
# Or for older versions
gazebo
```

## Physics Simulation Fundamentals

### Physics Engines

Gazebo supports multiple physics engines, each with different characteristics:

1. **ODE (Open Dynamics Engine)**: Default engine, good balance of speed and accuracy
2. **Bullet**: Good for complex collision detection and articulated bodies
3. **Simbody**: High accuracy for biomechanics and complex articulated systems

### Configuring Physics Properties

The physics properties are defined in the world file (SDF format):

```xml
<sdf version="1.7">
  <world name="default">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Environment content -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```

### Gravity Configuration

Gravity is a crucial parameter for humanoid simulation. The default value is -9.8 m/s² on the Z-axis:

```xml
<gravity>0 0 -9.8</gravity>
```

For humanoid robots, you might want to adjust:
- Gravity strength for different planetary environments
- Direction for testing non-standard scenarios
- Temporarily disable for specific testing purposes

### Time Step Configuration

The physics simulation accuracy depends on the time step settings:

```xml
<max_step_size>0.001</max_step_size>        <!-- Simulation time step (seconds) -->
<real_time_update_rate>1000.0</real_time_update_rate>  <!-- Updates per second -->
<real_time_factor>1.0</real_time_factor>    <!-- Simulation speed multiplier -->
```

For humanoid robots requiring precise control:
- Smaller time steps (0.001s) for better accuracy
- Higher update rates for responsive control
- Real-time factor of 1.0 for real-time simulation

## Collision Detection and Dynamics

### Collision Properties

Each link in a robot model needs proper collision properties:

```xml
<link name="link_name">
  <collision name="collision">
    <geometry>
      <box>
        <size>0.1 0.1 0.1</size>
      </box>
    </geometry>
    <surface>
      <friction>
        <ode>
          <mu>1.0</mu>    <!-- Static friction coefficient -->
          <mu2>1.0</mu2>  <!-- Secondary friction coefficient -->
        </ode>
      </friction>
      <bounce>
        <restitution_coefficient>0.0</restitution_coefficient>  <!-- Bounciness -->
        <threshold>100000.0</threshold>  <!-- Velocity threshold for bouncing -->
      </bounce>
      <contact>
        <ode>
          <kp>1e+16</kp>  <!-- Contact stiffness -->
          <kd>1e+12</kd>  <!-- Contact damping -->
          <max_vel>100.0</max_vel>  <!-- Maximum contact correction velocity -->
          <min_depth>0.001</min_depth>  <!-- Minimum contact depth -->
        </ode>
      </contact>
    </surface>
  </collision>
</link>
```

### Material Properties for Humanoid Simulation

For realistic humanoid interaction with the environment:

```xml
<!-- Feet with good grip for walking -->
<collision name="left_foot_collision">
  <geometry>
    <box>
      <size>0.15 0.08 0.01</size>
    </box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.5</mu>   <!-- High friction for stable walking -->
        <mu2>1.5</mu2>
      </ode>
    </friction>
  </surface>
</collision>
```

## Building Custom Simulation Environments

### Using Gazebo's Building Editor

Gazebo provides a building editor for creating custom environments:

1. Launch Gazebo with the building plugin:
```bash
gz sim -r -v 4 building_warehouse.sdf
```

2. Use the building editor to create:
   - Rooms with walls, doors, windows
   - Furniture and obstacles
   - Multi-story buildings
   - Complex architectural structures

### Creating Custom World Files

Basic world file structure:

```xml
<sdf version="1.7">
  <world name="humanoid_training_world">
    <!-- Physics configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Custom models -->
    <include>
      <uri>model://my_humanoid_robot</uri>
      <pose>0 0 1 0 0 0</pose>
    </include>

    <!-- Obstacles for training -->
    <model name="training_obstacle_1">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>1 0.3 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Environment Elements for Humanoid Training

#### Flat Ground Area
```xml
<!-- Large flat area for basic locomotion training -->
<model name="training_area">
  <pose>0 0 0 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>20 20 0.1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>20 20 0.1</size>
        </box>
      </geometry>
      <material>
        <ambient>0.6 0.6 0.6 1</ambient>
        <diffuse>0.7 0.7 0.7 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

#### Obstacle Course
```xml
<!-- Various obstacles for navigation training -->
<model name="narrow_passage">
  <pose>5 0 0.5 0 0 0</pose>
  <link name="left_wall">
    <collision name="collision">
      <geometry>
        <box>
          <size>0.1 2.0 2.0</size>
        </box>
      </geometry>
    </collision>
  </link>
  <link name="right_wall">
    <pose>1.9 0 0 0 0 0</pose>
    <collision name="collision">
      <geometry>
        <box>
          <size>0.1 2.0 2.0</size>
        </box>
      </geometry>
    </collision>
  </link>
</model>
```

## Gazebo-ROS 2 Integration

### Launching Gazebo with ROS 2

To integrate Gazebo with ROS 2, use the gazebo_ros packages:

```xml
<!-- In your launch file -->
<launch>
  <!-- Start Gazebo server -->
  <node name="gazebo_server" pkg="gazebo_ros" exec="gzserver" args="$(find-pkg-share my_robot_gazebo)/worlds/my_world.sdf"/>

  <!-- Start Gazebo client -->
  <node name="gazebo_client" pkg="gazebo_ros" exec="gzclient"/>
</launch>
```

### Spawning Robots in Gazebo

Use the spawn_entity service to place robots in the simulation:

```python
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity

class RobotSpawner(Node):
    def __init__(self):
        super().__init__('robot_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def spawn_robot(self, robot_name, robot_xml, x, y, z, roll, pitch, yaw):
        req = SpawnEntity.Request()
        req.name = robot_name
        req.xml = robot_xml
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z
        req.initial_pose.orientation = self.euler_to_quaternion(roll, pitch, yaw)

        future = self.cli.call_async(req)
        return future

def main(args=None):
    rclpy.init(args=args)
    spawner = RobotSpawner()
    # Spawn your humanoid robot
    future = spawner.spawn_robot("my_humanoid", robot_xml_content, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    rclpy.spin_until_future_complete(spawner, future)
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Optimization for Humanoid Simulation

### Reducing Computational Load

For complex humanoid models with many joints:

```xml
<!-- Optimize physics for humanoid simulation -->
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <!-- Use fewer threads for better humanoid control responsiveness -->
  <threads>4</threads>
  <!-- Adjust solver iterations for balance between accuracy and speed -->
  <ode>
    <solver>
      <type>quick</type>
      <iters>100</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### Visual Optimization

```xml>
<!-- Optimize visual rendering for humanoid simulation -->
<scene>
  <grid>false</grid>  <!-- Disable grid for performance -->
  <shadows>false</shadows>  <!-- Optional: disable shadows for performance -->
  <ambient>0.4 0.4 0.4 1</ambient>  <!-- Adjust lighting for performance -->
</scene>
```

## Troubleshooting Common Issues

### Robot Falling Through Ground

Common causes and solutions:
- Check collision geometry in URDF/SDF
- Verify physics parameters (contact stiffness/damping)
- Ensure proper mass and inertia values

### Unstable Simulation

- Reduce time step size
- Adjust solver parameters
- Check joint limits and constraints

### Performance Issues

- Simplify collision geometry (use boxes instead of meshes)
- Reduce visual complexity
- Adjust physics parameters for speed vs. accuracy

## Summary

This chapter introduced the fundamentals of Gazebo simulation for humanoid robotics, including:
- Installation and setup procedures
- Physics simulation configuration with gravity and collision detection
- Environment building for humanoid robot training
- Integration with ROS 2 for complete simulation workflows
- Performance optimization techniques for complex humanoid models

Gazebo provides a robust platform for developing, testing, and training humanoid robots in a safe virtual environment before real-world deployment. The combination of accurate physics simulation, realistic sensor modeling, and ROS 2 integration makes it an essential tool for humanoid robotics development.