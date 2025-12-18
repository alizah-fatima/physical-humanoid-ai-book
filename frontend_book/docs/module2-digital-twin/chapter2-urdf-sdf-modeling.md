---
sidebar_position: 2
---

# Chapter 2: Robot Description Formats - URDF and SDF for Humanoid Modeling

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the differences between URDF and SDF formats
- Create humanoid robot models using both URDF and SDF
- Configure simulation-specific properties in SDF
- Convert between URDF and SDF formats when needed
- Optimize robot models for Gazebo simulation

## Introduction to Robot Description Formats

Robot description formats are essential for defining the physical and visual properties of robots in simulation environments. Two primary formats are used in robotics:

1. **URDF (Unified Robot Description Format)**: Primarily used in ROS/ROS 2 for robot description
2. **SDF (Simulation Description Format)**: Used by Gazebo for simulation-specific properties

Both formats define the robot's structure, but they serve different purposes in the robotics pipeline.

## URDF vs SDF: Key Differences

### URDF (Unified Robot Description Format)

URDF is an XML-based format that describes robot structure and kinematics. It's the standard for ROS/ROS 2 and focuses on:

- Kinematic structure (links and joints)
- Visual and collision properties
- Inertial properties
- Basic sensor information

### SDF (Simulation Description Format)

SDF is an XML-based format designed specifically for simulation. It includes:

- All URDF properties
- Simulation-specific physics parameters
- Gazebo plugins and extensions
- Environment-specific properties
- Advanced sensor simulation parameters

### When to Use Each Format

- **URDF**: When working with ROS/ROS 2, kinematic analysis, and robot control
- **SDF**: When working with Gazebo simulation, advanced physics, and simulation plugins
- **Conversion**: URDF can be converted to SDF for simulation using xacro and Gazebo tools

## URDF for Humanoid Robots

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Links (rigid bodies) -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Joints connecting links -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <inertial>
      <mass value="8.0"/>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.2 0.5"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.2 0.5"/>
      </geometry>
    </collision>
  </link>
</robot>
```

### Humanoid-Specific URDF Considerations

#### Joint Types for Humanoid Robots

```xml
<!-- Revolute joints for human-like movement -->
<joint name="left_hip_joint" type="revolute">
  <parent link="torso"/>
  <child link="left_thigh"/>
  <origin xyz="0.1 -0.1 -0.25" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="2"/>
  <dynamics damping="1.0" friction="0.1"/>
</joint>

<!-- Continuous joints for unlimited rotation -->
<joint name="left_knee_joint" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_shin"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="2.35" effort="100" velocity="2"/>
</joint>

<!-- Spherical joints for ball-and-socket movement (using 3 revolute joints) -->
<joint name="left_shoulder_joint_yaw" type="revolute">
  <parent link="torso"/>
  <child link="left_upper_arm_yaw"/>
  <origin xyz="0.15 0.1 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
</joint>
```

#### Mass Distribution for Humanoid Balance

```xml
<!-- Realistic mass distribution for stable simulation -->
<link name="torso">
  <inertial>
    <mass value="15.0"/> <!-- Heavier torso for stability -->
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <inertia ixx="0.8" ixy="0" ixz="0" iyy="0.6" iyz="0" izz="0.3"/>
  </inertial>
</link>

<link name="head">
  <inertial>
    <mass value="3.0"/> <!-- Realistic head mass -->
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
  </inertial>
</link>
```

## SDF for Gazebo Simulation

### Converting URDF to SDF

When URDF is loaded into Gazebo, it's automatically converted to SDF. However, you can also create native SDF files:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="humanoid_robot">
    <!-- Base link -->
    <link name="base_link">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.1</iyy>
          <iyz>0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>

      <visual name="visual">
        <geometry>
          <box>
            <size>0.3 0.3 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 1 1</ambient>
          <diffuse>0 0 1 1</diffuse>
        </material>
      </visual>

      <collision name="collision">
        <geometry>
          <box>
            <size>0.3 0.3 0.2</size>
          </box>
        </geometry>
      </collision>
    </link>

    <!-- Joint -->
    <joint name="torso_joint" type="fixed">
      <parent>base_link</parent>
      <child>torso</child>
    </joint>

    <link name="torso">
      <pose>0 0 0.3 0 0 0</pose>
      <inertial>
        <mass>8.0</mass>
        <inertia>
          <ixx>0.2</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.2</iyy>
          <iyz>0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>

      <visual name="visual">
        <geometry>
          <box>
            <size>0.25 0.2 0.5</size>
          </box>
        </geometry>
      </visual>

      <collision name="collision">
        <geometry>
          <box>
            <size>0.25 0.2 0.5</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

### Gazebo-Specific Extensions in SDF

#### Physics Properties

```xml
<link name="left_foot">
  <inertial>
    <mass>1.5</mass>
    <inertia>
      <ixx>0.005</ixx>
      <ixy>0</ixy>
      <ixz>0</ixz>
      <iyy>0.005</iyy>
      <iyz>0</iyz>
      <izz>0.001</izz>
    </inertia>
  </inertial>

  <collision name="collision">
    <geometry>
      <box>
        <size>0.15 0.08 0.01</size>
      </box>
    </geometry>
    <!-- Gazebo-specific surface properties -->
    <surface>
      <friction>
        <ode>
          <mu>1.5</mu>    <!-- High friction for stable walking -->
          <mu2>1.5</mu2>
        </ode>
      </friction>
      <bounce>
        <restitution_coefficient>0.0</restitution_coefficient>
      </bounce>
    </surface>
  </collision>
</link>
```

#### Gazebo Plugins

```xml
<!-- Gazebo plugins for simulation -->
<gazebo reference="left_foot">
  <material>Gazebo/Blue</material>
  <mu1>1.5</mu1>
  <mu2>1.5</mu2>
  <kp>1000000.0</kp>  <!-- Contact stiffness -->
  <kd>100.0</kd>      <!-- Contact damping -->
</gazebo>

<!-- Sensor plugin -->
<gazebo reference="head_camera">
  <sensor name="head_camera" type="camera">
    <camera name="head_camera">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
    </camera>
    <always_on>1</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
  </sensor>
</gazebo>
```

## Complete Humanoid Robot Model Example

### URDF with Xacro for Complex Humanoid

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931"/>
  <xacro:property name="torso_mass" value="15.0"/>
  <xacro:property name="head_mass" value="3.0"/>
  <xacro:property name="arm_mass" value="2.0"/>
  <xacro:property name="leg_mass" value="5.0"/>

  <!-- Base footprint -->
  <link name="base_footprint">
    <inertial>
      <mass value="0.0001"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Torso -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <inertial>
      <mass value="${torso_mass}"/>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <inertia ixx="0.8" ixy="0" ixz="0" iyy="0.6" iyz="0" izz="0.3"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.2 0.5"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.2 0.5"/>
      </geometry>
    </collision>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/4}" upper="${M_PI/4}" effort="10" velocity="1"/>
  </joint>

  <link name="head">
    <inertial>
      <mass value="${head_mass}"/>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="0.8 0.6 0.4 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_yaw" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm_yaw"/>
    <origin xyz="0.15 0.1 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="50" velocity="2"/>
  </joint>

  <link name="left_upper_arm_yaw">
    <inertial>
      <mass value="${arm_mass/3}"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.2"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.2"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="left_upper_arm_yaw"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI}" effort="50" velocity="2"/>
  </joint>

  <link name="left_upper_arm">
    <inertial>
      <mass value="${arm_mass/2}"/>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-M_PI}" upper="${M_PI/2}" effort="30" velocity="2"/>
  </joint>

  <link name="left_lower_arm">
    <inertial>
      <mass value="${arm_mass/4}"/>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.0005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Leg -->
  <joint name="left_hip_yaw" type="revolute">
    <parent link="torso"/>
    <child link="left_thigh_yaw"/>
    <origin xyz="0.05 -0.05 -0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-M_PI/4}" upper="${M_PI/4}" effort="100" velocity="2"/>
  </joint>

  <link name="left_thigh_yaw">
    <inertial>
      <mass value="${leg_mass/5}"/>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.002"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_hip_pitch" type="revolute">
    <parent link="left_thigh_yaw"/>
    <child link="left_thigh"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="2"/>
  </joint>

  <link name="left_thigh">
    <inertial>
      <mass value="${leg_mass*2/3}"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.08" ixy="0" ixz="0" iyy="0.08" iyz="0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_knee" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="${M_PI/2}" effort="100" velocity="2"/>
  </joint>

  <link name="left_shin">
    <inertial>
      <mass value="${leg_mass/3}"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.04" ixy="0" ixz="0" iyy="0.04" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_ankle" type="revolute">
    <parent link="left_shin"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-M_PI/4}" upper="${M_PI/4}" effort="50" velocity="1"/>
  </joint>

  <link name="left_foot">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0.05 0 -0.05" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.01"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.01"/>
      </geometry>
    </collision>
  </link>

  <!-- Right side (mirrored) would follow similar pattern -->

</robot>
```

### SDF Version with Gazebo Plugins

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="simple_humanoid">
    <!-- Base footprint -->
    <link name="base_footprint">
      <inertial>
        <mass>0.0001</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0001</iyy>
          <iyz>0</iyz>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
    </link>

    <!-- Base link -->
    <link name="base_link">
      <pose>0 0 0.05 0 0 0</pose>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.1</length>
          </cylinder>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.1</length>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <joint name="base_joint" type="fixed">
      <parent>base_footprint</parent>
      <child>base_link</child>
    </joint>

    <!-- Torso -->
    <link name="torso">
      <pose>0 0 0.2 0 0 0</pose>
      <inertial>
        <mass>15.0</mass>
        <inertia>
          <ixx>0.8</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.6</iyy>
          <iyz>0</iyz>
          <izz>0.3</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.25 0.2 0.5</size>
          </box>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.25 0.2 0.5</size>
          </box>
        </geometry>
      </collision>
    </link>

    <joint name="torso_joint" type="fixed">
      <parent>base_link</parent>
      <child>torso</child>
    </joint>

    <!-- Add Gazebo-specific configurations -->
    <gazebo reference="torso">
      <material>Gazebo/Grey</material>
    </gazebo>

    <gazebo reference="left_foot">
      <material>Gazebo/Blue</material>
      <mu1>1.5</mu1>
      <mu2>1.5</mu2>
    </gazebo>

    <!-- ROS 2 Control plugin for joint control -->
    <gazebo>
      <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
        <parameters>$(find my_robot_gazebo)/config/my_robot_control.yaml</parameters>
      </plugin>
    </gazebo>
  </model>
</sdf>
```

## Converting Between URDF and SDF

### Using xacro to Generate SDF from URDF

```bash
# Convert URDF to SDF using xacro
xacro robot.urdf.xacro > robot.sdf

# Or directly convert if you have a URDF file
gz sdf -p robot.urdf > robot.sdf
```

### ROS 2 Launch Integration

```xml
<!-- In your launch file -->
<launch>
  <!-- Convert and spawn URDF robot -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" exec="robot_state_publisher">
    <param name="robot_description" value="$(command 'xacro $(find-pkg-share my_robot_description)/urdf/my_robot.urdf.xacro')"/>
  </node>

  <!-- Spawn in Gazebo -->
  <node name="spawn_robot" pkg="gazebo_ros" exec="spawn_entity.py"
        args="-topic robot_description -entity my_robot -x 0 -y 0 -z 1.0"/>
</launch>
```

## Optimization Techniques for Simulation

### Simplified Collision Models

For better simulation performance, use simplified collision geometries:

```xml
<!-- Complex visual model -->
<visual>
  <geometry>
    <mesh filename="package://my_robot/meshes/complex_arm.dae"/>
  </geometry>
</visual>

<!-- Simplified collision model -->
<collision>
  <geometry>
    <cylinder>
      <radius>0.05</radius>
      <length>0.3</length>
    </cylinder>
  </geometry>
</collision>
```

### Hierarchical Model Organization

Organize complex humanoid models in a logical hierarchy:

```
base_footprint
├── base_link
└── torso
    ├── head
    ├── left_arm (chain: shoulder_pitch -> shoulder_roll -> elbow -> wrist)
    ├── right_arm (chain: shoulder_pitch -> shoulder_roll -> elbow -> wrist)
    ├── left_leg (chain: hip_yaw -> hip_pitch -> knee -> ankle)
    └── right_leg (chain: hip_yaw -> hip_pitch -> knee -> ankle)
```

## Best Practices for Humanoid Modeling

### 1. Realistic Mass Distribution
- Center of mass should be in the torso region
- Limbs should have appropriate mass ratios
- Use realistic values based on human proportions

### 2. Appropriate Joint Limits
- Respect human anatomical limitations
- Consider safety margins for control
- Account for mechanical constraints

### 3. Stable Simulation Parameters
- Proper inertia tensors for stable dynamics
- Appropriate friction coefficients for walking
- Balanced contact properties

### 4. Model Validation
- Check for kinematic loops
- Verify mass properties
- Test in simulation environment

## Troubleshooting Common Issues

### Robot Falls Through Ground
- Check that all links have proper collision geometry
- Verify mass properties are set (not zero)
- Ensure inertial properties are properly defined

### Unstable Joint Behavior
- Verify joint limits and dynamics parameters
- Check for proper parent-child relationships
- Adjust solver parameters in Gazebo

### Performance Issues
- Simplify collision geometry
- Reduce model complexity where possible
- Use appropriate physics parameters

## Summary

This chapter covered the essential aspects of robot description formats for humanoid simulation:

- URDF vs SDF differences and use cases
- Creating complex humanoid models with proper kinematic structure
- Simulation-specific extensions in SDF
- Optimization techniques for better performance
- Best practices for stable and realistic simulation

Understanding both URDF and SDF formats is crucial for developing humanoid robots that can be effectively simulated in Gazebo and controlled through ROS 2. The combination of accurate kinematic modeling with proper simulation parameters ensures realistic behavior in virtual environments.