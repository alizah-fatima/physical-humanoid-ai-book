---
sidebar_position: 3
---

# Chapter 3: URDF for Humanoid Robot Description and Modeling

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the fundamentals of URDF (Unified Robot Description Format)
- Create URDF models for humanoid robots
- Define links, joints, and their properties
- Incorporate visual, collision, and inertial elements
- Apply best practices for humanoid robot modeling

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its links (rigid parts), joints (connections between links), and other properties like inertial characteristics, visual appearance, and collision properties.

URDF is essential for:
- Robot simulation in Gazebo
- Robot visualization in RViz
- Kinematic analysis
- Motion planning
- Control algorithms

## URDF Structure and Components

### Basic URDF Structure

A basic URDF file has the following structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links -->
  <link name="base_link">
    <!-- Link properties -->
  </link>

  <!-- Joints -->
  <joint name="joint_name" type="joint_type">
    <parent link="parent_link_name"/>
    <child link="child_link_name"/>
    <!-- Joint properties -->
  </joint>
</robot>
```

### Links

A link represents a rigid part of the robot. Each link can have multiple properties:

```xml
<link name="link_name">
  <!-- Inertial properties -->
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="1.0"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>

  <!-- Visual properties -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>

  <!-- Collision properties -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </collision>
</link>
```

### Joints

Joints connect links and define how they can move relative to each other:

```xml
<joint name="joint_name" type="joint_type">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>
</joint>
```

## Joint Types

### 1. Fixed Joint

A fixed joint has no degrees of freedom:

```xml
<joint name="fixed_joint" type="fixed">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
</joint>
```

### 2. Revolute Joint

A revolute joint rotates around a single axis:

```xml
<joint name="revolute_joint" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
</joint>
```

### 3. Continuous Joint

Similar to revolute but without joint limits:

```xml
<joint name="continuous_joint" type="continuous">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```

### 4. Prismatic Joint

A prismatic joint moves linearly along an axis:

```xml
<joint name="prismatic_joint" type="prismatic">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="0.5" effort="10" velocity="1"/>
</joint>
```

## Visual and Collision Elements

### Visual Elements

Visual elements define how the robot appears in visualization tools:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Box -->
    <box size="0.1 0.1 0.1"/>
    <!-- Sphere -->
    <!-- <sphere radius="0.05"/> -->
    <!-- Cylinder -->
    <!-- <cylinder radius="0.05" length="0.1"/> -->
    <!-- Mesh -->
    <!-- <mesh filename="package://my_robot/meshes/link.dae"/> -->
  </geometry>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
</visual>
```

### Collision Elements

Collision elements define the collision properties for physics simulation:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.1 0.1"/>
  </geometry>
</collision>
```

## Inertial Properties

Inertial properties are crucial for physics simulation:

```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="1.0"/>
  <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
</inertial>
```

The inertia values depend on the shape and mass distribution. For common shapes:

- **Box**: `ixx = m/12 * (h² + d²)`, `iyy = m/12 * (w² + d²)`, `izz = m/12 * (w² + h²)`
- **Cylinder**: `ixx = iyy = m/12 * (3*r² + h²)`, `izz = m/2 * r²`
- **Sphere**: `ixx = iyy = izz = 2/5 * m * r²`

## Complete URDF Example - Simple Robot

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Upper body -->
  <link name="upper_body">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>

  <!-- Joint connecting base to upper body -->
  <joint name="base_upper_joint" type="fixed">
    <parent link="base_link"/>
    <child link="upper_body"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
  </joint>
</robot>
```

## Humanoid Robot URDF Modeling

### Humanoid Robot Structure

A humanoid robot typically consists of:
- Torso (trunk)
- Head
- Two arms (with shoulders, elbows, wrists)
- Two legs (with hips, knees, ankles)
- Hands and feet

### Humanoid Torso Example

```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
    <material name="light_gray">
      <color rgba="0.7 0.7 0.7 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10.0"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.6" iyz="0" izz="0.2"/>
  </inertial>
</link>
```

### Humanoid Head Example

```xml
<link name="head">
  <visual>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
    <material name="skin">
      <color rgba="0.8 0.6 0.4 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="2.0"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
</link>

<joint name="neck_joint" type="revolute">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.35" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
</joint>
```

### Humanoid Arm Example

```xml
<!-- Left shoulder -->
<link name="left_upper_arm">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
    <material name="light_gray">
      <color rgba="0.7 0.7 0.7 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.5"/>
    <origin xyz="0 0 0.15"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
  </inertial>
</link>

<joint name="left_shoulder_joint" type="revolute">
  <parent link="torso"/>
  <child link="left_upper_arm"/>
  <origin xyz="0.2 0 0.2" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="15" velocity="1"/>
</joint>

<!-- Left elbow -->
<link name="left_lower_arm">
  <visual>
    <geometry>
      <cylinder radius="0.04" length="0.25"/>
    </geometry>
    <material name="light_gray">
      <color rgba="0.7 0.7 0.7 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.04" length="0.25"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0.125"/>
    <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.0005"/>
  </inertial>
</link>

<joint name="left_elbow_joint" type="revolute">
  <parent link="left_upper_arm"/>
  <child link="left_lower_arm"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="2.35" effort="10" velocity="1"/>
</joint>
```

## Advanced URDF Features

### Transmission Elements

Transmission elements define how actuators connect to joints:

```xml
<transmission name="left_shoulder_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_shoulder_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_shoulder_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Gazebo-Specific Elements

Gazebo-specific elements can be included in URDF:

```xml
<gazebo reference="left_upper_arm">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>
```

### Using Xacro for Complex Models

Xacro (XML Macros) helps create complex URDF models:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">
  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="arm_length" value="0.3" />
  <xacro:property name="arm_radius" value="0.05" />

  <!-- Macro for arm -->
  <xacro:macro name="arm" params="side parent xyz rpy">
    <link name="${side}_upper_arm">
      <visual>
        <geometry>
          <cylinder radius="${arm_radius}" length="${arm_length}"/>
        </geometry>
        <material name="light_gray">
          <color rgba="0.7 0.7 0.7 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${arm_radius}" length="${arm_length}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="1.5"/>
        <origin xyz="0 0 ${arm_length/2}"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
      </inertial>
    </link>

    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="${parent}"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="15" velocity="1"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:arm side="left" parent="torso" xyz="0.2 0 0.2" rpy="0 0 0"/>
  <xacro:arm side="right" parent="torso" xyz="-0.2 0 0.2" rpy="0 0 0"/>
</robot>
```

## Humanoid Robot URDF Best Practices

### 1. Proper Mass Distribution

Ensure realistic mass distribution for stable simulation:

```xml
<!-- Example: Proper mass for humanoid links -->
<link name="torso">
  <inertial>
    <mass value="15.0"/> <!-- Realistic for human torso -->
    <origin xyz="0 0 0"/>
    <inertia ixx="0.8" ixy="0" ixz="0" iyy="0.6" iyz="0" izz="0.3"/>
  </inertial>
</link>
```

### 2. Realistic Joint Limits

Set appropriate joint limits based on human anatomy:

```xml
<!-- Shoulder joint with realistic limits -->
<joint name="left_shoulder_joint" type="revolute">
  <parent link="torso"/>
  <child link="left_upper_arm"/>
  <origin xyz="0.15 0.1 0.2" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="1.5" effort="50" velocity="2"/>
</joint>
```

### 3. Consistent Units

Always use consistent units (SI units: meters, kilograms, radians):

```xml
<!-- Good: Consistent units -->
<geometry>
  <box size="0.1 0.1 0.1"/> <!-- meters -->
</geometry>
<mass value="1.0"/> <!-- kilograms -->
<limit lower="-1.57" upper="1.57"/> <!-- radians -->
```

### 4. Hierarchical Structure

Organize links in a logical hierarchical structure:

```
base_footprint
   base_link (pelvis)
       torso
          head
          left_upper_arm
             left_lower_arm
                 left_hand
          right_upper_arm
              right_lower_arm
                  right_hand
       left_upper_leg
          left_lower_leg
              left_foot
       right_upper_leg
           right_lower_leg
               right_foot
```

## URDF Validation and Tools

### URDF Validation

Validate your URDF files using ROS tools:

```bash
# Check for errors in URDF
check_urdf my_robot.urdf

# Parse and display robot information
urdf_to_graphiz my_robot.urdf
```

### Visualization Tools

- **RViz**: For visualizing robot models
- **Gazebo**: For physics simulation
- **Robot Model Plugin**: For detailed model inspection

## Common URDF Issues and Solutions

### 1. Floating Point Issues

Use appropriate precision:

```xml
<!-- Instead of -->
<origin xyz="0.12345678901234567" rpy="0.98765432109876543"/>

<!-- Use -->
<origin xyz="0.123" rpy="0.987"/>
```

### 2. Self-Collision Issues

Define appropriate collision properties:

```xml
<!-- Prevent self-collision between adjacent links if needed -->
<collision>
  <geometry>
    <cylinder radius="0.04" length="0.2"/>
  </geometry>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</collision>
```

### 3. Inertial Issues

Calculate realistic inertial properties:

```xml
<!-- Good approximation for complex shapes -->
<inertial>
  <mass value="2.0"/>
  <origin xyz="0 0 0"/>
  <!-- Use simplified geometric approximation -->
  <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.05"/>
</inertial>
```

## Summary

This chapter covered the fundamentals of URDF modeling for humanoid robots, including:
- Basic URDF structure and components (links, joints, visual, collision, inertial)
- Different joint types and their applications
- Visual and collision properties for realistic modeling
- Inertial properties for physics simulation
- Advanced features like transmissions and Xacro
- Best practices for humanoid robot modeling

URDF is crucial for creating accurate robot models that can be used in simulation, visualization, and control. Proper modeling ensures realistic behavior in simulated environments and accurate kinematic analysis for real-world applications.