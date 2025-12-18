---
description: "Data models and structures for Module 2: The Digital Twin (Gazebo & Unity) - Technical data specifications"
---

# Module 2: The Digital Twin (Gazebo & Unity) - Data Models

## Overview

This document defines the data models, structures, and formats used in the Digital Twin module covering Gazebo simulation, URDF/SDF modeling, and Unity integration for humanoid robotics.

## Robot Model Data Structures

### URDF Data Model

The Unified Robot Description Format follows this XML structure:

```xml
<robot name="robot_name">
  <link name="link_name">
    <inertial>
      <origin xyz="x y z" rpy="roll pitch yaw"/>
      <mass value="mass_in_kg"/>
      <inertia ixx="ixx" ixy="ixy" ixz="ixz" iyy="iyy" iyz="iyz" izz="izz"/>
    </inertial>
    <visual>
      <origin xyz="x y z" rpy="r p y"/>
      <geometry>
        <!-- box, cylinder, sphere, or mesh -->
      </geometry>
      <material name="material_name">
        <color rgba="r g b a"/>
      </material>
    </visual>
    <collision>
      <origin xyz="x y z" rpy="r p y"/>
      <geometry>
        <!-- box, cylinder, sphere, or mesh -->
      </geometry>
    </collision>
  </link>

  <joint name="joint_name" type="joint_type">
    <parent link="parent_link_name"/>
    <child link="child_link_name"/>
    <origin xyz="x y z" rpy="r p y"/>
    <axis xyz="x y z"/>
    <limit lower="lower_limit" upper="upper_limit" effort="effort_limit" velocity="velocity_limit"/>
    <dynamics damping="damping" friction="friction"/>
  </joint>
</robot>
```

### SDF Data Model

The Simulation Description Format extends URDF with simulation-specific properties:

```xml
<sdf version="1.7">
  <world name="world_name">
    <physics type="ode|bullet|simbody">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>
    <model name="model_name">
      <pose>x y z roll pitch yaw</pose>
      <link name="link_name">
        <inertial>
          <mass>mass_value</mass>
          <inertia>
            <ixx>ixx</ixx>
            <ixy>ixy</ixy>
            <ixz>ixz</ixz>
            <iyy>iyy</iyy>
            <iyz>iyz</iyz>
            <izz>izz</izz>
          </inertia>
        </inertial>
        <visual name="visual_name">
          <geometry>
            <!-- geometry specification -->
          </geometry>
        </visual>
        <collision name="collision_name">
          <geometry>
            <!-- geometry specification -->
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

## Sensor Data Models

### LiDAR Data Model (sensor_msgs/LaserScan)

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id

float32 angle_min          # start angle of the scan [rad]
float32 angle_max          # end angle of the scan [rad]
float32 angle_increment    # angular distance between measurements [rad]

float32 time_increment     # time between measurements [seconds]
float32 scan_time          # time between scans [seconds]

float32 range_min          # minimum range value [m]
float32 range_max          # maximum range value [m]

float32[] ranges           # range data [m]
float32[] intensities      # intensity data
```

### Camera Data Model (sensor_msgs/Image)

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id

uint32 height              # image height in pixels
uint32 width               # image width in pixels
string encoding            # encoding format (e.g., "rgb8", "bgr8", etc.)
uint8 is_bigendian         # endianness flag
uint32 step                # full row length in bytes
uint8[] data               # actual image data
```

### Depth Image Model (sensor_msgs/Image with depth encoding)

```
# Same as regular image but with:
# encoding = "32FC1" (32-bit float, 1 channel) for depth
# data contains depth values in meters
```

### IMU Data Model (sensor_msgs/Imu)

```
std_msgs/Header header
geometry_msgs/Quaternion orientation
  float64 x, y, z, w

float64[9] orientation_covariance  # 3x3 covariance matrix

geometry_msgs/Vector3 angular_velocity
  float64 x, y, z

float64[9] angular_velocity_covariance  # 3x3 covariance matrix

geometry_msgs/Vector3 linear_acceleration
  float64 x, y, z

float64[9] linear_acceleration_covariance  # 3x3 covariance matrix
```

## Unity Integration Data Models

### ROS Message Data Structure for Unity

```csharp
// Example structure for sensor data in Unity
[System.Serializable]
public class JointState
{
    public string[] name;
    public float[] position;
    public float[] velocity;
    public float[] effort;
}

[System.Serializable]
public class LaserScanData
{
    public float[] ranges;
    public float angle_min;
    public float angle_max;
    public float angle_increment;
    public float time_increment;
    public float range_min;
    public float range_max;
}

[System.Serializable]
public class ImuData
{
    public float orientation_x, orientation_y, orientation_z, orientation_w;
    public float angular_velocity_x, angular_velocity_y, angular_velocity_z;
    public float linear_acceleration_x, linear_acceleration_y, linear_acceleration_z;
}
```

### Coordinate System Transformations

```
ROS Standard: X forward, Y left, Z up
Unity Standard: X right, Y up, Z forward
Gazebo Standard: X forward, Y left, Z up

Transformation Matrix:
Unity = ROS_Rotation * ROS_Position
Where ROS_Rotation converts coordinate systems
```

## Simulation Environment Data Model

### World Configuration

```
World {
  PhysicsEngine: "ODE" | "Bullet" | "Simbody"
  Gravity: Vector3(x, y, z)
  TimeStep: float (seconds)
  RealTimeFactor: float
  Models: [Model]
  Lights: [Light]
  Plugins: [Plugin]
}

Model {
  Name: string
  Pose: Pose(x, y, z, roll, pitch, yaw)
  Links: [Link]
  Joints: [Joint]
  Static: bool
}

Link {
  Name: string
  Inertial: InertialProperties
  Visual: [VisualElement]
  Collision: [CollisionElement]
  Sensors: [Sensor]
}

Joint {
  Name: string
  Type: "revolute" | "continuous" | "prismatic" | "fixed" | "floating" | "planar"
  ParentLink: string
  ChildLink: string
  Pose: Pose
  Axis: AxisProperties
  Limits: JointLimits
}
```

## Humanoid-Specific Data Models

### Humanoid Kinematic Chain

```
HumanoidRobot {
  BaseFootprint: Link
  BaseLink: Link
  Torso: Link {
    Head: Link
    LeftArm: ArmChain
    RightArm: ArmChain
    LeftLeg: LegChain
    RightLeg: LegChain
  }
}

ArmChain {
  ShoulderYaw: Joint
  ShoulderPitch: Joint
  ShoulderRoll: Joint
  Elbow: Joint
  Wrist: JointChain
}

LegChain {
  HipYaw: Joint
  HipPitch: Joint
  HipRoll: Joint
  Knee: Joint
  Ankle: JointChain
}
```

### Balance Control Data Model

```
BalanceController {
  CenterOfMass: Vector3
  ZeroMomentPoint: Vector3
  SupportPolygon: Polygon
  DesiredState: RobotState
  CurrentState: RobotState
  ControlOutput: JointCommands
}

RobotState {
  BasePosition: Vector3
  BaseOrientation: Quaternion
  JointPositions: float[]
  JointVelocities: float[]
  IMUData: ImuData
  ForceTorqueData: ForceTorqueData
}
```

## Sensor Fusion Data Model

```
SensorFusion {
  InputSensors: {
    LiDAR: LaserScanData[]
    Cameras: ImageData[]
    IMUs: ImuData[]
    Encoders: JointState
    ForceTorque: ForceTorqueData[]
  }
  ProcessedData: {
    EnvironmentMap: OccupancyGrid
    RobotPose: PoseWithCovariance
    ObjectDetections: ObjectDetection[]
    HumanPoses: HumanPose[]
  }
  FusionAlgorithm: "EKF" | "UKF" | "ParticleFilter" | "NeuralNetwork"
}
```

## Performance Data Models

### Simulation Performance Metrics

```
PerformanceMetrics {
  RealTimeFactor: float
  UpdateRate: float
  PhysicsUpdateRate: float
  RenderingRate: float
  MemoryUsage: int (bytes)
  CPUUsage: float (percentage)
  SimulationSteps: int
  ErrorMetrics: {
    PositionError: float
    OrientationError: float
    TimingError: float
  }
}
```

## Configuration Data Models

### Robot Configuration File

```yaml
robot:
  name: "humanoid_robot"
  model_file: "path/to/robot.urdf"

sensors:
  lidar:
    topic: "/laser_scan"
    update_rate: 10
    range_min: 0.1
    range_max: 30.0

  camera:
    topic: "/camera/rgb/image_raw"
    resolution: [640, 480]
    fov: 60  # degrees

  imu:
    topic: "/imu/data"
    update_rate: 100

simulation:
  physics_engine: "ode"
  time_step: 0.001
  real_time_factor: 1.0
  gravity: [0, 0, -9.81]

unity_visualization:
  enable: true
  update_rate: 30
  quality_settings: "high"
```

These data models provide the foundation for consistent implementation of the Digital Twin concepts covered in Module 2, ensuring proper integration between Gazebo simulation, ROS 2 communication, and Unity visualization.