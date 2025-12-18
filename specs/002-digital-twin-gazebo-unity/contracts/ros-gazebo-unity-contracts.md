---
description: "API and interface contracts for Module 2: The Digital Twin (Gazebo & Unity) - Technical interface specifications"
---

# Module 2: The Digital Twin (Gazebo & Unity) - Contracts

## Overview

This document defines the API contracts, interface specifications, and communication protocols used in the Digital Twin module covering Gazebo simulation, URDF/SDF modeling, and Unity integration for humanoid robotics.

## ROS 2 Message Contracts

### Sensor Message Interfaces

#### LaserScan Interface
```
Topic: /laser_scan
Type: sensor_msgs/LaserScan
Frequency: 10-20 Hz (configurable)

Message Structure:
- header: std_msgs/Header
- angle_min: float32 (radians)
- angle_max: float32 (radians)
- angle_increment: float32 (radians)
- time_increment: float32 (seconds)
- scan_time: float32 (seconds)
- range_min: float32 (meters)
- range_max: float32 (meters)
- ranges: float32[] (meters)
- intensities: float32[] (optional)

Contract Guarantees:
- Range values between range_min and range_max
- Array sizes match: ranges.size == intensities.size (if provided)
- Valid angle range: angle_min < angle_max
- Non-negative range values (except for invalid readings)
```

#### Image Interface
```
Topic: /camera/rgb/image_raw
Type: sensor_msgs/Image
Frequency: 15-30 Hz (configurable)

Message Structure:
- header: std_msgs/Header
- height: uint32 (pixels)
- width: uint32 (pixels)
- encoding: string (e.g., "rgb8", "bgr8", "mono8")
- is_bigendian: uint8
- step: uint32 (full row length in bytes)
- data: uint8[] (image data)

Contract Guarantees:
- height and width > 0
- data.size == step * height
- Valid encoding format
- Consistent frame rate
```

#### IMU Interface
```
Topic: /imu/data
Type: sensor_msgs/Imu
Frequency: 100-200 Hz (recommended for balance)

Message Structure:
- header: std_msgs/Header
- orientation: geometry_msgs/Quaternion
- orientation_covariance: float64[9]
- angular_velocity: geometry_msgs/Vector3
- angular_velocity_covariance: float64[9]
- linear_acceleration: geometry_msgs/Vector3
- linear_acceleration_covariance: float64[9]

Contract Guarantees:
- Orientation quaternion is normalized
- Covariance matrices are symmetric
- Units in SI (m/s², rad/s, etc.)
```

### Robot Control Interfaces

#### Joint State Interface
```
Topic: /joint_states
Type: sensor_msgs/JointState
Frequency: 50-100 Hz

Message Structure:
- header: std_msgs/Header
- name: string[] (joint names)
- position: float64[] (radians or meters)
- velocity: float64[] (optional)
- effort: float64[] (optional)

Contract Guarantees:
- Array sizes match: name.size == position.size
- Position values within joint limits
- Consistent joint naming convention
```

#### Joint Command Interface
```
Topic: /position_commands (or /velocity_commands, /effort_commands)
Type: trajectory_msgs/JointTrajectory

Message Structure:
- header: std_msgs/Header
- joint_names: string[]
- points: JointTrajectoryPoint[]
  - positions: float64[]
  - velocities: float64[] (optional)
  - accelerations: float64[] (optional)
  - effort: float64[] (optional)
  - time_from_start: builtin_interfaces/Duration

Contract Guarantees:
- Joint names match robot model
- Position values within limits
- Time parameters valid and increasing
```

## Gazebo Plugin Interfaces

### ROS 2 Control Plugin Contract
```
Plugin Name: libgazebo_ros2_control.so
Parameters:
- robot_description: string (URDF content)
- parameters: string (path to control config file)

Services Provided:
- /controller_manager/switch_controller
- /controller_manager/list_controllers
- /controller_manager/load_controller

Contract Guarantees:
- Maintains real-time performance
- Handles controller switching gracefully
- Provides feedback on controller states
```

### Sensor Plugin Contracts

#### Ray Sensor Plugin
```
Plugin Name: libgazebo_ros_ray_sensor.so
Parameters:
- frame_name: string
- topic_name: string
- update_rate: double
- output_type: string ("sensor_msgs/LaserScan" or "sensor_msgs/PointCloud2")

Contract Guarantees:
- Publishes at specified update rate
- Maintains sensor accuracy parameters
- Handles simulation pause/resume
```

#### IMU Sensor Plugin
```
Plugin Name: libgazebo_ros_imu.so
Parameters:
- frame_name: string
- topic_name: string
- update_rate: double
- gaussian_noise: double

Contract Guarantees:
- Provides realistic noise characteristics
- Maintains orientation accuracy
- Updates at consistent rate
```

## Unity Integration Contracts

### TCP Communication Protocol

#### Message Format
```
[HEADER][PAYLOAD]
- HEADER: 8 bytes
  - Size: uint32 (payload size in bytes)
  - Type: uint32 (message type identifier)
- PAYLOAD: variable size
  - Serialized ROS message data
```

#### Connection Interface
```
Interface: IUnityRosBridge
Methods:
- Connect(string hostname, int port): bool
- Disconnect(): void
- IsConnected(): bool
- SendMessage<T>(string topic, T message): bool
- Subscribe<T>(string topic, Action<T> callback): bool
- Unsubscribe(string topic): void

Contract Guarantees:
- Thread-safe operations
- Reliable message delivery
- Proper connection state management
- Error handling for network issues
```

### Robot Model Interface
```
Interface: IRobotModel
Properties:
- JointPositions: Dictionary<string, float>
- JointVelocities: Dictionary<string, float>
- JointEfforts: Dictionary<string, float>
- BasePose: Pose (position + orientation)

Methods:
- SetJointPosition(string jointName, float position): void
- GetJointPosition(string jointName): float
- SetBasePose(Pose pose): void
- UpdateFromJointStates(JointState jointState): void

Contract Guarantees:
- Joint names match URDF/SDF definitions
- Position values respect joint limits
- Pose updates are synchronized
- Thread-safe access to model data
```

## Simulation Environment Contracts

### World State Interface
```
Interface: ISimulationWorld
Properties:
- SimulationTime: Time (simulation time)
- RealTimeFactor: float
- PhysicsEngine: string
- Gravity: Vector3

Methods:
- PauseSimulation(): void
- ResumeSimulation(): void
- ResetSimulation(): void
- SpawnModel(string modelFile, Pose initialPose): bool
- RemoveModel(string modelName): bool

Contract Guarantees:
- Time progression when not paused
- Consistent physics behavior
- Model persistence across states
- Proper resource cleanup
```

### Sensor Data Interface
```
Interface: ISensorDataProcessor
Methods:
- ProcessLaserScan(LaserScan scan): ProcessedEnvironmentData
- ProcessImage(Image image): ProcessedImageData
- ProcessImu(Imu imu): ProcessedImuData
- ProcessJointStates(JointState states): ProcessedJointData

Contract Guarantees:
- Real-time processing capability
- Data integrity maintenance
- Consistent coordinate system
- Error handling for invalid data
```

## Performance Contracts

### Timing Guarantees
```
Real-time Factor: 1.0 (nominal)
Update Rates:
- Physics: 1000 Hz (0.001s timestep)
- High-frequency sensors (IMU): 100-200 Hz
- Medium sensors (cameras): 15-30 Hz
- Low-frequency sensors (LiDAR): 10-20 Hz
- Control loops: 50-200 Hz

Contract Guarantees:
- No timing violations in real-time execution
- Consistent update intervals
- Proper synchronization between components
```

### Resource Usage Limits
```
CPU Usage: < 80% sustained
Memory Usage: < 8GB for humanoid simulation
GPU Memory: < 2GB for rendering
Network Bandwidth: < 100 Mbps for Unity bridge

Contract Guarantees:
- Performance within specified limits
- Graceful degradation if limits approached
- Proper resource cleanup on shutdown
```

## Error Handling Contracts

### Failure Modes and Recovery
```
Gazebo Failure:
- If physics engine fails: Safe shutdown with state preservation
- If sensor plugin fails: Continue with other sensors, log error
- If ROS communication fails: Attempt reconnection with backoff

Unity Connection Failure:
- If TCP connection drops: Attempt reconnection with exponential backoff
- If message deserialization fails: Skip message, continue processing
- If visualization lags: Reduce quality settings automatically

Contract Guarantees:
- No data loss during recovery
- Informative error messages
- Automatic recovery where possible
- Safe fallback states
```

## Validation Criteria

### Interface Compliance Tests
- All topics publish at specified rates
- Message formats match ROS 2 standards
- Service calls respond within timeout
- Coordinate systems are properly transformed
- Data integrity is maintained across interfaces

### Performance Validation
- Real-time factor maintained at target
- No dropped messages in simulation
- Smooth visualization in Unity
- Consistent control loop timing
- Proper resource utilization

These contracts ensure consistent, reliable, and predictable behavior across all components of the Digital Twin system for humanoid robotics simulation.