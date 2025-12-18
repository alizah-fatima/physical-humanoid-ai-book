---
description: "API and interface contracts for Module 1: The Robotic Nervous System (ROS 2) - Technical interface specifications"
---

# Module 1: The Robotic Nervous System (ROS 2) - Contracts

## Overview

This document defines the API contracts, interface specifications, and communication protocols used in the ROS 2 module covering architecture fundamentals, Python package development, and URDF modeling for humanoid robotics.

## ROS 2 Architecture Contracts

### Node Interface Contracts

#### Node Lifecycle Interface
```
Interface: rclpy.lifecycle.LifecycleNode
Methods:
- on_configure(State) -> TransitionCallbackReturn
- on_activate(State) -> TransitionCallbackReturn
- on_deactivate(State) -> TransitionCallbackReturn
- on_cleanup(State) -> TransitionCallbackReturn
- on_shutdown(State) -> TransitionCallbackReturn
- on_error(State) -> TransitionCallbackReturn

Contract Guarantees:
- State transitions follow strict lifecycle sequence
- Resources properly allocated in configure, released in cleanup
- Node can be safely activated/deactivated multiple times
- Error handling prevents system crashes
- Thread-safe state access
```

#### Node Communication Interface
```
Interface: rclpy.node.Node
Methods:
- create_publisher(msg_type, topic_name, qos_profile) -> Publisher
- create_subscription(msg_type, topic_name, callback, qos_profile) -> Subscription
- create_client(srv_type, srv_name) -> Client
- create_service(srv_type, srv_name, callback) -> Service
- create_timer(period, callback) -> Timer

Contract Guarantees:
- Publishers and subscribers properly typed
- QoS profiles respected across network
- Callbacks executed in proper context
- Resource cleanup on node destruction
- Thread-safe access to node resources
```

### Topic Communication Contracts

#### Standard Topic Types
```
Topic: /tf (tf2_msgs/TFMessage)
Frequency: Variable (typically 10-100 Hz)
Message Structure:
- transforms: geometry_msgs/TransformStamped[]
  - header: std_msgs/Header
    - stamp: builtin_interfaces/Time
    - frame_id: string
  - child_frame_id: string
  - transform: geometry_msgs/Transform
    - translation: geometry_msgs/Vector3
    - rotation: geometry_msgs/Quaternion

Contract Guarantees:
- Transform trees maintained consistently
- Timestamps monotonic and synchronized
- Frame IDs follow ROS naming conventions
- No circular dependencies in transforms
- Proper quaternion normalization
```

```
Topic: /joint_states (sensor_msgs/JointState)
Frequency: 50-200 Hz (control rate dependent)
Message Structure:
- header: std_msgs/Header
- name: string[] (joint names)
- position: float64[] (joint positions in radians)
- velocity: float64[] (joint velocities in rad/s)
- effort: float64[] (joint efforts in Nm)

Contract Guarantees:
- Arrays synchronized by index
- Position values within joint limits
- Velocity and effort optional but consistent
- Joint names match URDF/SDF definitions
- Timestamps reflect actual measurement time
```

```
Topic: /cmd_vel (geometry_msgs/Twist)
Frequency: 10-50 Hz (control rate)
Message Structure:
- linear: geometry_msgs/Vector3
  - x: float64 (forward/backward velocity)
  - y: float64 (lateral velocity)
  - z: float64 (vertical velocity)
- angular: geometry_msgs/Vector3
  - x: float64 (roll rate)
  - y: float64 (pitch rate)
  - z: float64 (yaw rate)

Contract Guarantees:
- Units in meters and radians
- Values within robot capability limits
- Coordinate frame clearly defined
- Smooth velocity transitions
- Zero velocity stops robot safely
```

### Service Contracts

#### Parameter Services
```
Service: /set_parameters (rcl_interfaces/SetParameters)
Request:
- parameters: rcl_interfaces/Parameter[] (parameters to set)

Response:
- results: rcl_interfaces/SetParametersResult[]
  - successful: bool
  - reason: string
  - type: uint8

Contract Guarantees:
- Parameter validation before setting
- Atomic parameter updates
- Proper error reporting
- Type safety enforced
- Callbacks executed on change
```

```
Service: /get_parameter_types (rcl_interfaces/GetParameterTypes)
Request:
- names: string[] (parameter names to query)

Response:
- types: uint8[] (parameter types in same order)

Contract Guarantees:
- Types match actual parameter values
- Names and types arrays aligned
- Fast lookup performance
- Consistent type enumeration
```

## ROS 2 Package Development Contracts

### Python Package Structure
```
Package Structure: {package_name}/
├── setup.py          # Package setup configuration
├── package.xml       # Package metadata and dependencies
├── CMakeLists.txt    # CMake build configuration
├── launch/           # Launch files for node execution
├── config/           # Configuration files
├── src/              # Python source files
│   └── {package_name}/
│       ├── __init__.py
│       └── *.py
├── test/             # Unit and integration tests
└── resource/         # Package resource files

Contract Guarantees:
- Package follows ROS 2 naming conventions
- Dependencies properly declared in package.xml
- Setup.py compatible with colcon build
- Code follows ROS 2 Python style guide
- Tests cover critical functionality
```

### AI Agent Bridge Interface
```
Interface: ros2_bridge.AIAgentBridge
Methods:
- send_message(str message) -> bool
- receive_message() -> str
- register_callback(callable callback) -> None
- disconnect() -> None

Message Structure:
- type: string (message type: "command", "data", "status")
- content: dict (message content)
- timestamp: float (ROS time)
- source: string (node name)
- destination: string (target node)

Contract Guarantees:
- Message format consistency
- Error handling for connection failures
- Thread-safe message passing
- Proper serialization/deserialization
- Timeout handling for blocking operations
```

### rclpy Client Library Contracts

#### Publisher Interface
```
Class: rclpy.publisher.Publisher
Methods:
- publish(message) -> None
- get_subscription_count() -> int
- assert_liveliness() -> None
- wait_for_all_acked(timeout_sec) -> bool

Contract Guarantees:
- Messages properly serialized
- QoS policies enforced
- Memory management handled automatically
- Thread-safe publishing
- Proper cleanup on destruction
```

#### Subscription Interface
```
Class: rclpy.subscription.Subscription
Methods:
- handle: Handle to internal subscription
- callback: Callable callback function
- message_type: Type of message

Contract Guarantees:
- Callbacks executed in node executor context
- Message type validation
- QoS policy enforcement
- Proper resource management
- Thread-safe access patterns
```

## URDF Modeling Contracts

### URDF Robot Description Format
```
Root Element: <robot>
Required Attributes:
- name: string (unique robot name)

Child Elements:
- <link> (one or more)
- <joint> (zero or more)
- <material> (zero or more)
- <gazebo> (simulation-specific, optional)

Contract Guarantees:
- Valid XML structure
- Unique names for all elements
- Complete kinematic chain
- Proper parent-child relationships
- Consistent units (meters, radians)
```

### Link Element Contract
```
Element: <link>
Required Attributes:
- name: string (unique link name)

Child Elements:
- <inertial> (mass, origin, inertia)
- <visual> (geometry, material, origin)
- <collision> (geometry, origin)

Contract Guarantees:
- Mass properties physically valid
- Geometry properly defined
- Visual and collision properties consistent
- Origin transformations valid
- No duplicate link names
```

### Joint Element Contract
```
Element: <joint>
Required Attributes:
- name: string (unique joint name)
- type: string (revolute, continuous, prismatic, fixed, etc.)

Child Elements:
- <parent link="parent_name"/>
- <child link="child_name"/>
- <origin xyz="x y z" rpy="r p y"/>
- <axis xyz="x y z"/> (for revolute/continuous/prismatic)
- <limit lower="min" upper="max" effort="max_effort" velocity="max_vel"/> (for revolute/prismatic)

Contract Guarantees:
- Joint connects exactly two links
- Parent and child links exist
- Joint type matches limits (fixed joints have no limits)
- Axis properly normalized
- Joint limits within physical constraints
```

### Humanoid-Specific URDF Extensions
```
Humanoid Joint Types:
- <joint name="hip_yaw" type="revolute">
  - Range: -1.57 to 1.57 radians
  - Max effort: 200 Nm
  - Max velocity: 3.14 rad/s

- <joint name="knee_pitch" type="revolute">
  - Range: 0.0 to 2.35 radians (only forward bending)
  - Max effort: 250 Nm
  - Max velocity: 3.14 rad/s

- <joint name="ankle_pitch" type="revolute">
  - Range: -0.78 to 0.78 radians
  - Max effort: 150 Nm
  - Max velocity: 3.14 rad/s

Contract Guarantees:
- Joint limits prevent self-collision
- Torque limits within actuator capabilities
- Range of motion matches human-like constraints
- Proper naming conventions for humanoid joints
- Kinematic chain enables stable walking
```

## Quality of Service (QoS) Contracts

### Standard QoS Profiles
```
Profile: sensor_data
- Reliability: BEST_EFFORT or RELIABLE
- Durability: VOLATILE
- History: KEEP_LAST with depth 1
- Deadline: None
- Lifespan: None
- Liveliness: AUTOMATIC
- Matched: 1 (minimum participants)

Contract Guarantees:
- Latest sensor data delivered
- Low latency for real-time processing
- Appropriate for high-frequency data
- Resource efficient
```

```
Profile: configuration
- Reliability: RELIABLE
- Durability: TRANSIENT_LOCAL
- History: KEEP_ALL
- Deadline: None
- Lifespan: None
- Liveliness: AUTOMATIC
- Matched: 1 (minimum participants)

Contract Guarantees:
- All configuration messages preserved
- Guaranteed delivery to late-joining nodes
- Suitable for critical parameters
- Persistent across node restarts
```

```
Profile: commands
- Reliability: RELIABLE
- Durability: VOLATILE
- History: KEEP_LAST with depth 1
- Deadline: 100ms
- Lifespan: None
- Liveliness: AUTOMATIC
- Matched: 1 (minimum participants)

Contract Guarantees:
- Command delivery guaranteed
- Commands not older than deadline
- Latest command takes precedence
- Timely execution of robot commands
```

## Error Handling Contracts

### Standard Error Messages
```
Error Types:
- rclpy.exceptions.ROSException: Base ROS exception
- rclpy.exceptions.ParameterException: Parameter validation
- rclpy.exceptions.UnsupportedEventTypeException: Event type not supported
- rclpy.exceptions.InvalidHandleException: Invalid node handle
- rclpy.exceptions.DuplicateNodeNameException: Node name conflict

Contract Guarantees:
- All exceptions inherit from ROSException
- Proper error context included
- Human-readable error messages
- Appropriate exception types for scenarios
- Stack traces preserved for debugging
```

### Service Error Handling
```
Service Response Structure:
- success: bool (operation completed successfully)
- message: string (human-readable status)
- error_code: int (machine-readable code)
- details: dict (additional error information)

Contract Guarantees:
- Error responses follow same format as success
- Error codes standardized across services
- Sufficient detail for debugging
- No sensitive information in error messages
- Consistent error handling patterns
```

## Validation Criteria

### Interface Compliance Tests
- All topics publish at specified rates
- Message formats match ROS 2 standards
- Service calls respond within timeout
- Coordinate systems are properly transformed
- Data integrity is maintained across interfaces

### URDF Validation
- XML structure valid
- All links referenced by joints exist
- Kinematic chains are complete
- Units are consistent
- No self-collision in default pose

### Performance Validation
- Node startup time < 1 second
- Message latency < 10ms for local communication
- Parameter service response < 100ms
- URDF parsing time < 500ms
- Memory usage within expected bounds

These contracts ensure consistent, reliable, and predictable behavior across all components of the ROS 2 system for humanoid robotics development.