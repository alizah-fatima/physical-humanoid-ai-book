---
sidebar_position: 3
---

# Chapter 3: Nav2 and Path Planning for Bipedal Humanoid Movement

## Learning Objectives

By the end of this chapter, you will be able to:
- Adapt Nav2 framework for bipedal humanoid navigation
- Implement path planning algorithms considering humanoid constraints
- Design footstep planning for stable bipedal movement
- Execute sim-to-real transfer techniques for humanoid navigation
- Optimize navigation for complex humanoid environments
- Validate and test humanoid navigation systems

## Introduction to Humanoid Navigation Challenges

Humanoid navigation presents unique challenges compared to wheeled or tracked robots due to the inherent complexity of bipedal locomotion. Unlike traditional mobile robots, humanoid robots must maintain balance while navigating, which introduces additional constraints and considerations.

### Key Differences from Traditional Navigation

1. **Balance Requirements**: Humanoid robots must maintain center of mass within support polygon
2. **Step Constraints**: Limited step size, height, and placement capabilities
3. **Dynamic Stability**: Movement requires continuous balance control
4. **Gait Patterns**: Different walking patterns affect navigation planning
5. **Terrain Adaptation**: Must handle various surface types and obstacles

### Nav2 Framework Overview

Navigation2 (Nav2) is the standard navigation framework for ROS 2, but it requires adaptation for humanoid robots. The standard Nav2 stack includes:

- **Global Planner**: Path planning from start to goal
- **Local Planner**: Local obstacle avoidance and trajectory following
- **Controller**: Low-level control commands
- **Costmap**: Environment representation with obstacles
- **Recovery Behaviors**: Actions when navigation fails

For humanoid robots, each component needs to be adapted to consider bipedal constraints.

## Nav2 Adaptation for Humanoid Robots

### Humanoid-Specific Configuration

```yaml
# config/humanoid_nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.5
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_to_pose_bt_xml: "humanoid_nav_to_pose.xml"

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0  # Lower frequency for humanoid stability
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["HumanoidController"]

    # Humanoid-specific controller
    HumanoidController:
      plugin: "humanoid_mppi_controller::HumanoidMPPIController"
      time_steps: 50
      control_horizon: 1.0
      model_dt: 0.05
      # Humanoid-specific parameters
      vx_std: 0.2
      vy_std: 0.1
      wz_std: 0.3
      iteration_count: 100
      temperature: 0.3
      track_target_heading: True
      transform_tolerance: 0.3
      # Larger tolerances for humanoid stability
      xy_goal_tolerance: 0.3
      yaw_goal_tolerance: 0.3
      state_reset: True
      publish_cost_grid_pc: False
      progress_checker: "progress_checker"
      goal_checker: "goal_checker"
      # Humanoid-specific parameters
      max_step_length: 0.3      # Maximum step length in meters
      max_step_height: 0.1      # Maximum step height
      step_duration: 0.8        # Time per step in seconds
      balance_margin: 0.1       # Safety margin for balance

local_costmap:
  ros__parameters:
    use_sim_time: True
    update_frequency: 5.0
    publish_frequency: 2.0
    global_frame: odom
    robot_base_frame: base_link
    rolling_window: true
    width: 8.0    # Larger window for humanoid planning
    height: 8.0
    resolution: 0.05  # Higher resolution for step planning
    robot_radius: 0.4 # Larger radius for humanoid safety
    plugins: ["voxel_layer", "inflation_layer"]
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 3.0
      inflation_radius: 0.8  # Larger inflation for humanoid safety
    voxel_layer:
      plugin: "nav2_costmap_2d::VoxelLayer"
      enabled: True
      publish_voxel_map: False
      origin_z: 0.0
      z_resolution: 0.2
      z_voxels: 10
      max_obstacle_height: 2.0
      mark_threshold: 0
      observation_sources: pointcloud laser_scan
      pointcloud:
        topic: /points
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "PointCloud2"
        queue_size: 10
        expected_update_rate: 0.0
        observation_persistence: 0.0
        max_obstacle_range: 5.0  # Humanoid-specific range
        min_obstacle_range: 0.1
      laser_scan:
        topic: /scan
        max_obstacle_range: 5.0  # Extended range for humanoid planning
        clearing: True
        marking: True
        data_type: "LaserScan"
        raytrace_range: 6.0
        obstacle_range: 5.0

global_costmap:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    update_frequency: 1.0
    publish_frequency: 1.0
    static_map: true
    rolling_window: false
    width: 40.0    # Much larger for humanoid navigation
    height: 40.0
    resolution: 0.05 # Higher resolution for detailed planning
    robot_radius: 0.4
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      enabled: True
      observation_sources: pointcloud laser_scan
      pointcloud:
        topic: /points
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "PointCloud2"
        queue_size: 10
        expected_update_rate: 0.0
        observation_persistence: 0.0
        max_obstacle_range: 5.0
        min_obstacle_range: 0.1
      laser_scan:
        topic: /scan
        max_obstacle_range: 5.0
        clearing: True
        marking: True
        data_type: "LaserScan"
        raytrace_range: 6.0
        obstacle_range: 5.0
    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_subscribe_transient_local: True
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 3.0
      inflation_radius: 0.8
```

### Humanoid-Specific Behavior Tree

```xml
<!-- humanoid_nav_to_pose.xml -->
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <PipelineSequence name="NavigateWithReplanning">
            <RateController hz="1.0">
                <RecoveryNode number_of_retries="6" name="ComputeAndFollowPath">
                    <PipelineSequence name="ComputeAndFollowPath">
                        <RecoveryNode number_of_retries="2" name="ComputePath">
                            <RecoveryNode number_of_retries="2" name="SmoothPath">
                                <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
                                <SmoothPath input_path="{path}" output_path="{smoothed_path}" smoother_id="SimpleSmoother"/>
                            </RecoveryNode>
                            <FollowPath path="{smoothed_path}" controller_id="HumanoidController"/>
                        </RecoveryNode>
                    </PipelineSequence>
                    <RecoveryNode number_of_retries="2" name="HumanoidSpin">
                        <Spin spin_dist="1.57"/>
                    </RecoveryNode>
                </RecoveryNode>
            </RateController>
            <ReactiveSequence>
                <GoalUpdated/>
                <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
            </ReactiveSequence>
        </PipelineSequence>
    </BehaviorTree>
</root>
```

## Bipedal Path Planning Algorithms

### Footstep Planning Fundamentals

Footstep planning is critical for humanoid navigation. Unlike wheeled robots that can move continuously, humanoid robots must plan each foot placement to maintain stability.

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from scipy.spatial.transform import Rotation as R

class FootstepPlanner(Node):
    def __init__(self):
        super().__init__('footstep_planner')

        # Create publishers for visualization
        self.footstep_pub = self.create_publisher(MarkerArray, '/footsteps', 10)
        self.path_pub = self.create_publisher(Path, '/bipedal_path', 10)

        # Humanoid-specific parameters
        self.step_length = 0.3  # meters
        self.step_width = 0.2   # meters (distance between feet)
        self.max_step_height = 0.1
        self.step_duration = 0.8  # seconds per step
        self.support_polygon_margin = 0.05  # safety margin

    def plan_footsteps(self, global_path, start_pose):
        """
        Plan footstep sequence from global path
        """
        footsteps = []

        # Start with current position
        current_left_foot = self.calculate_initial_foot_position(start_pose, 'left')
        current_right_foot = self.calculate_initial_foot_position(start_pose, 'right')

        # Convert global path to footstep sequence
        for i in range(len(global_path.poses) - 1):
            # Calculate next step position based on path direction
            current_pose = global_path.poses[i]
            next_pose = global_path.poses[i + 1]

            # Calculate step direction
            dx = next_pose.pose.position.x - current_pose.pose.position.x
            dy = next_pose.pose.position.y - current_pose.pose.position.y
            step_direction = np.arctan2(dy, dx)

            # Generate footsteps based on gait pattern
            left_step, right_step = self.generate_step_pair(
                current_left_foot, current_right_foot, step_direction
            )

            footsteps.append(left_step)
            footsteps.append(right_step)

            # Update current foot positions
            current_left_foot = left_step
            current_right_foot = right_step

        return footsteps

    def calculate_initial_foot_position(self, robot_pose, foot_type):
        """
        Calculate initial foot position based on robot pose
        """
        # For initial position, place feet in a stable stance
        offset_x = 0.0
        if foot_type == 'left':
            offset_y = self.step_width / 2.0
        else:  # right
            offset_y = -self.step_width / 2.0

        # Apply offset in robot's coordinate frame
        rotation = R.from_quat([
            robot_pose.pose.orientation.x,
            robot_pose.pose.orientation.y,
            robot_pose.pose.orientation.z,
            robot_pose.pose.orientation.w
        ])

        offset_local = np.array([offset_x, offset_y, 0.0])
        offset_world = rotation.apply(offset_local)

        foot_pose = PoseStamped()
        foot_pose.header = robot_pose.header
        foot_pose.pose.position.x = robot_pose.pose.position.x + offset_world[0]
        foot_pose.pose.position.y = robot_pose.pose.position.y + offset_world[1]
        foot_pose.pose.position.z = robot_pose.pose.position.z  # Ground level

        return foot_pose

    def generate_step_pair(self, left_foot, right_foot, direction):
        """
        Generate a pair of footsteps based on gait pattern
        """
        # Simple alternating gait: left, right, left, right...
        # Calculate next step position in the direction of movement
        step_distance = self.step_length

        # Calculate new positions
        dx = step_distance * np.cos(direction)
        dy = step_distance * np.sin(direction)

        # For alternating gait, each foot takes a step
        new_left = PoseStamped()
        new_left.header = left_foot.header
        new_left.pose.position.x = left_foot.pose.position.x + dx
        new_left.pose.position.y = left_foot.pose.position.y + dy
        new_left.pose.position.z = left_foot.pose.position.z

        new_right = PoseStamped()
        new_right.header = right_foot.header
        new_right.pose.position.x = right_foot.pose.position.x + dx
        new_right.pose.position.y = right_foot.pose.position.y + dy
        new_right.pose.position.z = right_foot.pose.position.z

        # Add some offset to maintain balance
        # In a real implementation, this would consider support polygon
        return new_left, new_right

    def validate_footstep(self, foot_pose, costmap):
        """
        Validate that a footstep is safe and stable
        """
        # Check if position is in collision-free area
        # Check if step maintains balance
        # Check if terrain is suitable for stepping
        return True  # Simplified for example

    def visualize_footsteps(self, footsteps):
        """
        Visualize footsteps in RViz
        """
        marker_array = MarkerArray()

        for i, footstep in enumerate(footsteps):
            marker = Marker()
            marker.header = footstep.header
            marker.ns = "footsteps"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose = footstep.pose
            marker.scale.x = 0.15  # Foot size
            marker.scale.y = 0.08
            marker.scale.z = 0.01

            if i % 2 == 0:
                marker.color.r = 1.0  # Left foot in red
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0  # Right foot in blue
                marker.color.g = 0.0
                marker.color.b = 1.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        self.footstep_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    footstep_planner = FootstepPlanner()

    try:
        rclpy.spin(footstep_planner)
    except KeyboardInterrupt:
        pass
    finally:
        footstep_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Center of Mass Trajectory Planning

```python
class CoMTrajectoryPlanner:
    def __init__(self):
        self.footstep_planner = FootstepPlanner()
        self.zmp_reference = []  # Zero Moment Point reference trajectory
        self.com_trajectory = []  # Center of Mass trajectory

    def generate_com_trajectory(self, footsteps):
        """
        Generate Center of Mass trajectory from footstep plan
        """
        # Use inverted pendulum model for CoM planning
        # The CoM trajectory should keep the Zero Moment Point (ZMP)
        # within the support polygon defined by feet

        com_trajectory = []

        # For each footstep, calculate CoM position that maintains balance
        for i in range(len(footsteps)):
            # Calculate support polygon from current and next foot positions
            support_polygon = self.calculate_support_polygon(footsteps, i)

            # Plan CoM trajectory to stay within support polygon
            com_point = self.plan_com_point(support_polygon, i)
            com_trajectory.append(com_point)

        return com_trajectory

    def calculate_support_polygon(self, footsteps, step_index):
        """
        Calculate support polygon based on foot positions
        """
        # Support polygon is the convex hull of contact points
        # For biped, it's the area between feet
        if step_index < len(footsteps) - 1:
            # Double support phase (both feet on ground)
            left_foot = footsteps[step_index]
            right_foot = footsteps[step_index + 1]

            # Create polygon from foot positions
            polygon = [
                (left_foot.pose.position.x, left_foot.pose.position.y),
                (right_foot.pose.position.x, right_foot.pose.position.y)
            ]
        else:
            # Single support phase (one foot on ground)
            support_foot = footsteps[step_index]
            polygon = [(support_foot.pose.position.x, support_foot.pose.position.y)]

        return polygon

    def plan_com_point(self, support_polygon, step_index):
        """
        Plan CoM point that maintains stability
        """
        # Calculate CoM position that keeps ZMP within support polygon
        # This is a simplified version - real implementation would use
        # more sophisticated balance control algorithms

        # For now, keep CoM roughly centered over support polygon
        com_x = sum([p[0] for p in support_polygon]) / len(support_polygon)
        com_y = sum([p[1] for p in support_polygon]) / len(support_polygon)
        com_z = 0.8  # Typical CoM height for humanoid

        return (com_x, com_y, com_z)

    def generate_zmp_trajectory(self, com_trajectory, dt=0.01):
        """
        Generate ZMP trajectory from CoM trajectory
        """
        zmp_trajectory = []

        for i in range(len(com_trajectory)):
            com = com_trajectory[i]

            # Calculate ZMP based on inverted pendulum model
            # ZMP_x = CoM_x - (CoM_z - support_z) * CoM_ddot_x / gravity
            # ZMP_y = CoM_y - (CoM_z - support_z) * CoM_ddot_y / gravity

            # Simplified: ZMP close to CoM projection
            zmp_x = com[0]
            zmp_y = com[1]

            zmp_trajectory.append((zmp_x, zmp_y))

        return zmp_trajectory
```

### Advanced Path Planning for Humanoid Constraints

```python
class HumanoidPathPlanner:
    def __init__(self):
        self.step_constraints = {
            'max_step_length': 0.3,
            'max_step_width': 0.4,
            'max_step_height': 0.1,
            'min_step_length': 0.05,
            'foot_size': (0.15, 0.08),  # length, width
        }
        self.balance_constraints = {
            'max_com_offset': 0.1,
            'support_polygon_margin': 0.05,
        }

    def plan_path_with_constraints(self, start, goal, costmap):
        """
        Plan path considering humanoid-specific constraints
        """
        # Use A* or Dijkstra with humanoid-specific cost function
        # Consider step size limits, balance requirements, and terrain
        path = self.humanoid_astar(start, goal, costmap)
        return path

    def humanoid_astar(self, start, goal, costmap):
        """
        A* algorithm adapted for humanoid navigation
        """
        import heapq

        # Priority queue: (cost, position, g_score)
        open_set = [(0, start, 0)]
        closed_set = set()
        came_from = {}
        g_score = {start: 0}

        while open_set:
            current_cost, current, current_g = heapq.heappop(open_set)

            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]

            closed_set.add(current)

            # Get neighbors considering humanoid step constraints
            neighbors = self.get_humanoid_neighbors(current, costmap)

            for neighbor in neighbors:
                if neighbor in closed_set:
                    continue

                # Calculate tentative g_score
                tentative_g = current_g + self.calculate_step_cost(current, neighbor)

                if neighbor not in [item[1] for item in open_set] or tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)

                    heapq.heappush(open_set, (f_score, neighbor, tentative_g))

        return []  # No path found

    def get_humanoid_neighbors(self, position, costmap):
        """
        Get valid neighboring positions considering humanoid constraints
        """
        neighbors = []

        # Generate possible step positions within constraints
        for step_x in np.arange(-self.step_constraints['max_step_length'],
                                self.step_constraints['max_step_length'], 0.1):
            for step_y in np.arange(-self.step_constraints['max_step_width']/2,
                                    self.step_constraints['max_step_width']/2, 0.1):
                neighbor_x = position[0] + step_x
                neighbor_y = position[1] + step_y

                neighbor = (neighbor_x, neighbor_y)

                # Check if step is valid
                if self.is_valid_step(position, neighbor, costmap):
                    neighbors.append(neighbor)

        return neighbors

    def is_valid_step(self, from_pos, to_pos, costmap):
        """
        Check if a step is valid considering all constraints
        """
        # Check step size constraints
        step_dist = np.sqrt((to_pos[0] - from_pos[0])**2 + (to_pos[1] - from_pos[1])**2)
        if step_dist > self.step_constraints['max_step_length']:
            return False

        # Check collision with obstacles
        if self.is_collision(to_pos, costmap):
            return False

        # Check terrain suitability
        if not self.is_traversable_terrain(to_pos, costmap):
            return False

        return True

    def calculate_step_cost(self, from_pos, to_pos):
        """
        Calculate cost of taking a step from one position to another
        """
        # Base cost is distance
        distance = np.sqrt((to_pos[0] - from_pos[0])**2 + (to_pos[1] - from_pos[1])**2)

        # Add penalties for difficult terrain
        terrain_penalty = self.get_terrain_penalty(to_pos)

        return distance + terrain_penalty

    def heuristic(self, pos, goal):
        """
        Heuristic function for A* (Euclidean distance)
        """
        return np.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)

    def is_collision(self, position, costmap):
        """
        Check if position is in collision
        """
        # Check costmap for obstacle
        costmap_value = self.get_costmap_value(position, costmap)
        return costmap_value >= 50  # Threshold for obstacle

    def is_traversable_terrain(self, position, costmap):
        """
        Check if terrain is suitable for humanoid stepping
        """
        # Check if terrain is flat enough, not too steep, etc.
        return True  # Simplified

    def get_terrain_penalty(self, position):
        """
        Get penalty for traversing terrain at position
        """
        # Higher penalty for rough terrain, stairs, etc.
        return 0.0  # Simplified
```

## Sim-to-Real Transfer Techniques

### Domain Randomization for Navigation

Domain randomization helps improve sim-to-real transfer by training navigation systems on diverse environments:

```python
class DomainRandomizer:
    def __init__(self):
        self.randomization_params = {
            'lighting': {
                'intensity_range': (1000, 5000),
                'color_temperature_range': (3000, 8000)
            },
            'textures': {
                'roughness_range': (0.0, 1.0),
                'metallic_range': (0.0, 1.0),
                'normal_map_strength_range': (0.0, 1.0)
            },
            'materials': {
                'friction_range': (0.1, 1.0),
                'restitution_range': (0.0, 0.5)
            },
            'geometry': {
                'scale_variation': 0.1,
                'position_jitter': 0.05
            }
        }

    def randomize_scene(self, scene_data):
        """
        Apply domain randomization to scene
        """
        randomized_scene = scene_data.copy()

        # Randomize lighting
        randomized_scene = self.randomize_lighting(randomized_scene)

        # Randomize textures
        randomized_scene = self.randomize_textures(randomized_scene)

        # Randomize materials
        randomized_scene = self.randomize_materials(randomized_scene)

        # Randomize geometry
        randomized_scene = self.randomize_geometry(randomized_scene)

        return randomized_scene

    def randomize_lighting(self, scene):
        """
        Randomize lighting conditions
        """
        intensity = np.random.uniform(
            self.randomization_params['lighting']['intensity_range'][0],
            self.randomization_params['lighting']['intensity_range'][1]
        )

        # Apply to all lights in scene
        for light in scene['lights']:
            light['intensity'] = intensity

        return scene

    def randomize_textures(self, scene):
        """
        Randomize texture properties
        """
        for obj in scene['objects']:
            if 'texture' in obj:
                obj['texture']['roughness'] = np.random.uniform(
                    self.randomization_params['textures']['roughness_range'][0],
                    self.randomization_params['textures']['roughness_range'][1]
                )

        return scene

    def randomize_materials(self, scene):
        """
        Randomize material properties
        """
        for obj in scene['objects']:
            if 'material' in obj:
                obj['material']['friction'] = np.random.uniform(
                    self.randomization_params['materials']['friction_range'][0],
                    self.randomization_params['materials']['friction_range'][1]
                )

        return scene

    def randomize_geometry(self, scene):
        """
        Randomize geometric properties
        """
        for obj in scene['objects']:
            # Add small random variations to position and scale
            pos_jitter = np.random.uniform(
                -self.randomization_params['geometry']['position_jitter'],
                self.randomization_params['geometry']['position_jitter'],
                3
            )
            obj['position'] = np.array(obj['position']) + pos_jitter

        return scene
```

### System Identification for Real Robot Tuning

System identification helps adapt simulation parameters to match real robot behavior:

```python
class SystemIdentifier:
    def __init__(self):
        self.model_type = 'bipedal_dynamics'
        self.parameters = {
            'mass': 50.0,  # kg
            'com_height': 0.8,  # m
            'step_length_max': 0.3,  # m
            'step_duration': 0.8,  # s
            'balance_stiffness': 100.0,
            'control_gain': 1.0
        }

    def collect_system_data(self, robot, trajectory):
        """
        Collect input-output data for system identification
        """
        inputs = []
        outputs = []

        # Execute trajectory and collect data
        for cmd in trajectory:
            # Send command to robot
            robot.send_command(cmd)

            # Wait and measure response
            robot.wait_for_response()

            # Collect input (command) and output (actual pose, sensor data)
            input_data = self.get_command_data(cmd)
            output_data = self.get_sensor_data(robot)

            inputs.append(input_data)
            outputs.append(output_data)

        return inputs, outputs

    def estimate_parameters(self, inputs, outputs):
        """
        Estimate system parameters from input-output data
        """
        # Use least squares or other system identification method
        # This is a simplified example
        A = np.array(inputs)
        B = np.array(outputs)

        # Solve: params = (A^T * A)^(-1) * A^T * B
        try:
            estimated_params = np.linalg.lstsq(A, B, rcond=None)[0]
            return estimated_params
        except np.linalg.LinAlgError:
            # Handle singular matrix case
            return self.parameters  # Return default if estimation fails

    def update_simulation_model(self, estimated_params):
        """
        Update simulation model with estimated parameters
        """
        # Update physics parameters in simulation
        # This would involve modifying simulation configuration
        updated_params = self.parameters.copy()

        # Apply estimated parameters
        for i, param_name in enumerate(self.parameters.keys()):
            if i < len(estimated_params):
                updated_params[param_name] = estimated_params[i]

        return updated_params

    def validate_model(self, simulation_model, real_robot_data):
        """
        Validate that simulation model matches real robot behavior
        """
        # Compare simulation output with real robot output
        # for the same inputs
        simulation_output = self.run_simulation(simulation_model, real_robot_data['inputs'])
        real_output = real_robot_data['outputs']

        # Calculate error metrics
        error = np.mean(np.abs(simulation_output - real_output))

        return error < 0.1  # Return True if error is acceptable

    def run_simulation(self, model, inputs):
        """
        Run simulation with given model and inputs
        """
        # This would run the actual simulation
        # Return simulation outputs
        pass
```

### Transfer Learning Approaches

```python
class TransferLearner:
    def __init__(self):
        self.simulation_policy = None
        self.real_robot_policy = None
        self.transfer_method = 'domain_adaptation'

    def pretrain_in_simulation(self, env, policy_network):
        """
        Pre-train navigation policy in simulation
        """
        import torch
        import torch.nn as nn
        import torch.optim as optim

        # Train policy in simulation environment
        optimizer = optim.Adam(policy_network.parameters(), lr=0.001)
        criterion = nn.MSELoss()

        for episode in range(1000):  # Training episodes
            # Run episode in simulation
            state = env.reset()
            total_reward = 0

            for step in range(100):  # Steps per episode
                # Get action from policy
                action = policy_network(torch.tensor(state, dtype=torch.float32))

                # Execute action
                next_state, reward, done, _ = env.step(action.detach().numpy())

                # Update policy
                loss = criterion(action, torch.tensor([0.5, 0.5]))  # Simplified
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

                state = next_state
                total_reward += reward

                if done:
                    break

            if episode % 100 == 0:
                print(f"Episode {episode}, Average Reward: {total_reward}")

        return policy_network

    def adapt_to_real_robot(self, pretrained_policy, real_data):
        """
        Adapt simulation-trained policy to real robot
        """
        # Fine-tune policy with real robot data
        # This could involve domain adaptation techniques
        adapted_policy = pretrained_policy

        # Adjust for real robot dynamics
        # Update network weights based on real data
        return adapted_policy

    def validate_transfer(self, adapted_policy, real_env):
        """
        Validate that transferred policy works on real robot
        """
        success_rate = 0
        total_trials = 0

        for trial in range(10):  # Validation trials
            state = real_env.reset()
            success = False

            for step in range(200):  # Max steps per trial
                action = adapted_policy(state)
                next_state, reward, done, info = real_env.step(action)

                if info.get('reached_goal', False):
                    success = True
                    break
                elif done:
                    break

                state = next_state

            if success:
                success_rate += 1
            total_trials += 1

        return success_rate / total_trials if total_trials > 0 else 0
```

## Isaac Sim Integration for Humanoid Navigation

### Creating Navigation Training Environments

```python
# Isaac Sim script for creating navigation training environments
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import create_primitive
from omni.isaac.core.materials import OmniPBR
import numpy as np

class HumanoidNavigationTrainer:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)
        self.assets_root_path = get_assets_root_path()
        self.setup_environment()

    def setup_environment(self):
        """
        Set up navigation training environment in Isaac Sim
        """
        # Add humanoid robot
        humanoid_path = self.assets_root_path + "/Isaac/Robots/Humanoid/humanoid_instanceable.usd"
        add_reference_to_stage(usd_path=humanoid_path, prim_path="/World/Humanoid")

        # Create navigation course with obstacles
        self.create_navigation_course()

        # Add sensors for navigation
        self.add_navigation_sensors()

        # Enable physics
        self.world.reset()

    def create_navigation_course(self):
        """
        Create a navigation course with various obstacles
        """
        # Create ground plane
        create_primitive(
            prim_path="/World/ground",
            primitive_type="Plane",
            position=[0, 0, 0],
            scale=[20, 20, 1],
            orientation=[0.0, 0.0, 0.0, 1.0]
        )

        # Add obstacles
        obstacles = [
            {"type": "box", "pos": [2, 0, 0.5], "size": [0.5, 2, 1]},
            {"type": "box", "pos": [-2, 3, 0.5], "size": [2, 0.5, 1]},
            {"type": "cylinder", "pos": [0, -3, 1], "size": [0.5, 2]},
        ]

        for i, obs in enumerate(obstacles):
            if obs["type"] == "box":
                create_primitive(
                    prim_path=f"/World/obstacle_{i}",
                    primitive_type="Cube",
                    position=obs["pos"],
                    scale=obs["size"]
                )
            elif obs["type"] == "cylinder":
                create_primitive(
                    prim_path=f"/World/obstacle_{i}",
                    primitive_type="Cylinder",
                    position=obs["pos"],
                    scale=[obs["size"][0], obs["size"][0], obs["size"][1]]
                )

    def add_navigation_sensors(self):
        """
        Add sensors needed for navigation
        """
        from omni.isaac.sensor import Camera, LidarRtx
        from omni.isaac.core.sensors import ImuSensor

        # Add RGB-D camera for perception
        self.camera = Camera(
            prim_path="/World/Humanoid/Head/Camera",
            frequency=30,
            resolution=(640, 480)
        )

        # Add LiDAR for obstacle detection
        self.lidar = LidarRtx(
            prim_path="/World/Humanoid/Torso/Lidar",
            points_per_second=500000,
            rotation_frequency=20,
            channels=16,
            horizontal_resolution=2,
            vertical_resolution=2,
            upper_fov=15,
            lower_fov=-15,
            max_range=25.0,
            min_range=0.1
        )

        # Add IMU for balance feedback
        self.imu = ImuSensor(
            prim_path="/World/Humanoid/Torso/Imu",
            name="torso_imu",
            frequency=100
        )

    def run_navigation_training(self, num_episodes=1000):
        """
        Run navigation training episodes
        """
        for episode in range(num_episodes):
            # Reset environment
            self.world.reset()

            # Apply randomization for domain randomization
            self.randomize_environment()

            # Run navigation episode
            self.run_navigation_episode()

            if episode % 100 == 0:
                print(f"Completed episode {episode}")

    def randomize_environment(self):
        """
        Apply domain randomization to environment
        """
        # Randomize lighting
        from omni.isaac.core.utils.prims import get_prim_at_path
        from pxr import Gf

        dome_light = get_prim_at_path("/World/DomeLight")
        if dome_light:
            intensity = np.random.uniform(1000, 5000)
            dome_light.GetAttribute("inputs:intensity").Set(intensity)

    def run_navigation_episode(self):
        """
        Run a single navigation episode
        """
        # This would implement the actual navigation algorithm
        # and collect training data
        pass
```

## Validation and Testing Strategies

### Simulation Validation

```python
class NavigationValidator:
    def __init__(self, simulation_env, real_robot=None):
        self.sim_env = simulation_env
        self.real_robot = real_robot
        self.metrics = {}

    def validate_in_simulation(self):
        """
        Validate navigation performance in simulation
        """
        metrics = {
            'success_rate': 0,
            'path_efficiency': 0,
            'collision_rate': 0,
            'com_stability': 0,
            'computation_time': 0
        }

        total_trials = 100
        successful_trials = 0
        total_path_length = 0
        total_optimal_length = 0
        collision_count = 0
        stability_score = 0

        for trial in range(total_trials):
            # Run navigation trial
            result = self.run_navigation_trial()

            if result['success']:
                successful_trials += 1
                total_path_length += result['path_length']
                total_optimal_length += result['optimal_length']

            if result['collision']:
                collision_count += 1

            stability_score += result['stability_score']

        metrics['success_rate'] = successful_trials / total_trials
        metrics['path_efficiency'] = total_optimal_length / total_path_length if total_path_length > 0 else 0
        metrics['collision_rate'] = collision_count / total_trials
        metrics['com_stability'] = stability_score / total_trials

        self.metrics['simulation'] = metrics
        return metrics

    def run_navigation_trial(self):
        """
        Run a single navigation trial
        """
        # Reset environment
        self.sim_env.reset()

        # Set random start and goal positions
        start_pos = self.generate_random_position()
        goal_pos = self.generate_random_goal_position(start_pos)

        # Run navigation
        path, success, collision = self.execute_navigation(start_pos, goal_pos)

        # Calculate metrics
        optimal_length = self.calculate_optimal_distance(start_pos, goal_pos)
        path_length = self.calculate_path_length(path)
        stability_score = self.calculate_balance_stability(path)

        return {
            'success': success,
            'collision': collision,
            'path_length': path_length,
            'optimal_length': optimal_length,
            'stability_score': stability_score
        }

    def validate_sim_to_real_transfer(self):
        """
        Validate sim-to-real transfer performance
        """
        if not self.real_robot:
            print("No real robot available for validation")
            return None

        # Compare simulation and real robot performance
        sim_metrics = self.metrics.get('simulation', {})
        real_metrics = self.validate_on_real_robot()

        transfer_gap = {}
        for metric in sim_metrics:
            if metric in real_metrics:
                gap = abs(sim_metrics[metric] - real_metrics[metric])
                transfer_gap[metric] = gap

        self.metrics['transfer_gap'] = transfer_gap
        return transfer_gap

    def validate_on_real_robot(self):
        """
        Validate navigation on real robot
        """
        # This would run actual tests on the real robot
        # Collecting the same metrics as in simulation
        pass

    def generate_random_position(self):
        """
        Generate random starting position
        """
        x = np.random.uniform(-10, 10)
        y = np.random.uniform(-10, 10)
        return (x, y)

    def generate_random_goal_position(self, start_pos):
        """
        Generate random goal position different from start
        """
        while True:
            goal_pos = self.generate_random_position()
            distance = np.sqrt((goal_pos[0] - start_pos[0])**2 + (goal_pos[1] - start_pos[1])**2)
            if distance > 2.0:  # Minimum distance from start
                return goal_pos
```

## Deployment and Real-World Testing

### Real Robot Integration

```python
class RealRobotDeployer:
    def __init__(self, robot_interface):
        self.robot = robot_interface
        self.nav_system = None
        self.safety_monitor = SafetyMonitor()

    def deploy_navigation_system(self, config_file):
        """
        Deploy navigation system to real robot
        """
        # Load configuration
        config = self.load_config(config_file)

        # Initialize navigation stack
        self.nav_system = self.initialize_navigation_stack(config)

        # Set up safety systems
        self.safety_monitor.start_monitoring()

        print("Navigation system deployed successfully")

    def load_config(self, config_file):
        """
        Load navigation configuration
        """
        import yaml
        with open(config_file, 'r') as f:
            return yaml.safe_load(f)

    def initialize_navigation_stack(self, config):
        """
        Initialize navigation stack with proper parameters
        """
        # Initialize Nav2 with humanoid-specific parameters
        # Set up proper TF frames
        # Configure sensors
        pass

    def run_safety_checks(self):
        """
        Run pre-navigation safety checks
        """
        checks = [
            self.check_battery_level(),
            self.check_sensor_health(),
            self.verify_calibration(),
            self.validate_environment(),
        ]

        return all(checks)

    def check_battery_level(self):
        """
        Check robot battery level
        """
        battery_level = self.robot.get_battery_level()
        return battery_level > 0.2  # At least 20% battery

    def check_sensor_health(self):
        """
        Check if all navigation sensors are healthy
        """
        sensors = ['lidar', 'camera', 'imu', 'joint_encoders']
        for sensor in sensors:
            if not self.robot.is_sensor_healthy(sensor):
                return False
        return True

    def verify_calibration(self):
        """
        Verify sensor and robot calibration
        """
        # Check if IMU is calibrated
        # Check if cameras are calibrated
        # Check if joint encoders are accurate
        pass

    def navigate_with_monitoring(self, goal):
        """
        Navigate to goal with continuous safety monitoring
        """
        if not self.run_safety_checks():
            print("Safety checks failed, aborting navigation")
            return False

        try:
            # Start navigation
            self.nav_system.navigate_to_goal(goal)

            # Monitor during navigation
            while not self.nav_system.is_goal_reached():
                if self.safety_monitor.is_unsafe():
                    self.emergency_stop()
                    return False

                # Continue monitoring
                self.safety_monitor.update()

            return True

        except Exception as e:
            print(f"Navigation error: {e}")
            self.emergency_stop()
            return False

    def emergency_stop(self):
        """
        Emergency stop for safety
        """
        self.robot.stop_motion()
        self.nav_system.cancel_all_goals()
        print("Emergency stop executed")

class SafetyMonitor:
    def __init__(self):
        self.unsafe_conditions = []
        self.monitoring_active = False

    def start_monitoring(self):
        """
        Start safety monitoring
        """
        self.monitoring_active = True
        # Start monitoring threads/processes
        pass

    def is_unsafe(self):
        """
        Check if current state is unsafe
        """
        # Check for various unsafe conditions
        conditions = [
            self.is_balance_unstable(),
            self.is_collision_imminent(),
            self.is_motor_overheating(),
            self.is_battery_critical(),
        ]

        return any(conditions)

    def is_balance_unstable(self):
        """
        Check if robot balance is unstable
        """
        # Check IMU data for balance issues
        # Check CoM position relative to support polygon
        pass

    def is_collision_imminent(self):
        """
        Check if collision is imminent
        """
        # Check proximity sensors
        # Check planned path for obstacles
        pass

    def update(self):
        """
        Update safety monitoring
        """
        if self.monitoring_active:
            # Update all safety checks
            pass
```

## Performance Optimization

### Real-time Performance Considerations

```python
class PerformanceOptimizer:
    def __init__(self):
        self.profiling_enabled = True
        self.performance_targets = {
            'path_planning_rate': 1.0,      # Hz
            'control_rate': 100.0,          # Hz
            'sensor_processing_rate': 30.0, # Hz
            'max_cpu_usage': 80.0,          # Percentage
            'max_memory_usage': 80.0,       # Percentage
        }

    def optimize_for_real_time(self, node):
        """
        Optimize navigation node for real-time performance
        """
        # Use real-time scheduling
        import os
        import sched
        import time

        # Set appropriate QoS profiles
        # Use dedicated threads for different components
        # Optimize data structures for speed
        pass

    def profile_performance(self, component_name, func, *args, **kwargs):
        """
        Profile performance of a component
        """
        import time
        start_time = time.perf_counter()

        result = func(*args, **kwargs)

        end_time = time.perf_counter()
        execution_time = end_time - start_time

        print(f"{component_name} execution time: {execution_time:.4f}s")

        return result, execution_time

    def optimize_footstep_planning(self):
        """
        Optimize footstep planning for real-time performance
        """
        # Use lookup tables for common calculations
        # Pre-compute stable footstep positions
        # Use efficient data structures
        pass
```

## Troubleshooting Common Issues

### Navigation Issues and Solutions

1. **Balance Instability**:
   ```bash
   # Check IMU calibration
   ros2 run imu_tools imu_calib

   # Verify CoM estimation
   ros2 topic echo /humanoid/com_state
   ```

2. **Path Planning Failures**:
   ```bash
   # Check costmap configuration
   ros2 param list | grep costmap

   # Visualize costmap in RViz
   ros2 run rviz2 rviz2
   ```

3. **Footstep Planning Issues**:
   ```bash
   # Check footstep planner status
   ros2 service call /footstep_planner/get_state lifecycle_msgs/srv/GetState

   # Verify robot kinematics
   ros2 run tf2_tools view_frames
   ```

## Summary

This chapter covered Nav2 and path planning for bipedal humanoid movement:

- Nav2 framework adaptation for humanoid robots with specific configurations
- Bipedal path planning algorithms including footstep planning
- Center of Mass trajectory planning for balance maintenance
- Sim-to-real transfer techniques using domain randomization and system identification
- Isaac Sim integration for navigation training
- Validation and testing strategies for both simulation and real robots
- Deployment considerations and safety monitoring
- Performance optimization for real-time operation

The key to successful humanoid navigation lies in properly accounting for bipedal constraints, maintaining balance during movement, and ensuring smooth sim-to-real transfer. By adapting traditional navigation approaches to consider humanoid-specific requirements, robots can navigate complex environments while maintaining stability and safety.