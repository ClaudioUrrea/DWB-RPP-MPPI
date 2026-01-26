# Methodology

## Complete Experimental Methodology for Nav2 Controller Benchmark

This document provides comprehensive details of the experimental methodology used to systematically evaluate DWB, RPP, and MPPI local navigation controllers in semi-structured environments.

---

## Table of Contents

1. [Experimental Design](#experimental-design)
2. [Robot Platform and Simulation](#robot-platform-and-simulation)
3. [Perception and Localization](#perception-and-localization)
4. [Controller Configurations](#controller-configurations)
5. [Evaluation Scenarios](#evaluation-scenarios)
6. [Data Collection Protocol](#data-collection-protocol)
7. [Metrics Computation](#metrics-computation)
8. [Statistical Analysis](#statistical-analysis)
9. [Reproducibility](#reproducibility)

---

## Experimental Design

### Research Questions

1. **RQ1**: How do different local planning paradigms (sampling-based DWB, path-following RPP, optimization-based MPPI) perform in semi-structured environments?

2. **RQ2**: What are the fundamental tradeoffs between navigation speed, tracking accuracy, control smoothness, and safety margins?

3. **RQ3**: In what regimes is controller performance limited by perception quality versus controller logic?

### Hypothesis

**H1**: RPP achieves faster navigation due to aggressive goal-directed behavior but exhibits higher oscillations.

**H2**: MPPI produces smoother control commands at the cost of increased computation and slower execution.

**H3**: DWB provides balanced performance with moderate speed and tracking accuracy.

**H4**: In perception-limited scenarios (e.g., slopes with 2D LiDAR), tracking errors converge across controllers, indicating upstream bottlenecks.

### Experimental Factors

**Independent Variables (Fixed)**:
- **Controller Type**: DWB, RPP, MPPI (categorical, 3 levels)
- **Direction**: Forward (S→G), Return (G→S) (categorical, 2 levels)
- **Scene**: 1 (warehouse), 2 (slopes), 3 (narrow corridor) (categorical, 3 levels)

**Controlled Variables (Held Constant)**:
- Localization: AMCL with identical parameterization
- Global Planner: NavFn (A* search)
- Costmap: Static + Obstacle + Inflation layers, fixed resolution (0.05 m/cell)
- Sensor: 2D planar LiDAR (30 Hz, 360°, 0.25° resolution)
- Robot: Clearpath Husky A200 (differential drive, 0.99 m × 0.67 m footprint)
- Simulation: Gazebo Ignition Fortress, physics timestep 0.001 s

**Dependent Variables (Measured)**:
- 45+ performance metrics across temporal, spatial, tracking, safety, control, and smoothness categories (see [METRICS_REFERENCE.md](METRICS_REFERENCE.md))

### Sample Size and Repetitions

**Design**: 3 (controllers) × 3 (scenes) × 2 (directions) × 15 (repetitions) = **270 trials**

**Repetitions**: 15 independent trials per controller-direction-scene cell
- Rationale: Sufficient for robust nonparametric statistics (median, IQR)
- Power analysis (Mann-Whitney U): 80% power to detect medium effect sizes (Cliff's δ ≥ 0.33) at α = 0.05

**Trial Order**: Fully crossed within each scene
- Execution sequence: All trials for Controller A, then B, then C
- Within controller: Alternating forward/return to balance system state
- No randomization of controller order (fixed: DWB → RPP → MPPI) to ensure consistent Nav2 warm-up

---

## Robot Platform and Simulation

### Clearpath Husky A200

**Kinematics**:
- Type: Differential drive (skid-steer approximation)
- Wheelbase: 0.512 m
- Track width: 0.555 m

**Physical Dimensions**:
- Length: 0.990 m
- Width: 0.670 m
- Height: 0.390 m
- Mass: 50 kg (simulated)

**Actuation Limits** (simulated):
- Maximum linear velocity: 1.0 m/s
- Maximum angular velocity: 2.0 rad/s
- Maximum linear acceleration: 0.5 m/s²
- Maximum angular acceleration: 3.5 rad/s²

**Sensors**:
- 2D Planar LiDAR (Hokuyo UTM-30LX equivalent):
  - Range: 0.1-30 m
  - Field of view: 360° (full rotation)
  - Angular resolution: 0.25°
  - Scan frequency: 30 Hz
  - Mounting height: 0.2 m above ground

### Gazebo Ignition Fortress

**Physics Engine**: Open Dynamics Engine (ODE)
- Time step: 0.001 s (1000 Hz)
- Real-time factor: 1.0 (enforced)
- Gravity: 9.81 m/s² (standard Earth gravity)
- Contact dynamics: Soft-contact model with friction coefficient 0.8

**Simulation Fidelity**:
- Kinematic constraints: Enforced via joint limits and friction
- Sensor noise: Gaussian noise added to LiDAR (σ = 0.01 m)
- Odometry drift: Integrated wheel encoder noise (σ_v = 0.01 m/s, σ_ω = 0.01 rad/s)

**Rendering**: Optional (experiments run headless for consistency)

---

## Perception and Localization

### Localization: Adaptive Monte Carlo Localization (AMCL)

**Parameters** (identical across all trials):
```yaml
amcl:
  ros__parameters:
    min_particles: 500
    max_particles: 2000
    kld_err: 0.05
    kld_z: 0.99
    update_min_d: 0.2        # Minimum translation for update
    update_min_a: 0.5236     # Minimum rotation for update (30°)
    resample_interval: 1     # Resample every update
    
    # Motion model (differential drive)
    odom_alpha1: 0.2         # Rotation noise from rotation
    odom_alpha2: 0.2         # Rotation noise from translation
    odom_alpha3: 0.2         # Translation noise from translation
    odom_alpha4: 0.2         # Translation noise from rotation
    
    # Sensor model
    laser_max_beams: 60      # Subsample LiDAR (360 beams → 60)
    laser_sigma_hit: 0.2     # Measurement noise
    laser_z_hit: 0.5
    laser_z_rand: 0.5
    
    # Initialization
    set_initial_pose: true
    initial_pose_x: 0.0
    initial_pose_y: 0.0
    initial_pose_yaw: 0.0
    initial_pose_covariance:
      [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.06853]
```

**Convergence**: Verified by monitoring particle spread (standard deviation < 0.1 m)

### Costmap Configuration

**Global Costmap** (for global planning):
- Resolution: 0.05 m/cell
- Size: Dynamic (matches map)
- Update frequency: 1.0 Hz
- Layers: Static map only

**Local Costmap** (for local planning):
- Resolution: 0.05 m/cell
- Width: 5.0 m
- Height: 5.0 m
- Update frequency: 5.0 Hz
- Publish frequency: 2.0 Hz

**Layers** (in order):
1. **Static Layer**: Pre-loaded occupancy grid map
2. **Obstacle Layer**: LiDAR-based dynamic obstacles
   - Observation sources: `/sensors/lidar2d_0/scan`
   - Raycasting: Enabled (clear free space)
   - Clearing: Enabled (obstacle decay)
   - Max obstacle height: 2.0 m
   - Min obstacle height: 0.0 m
3. **Inflation Layer**: Distance-based cost inflation
   - Inflation radius: 0.55 m
   - Cost scaling factor: 10.0
   - Cost decay: Exponential

**Cost Values**:
- 0: Free space
- 1-127: Low cost (influenced by inflation)
- 128-252: Inflation region
- 253: Inscribed obstacle (footprint contact)
- 254: Lethal obstacle (collision)
- 255: Unknown (unmapped area)

### Global Planner: NavFn

**Algorithm**: A* search with Euclidean heuristic

**Parameters**:
```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5           # Goal tolerance
      use_astar: true          # A* vs Dijkstra
      allow_unknown: true      # Traverse unknown cells
```

**Plan Generation**: On-demand when goal received, updated if robot deviates significantly from plan

---

## Controller Configurations

All controllers were tuned for **fair comparison**, meaning:
1. Similar maximum velocities (within 10%)
2. Similar safety margins (inflation radius, clearance thresholds)
3. Comparable computational budgets (where applicable)

### DWB (Dynamic Window Approach)

**Core Parameters**:
```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      
      # Velocity limits
      min_vel_x: 0.0
      max_vel_x: 0.5           # Conservative for fair comparison
      min_vel_y: 0.0
      max_vel_y: 0.0           # Differential drive (no lateral motion)
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      
      # Acceleration limits
      acc_lim_x: 0.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -0.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      
      # Simulation
      vx_samples: 20
      vy_samples: 1            # Holonomic not used
      vth_samples: 40
      sim_time: 1.7            # Forward simulation horizon
      sim_granularity: 0.025
      angular_sim_granularity: 0.025
      
      # Critics (weighted objectives)
      critics: [
        "RotateToGoal",
        "Oscillation",
        "BaseObstacle",
        "GoalAlign",
        "PathAlign",
        "PathDist",
        "GoalDist"
      ]
      
      # Critic weights
      PathAlign.scale: 32.0
      PathDist.scale: 32.0
      GoalAlign.scale: 24.0
      GoalDist.scale: 24.0
      BaseObstacle.scale: 0.02
      Oscillation.scale: 1.0
      RotateToGoal.scale: 0.0  # Not used in our scenarios
      
      # Obstacle handling
      BaseObstacle.sum_scores: false
```

**Tuning Rationale**:
- Lower max velocity (0.5 m/s vs 1.0 m/s limit) to match RPP/MPPI achievable speeds
- High path alignment weight to enforce tracking
- Balanced goal distance vs path distance for smooth approach

### RPP (Regulated Pure Pursuit)

**Core Parameters**:
```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      
      # Lookahead behavior
      desired_linear_vel: 0.5
      lookahead_dist: 0.6
      min_lookahead_dist: 0.4
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      use_velocity_scaled_lookahead_dist: true
      
      # Curvature constraints
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 0.6
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_collision_detection: true
      
      # Rotation handling
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.785  # 45°
      rotate_to_heading_angular_vel: 1.8
      max_angular_accel: 3.2
      
      # Regulation
      use_cost_regulated_linear_velocity_scaling: true
      inflation_cost_scaling_factor: 3.0
      cost_scaling_dist: 0.6
      cost_scaling_gain: 1.0
      
      # Path handling
      transform_tolerance: 0.1
      use_interpolation: true
      
      # Limits
      max_robot_pose_search_dist: 10.0
      allow_reversing: false
```

**Tuning Rationale**:
- Adaptive lookahead (0.4-0.9 m) balances smoothness and responsiveness
- Cost-regulated velocity scaling prevents high-speed near obstacles
- Rotate-to-heading improves orientation accuracy at goal

### MPPI (Model Predictive Path Integral)

**Core Parameters**:
```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      
      # Time horizon
      time_steps: 56           # Discrete steps
      model_dt: 0.1            # 100 ms timestep → 5.6s horizon
      batch_size: 2000         # Number of sampled trajectories
      
      # Velocity limits
      vx_std: 0.2
      vy_std: 0.0              # Differential drive
      wz_std: 0.4
      vx_max: 0.5
      vx_min: -0.35            # Allow some backing
      vy_max: 0.0
      wz_max: 1.9
      
      # Optimization
      temperature: 0.3         # Lower = more exploitation
      gamma: 0.015             # Discount factor
      
      # Motion model
      motion_model: "DiffDrive"
      
      # Critics (objective functions)
      critics: [
        "ConstraintCritic",
        "CostCritic", 
        "GoalCritic",
        "GoalAngleCritic",
        "PathAlignCritic",
        "PathFollowCritic",
        "PathAngleCritic",
        "PreferForwardCritic"
      ]
      
      # Critic weights
      ConstraintCritic.cost_weight: 4.0
      ConstraintCritic.cost_power: 1
      CostCritic.cost_weight: 3.81
      CostCritic.cost_power: 1
      GoalCritic.cost_weight: 5.0
      GoalCritic.cost_power: 1
      GoalAngleCritic.cost_weight: 3.0
      GoalAngleCritic.cost_power: 1
      PathAlignCritic.cost_weight: 10.0
      PathAlignCritic.cost_power: 1
      PathFollowCritic.cost_weight: 5.0
      PathFollowCritic.cost_power: 1
      PathAngleCritic.cost_weight: 2.0
      PathAngleCritic.cost_power: 1
      PreferForwardCritic.cost_weight: 5.0
      PreferForwardCritic.cost_power: 1
      
      # Threading
      iteration_count: 1
      prune_distance: 1.7
      transform_tolerance: 0.1
      enforce_path_inversion: false
      inversion_xy_tolerance: 0.2
      inversion_yaw_tolerance: 0.4
```

**Tuning Rationale**:
- 2000 samples balance exploration and computation time (~30-50 ms per cycle)
- Temperature 0.3 provides moderate stochasticity
- High path alignment weight ensures tracking fidelity
- Forward preference reduces unnecessary backing

---

## Evaluation Scenarios

### Scene 1: Structured Corridor with Dynamic Obstacle

**Purpose**: Evaluate dynamic obstacle avoidance and costmap clearing behavior

**Environment**:
- Layout: 10m × 8m rectangular warehouse
- Obstacle: Single cylindrical object (radius 0.3 m, height 1.0 m)
- Obstacle motion: Lateral movement at 0.2 m/s for 5 seconds, then static
- Obstacle trajectory: Crosses robot's planned path on S→G leg

**Start Position**: (0.0, 0.0, 0.0) [x, y, yaw in radians]
**Goal Position**: (3.5, 2.0, 0.0)

**Path Length**: ~4.2 m (straight-line distance with minor curvature)

**Challenges**:
1. Detect and avoid moving obstacle
2. Resume navigation after obstacle clears
3. Handle residual inflation in costmap (temporal lag)

### Scene 2: Semi-Structured Slopes

**Purpose**: Stress-test perception-limited navigation (2D LiDAR on 3D terrain)

**Environment**:
- Layout: Cave-like terrain with undulating surface
- Slopes: ±15° grade in multiple sections
- Surface: Uneven (varied elevation 0-1.5 m)
- Perceptual challenge: Ground plane not parallel to LiDAR scan plane

**Start Position**: (0.0, 0.0, 0.0)
**Goal Position**: (5.0, 3.0, 0.0)

**Path Length**: ~6.8 m (accounting for terrain-induced path elongation)

**Challenges**:
1. False obstacles from elevation changes detected as occupied cells
2. Odometry drift on slopes (wheel slip)
3. Costmap bias toward overly conservative paths

**Expected Outcome**: Tracking errors compress across controllers, revealing perception bottleneck

### Scene 3: Narrow Corridor

**Purpose**: Test tight-space navigation and inflation radius effects

**Environment**:
- Layout: 12m × 2m corridor with 90° turn
- Corridor width: 1.2-1.5 m (robot width: 0.67 m → clearance 0.25-0.4 m per side)
- Walls: Static obstacles (lethal cost)

**Start Position**: (0.0, 0.0, 0.0)
**Goal Position**: (4.0, 1.5, 0.0)

**Path Length**: ~5.1 m

**Challenges**:
1. Navigate with minimal lateral clearance
2. Negotiate 90° turn without collision
3. Balance speed vs safety in constrained space

**Expected Outcome**: Higher oscillations, slower speeds, increased high-cost exposure

---

## Data Collection Protocol

### Rosbag2 Recording

**Storage Backend**: SQLite3 (`.db3` file format)

**Recording Trigger**: Programmatic start/stop synchronized with goal publication and arrival detection

**Topics Recorded** (per trial):
```yaml
/a200_0000/platform/odom/filtered:
  type: nav_msgs/Odometry
  frequency: 50 Hz
  purpose: Robot pose and velocity (filtered by EKF)

/a200_0000/amcl_pose:
  type: geometry_msgs/PoseWithCovarianceStamped
  frequency: 2 Hz
  purpose: Localization estimate (primary pose for metrics)

/a200_0000/plan:
  type: nav_msgs/Path
  frequency: Variable (on replan)
  purpose: Global plan from NavFn planner

/a200_0000/received_global_plan:
  type: nav_msgs/Path
  frequency: Variable
  purpose: Plan at controller (for robust selection)

/a200_0000/goal_pose:
  type: geometry_msgs/PoseStamped
  frequency: On demand (once per trial)
  purpose: Navigation goal (defines t₀)

/a200_0000/cmd_vel:
  type: geometry_msgs/Twist
  frequency: 20-30 Hz (controller-dependent)
  purpose: Velocity commands (for control effort metrics)

/a200_0000/local_costmap/costmap:
  type: nav_msgs/OccupancyGrid
  frequency: 5 Hz (update) / 2 Hz (publish)
  purpose: Local obstacle map (for safety metrics)

/a200_0000/sensors/lidar2d_0/scan:
  type: sensor_msgs/LaserScan
  frequency: 30 Hz
  purpose: LiDAR measurements (for clearance metrics)

/clock:
  type: rosgraph_msgs/Clock
  frequency: 1000 Hz
  purpose: Simulation time (for temporal synchronization)
```

**QoS Settings**:
- Reliability: Best-effort for high-rate topics (scan, odom)
- Reliability: Reliable for low-rate topics (goal, plan)
- Durability: Volatile (no persistence requirement)
- History depth: 10 samples

### Temporal Synchronization

**Time Origin (t₀)**: First timestamp of `/goal_pose` message
- Rationale: Marks start of navigation task (controller activation)
- Fallback: If goal message missing, use first odometry timestamp

**Arrival Detection**:
```python
def check_arrival(pose, goal, params):
    # Euclidean distance
    dist = sqrt((pose.x - goal.x)² + (pose.y - goal.y)²)
    
    # Position criterion
    if dist <= params.pos_radius:  # Default: 0.50 m
        if time_in_radius >= params.pos_hold:  # Default: 0.30 s
            position_satisfied = True
    
    # Yaw criterion (optional, not enforced by default)
    yaw_error = abs(normalize_angle(pose.yaw - goal.yaw))
    if yaw_error <= params.yaw_tolerance:  # Default: 15° = 0.262 rad
        if time_at_yaw >= params.yaw_hold:  # Default: 0.20 s
            yaw_satisfied = True
    
    # Arrival if position satisfied (yaw optional)
    return position_satisfied
```

**Arrival Time (t_arrival)**: First instant when arrival criteria met and sustained for dwell time

**Signal Cropping**: All messages filtered to [t₀, t_arrival] for consistent temporal window

### Data Validation

**Automatic Checks** (during recording):
1. ✅ Rosbag2 writer initialized successfully
2. ✅ All expected topics are publishing
3. ✅ Message timestamps are monotonically increasing
4. ✅ No "dropped messages" warnings
5. ✅ File size increases continuously (not stuck)

**Post-Recording Validation**:
```bash
# Check file integrity
ros2 bag info <bagfile>.db3

# Expected output:
# - Duration: 10-120 seconds (scene-dependent)
# - Message count: >5000 total messages
# - Topics: 9 topics present
# - No warnings or errors
```

---

## Metrics Computation

All metrics are computed offline from rosbag2 recordings using `analyze_bags_extended.py`.

See [METRICS_REFERENCE.md](METRICS_REFERENCE.md) for complete mathematical formulations.

### Preprocessing Steps

1. **Load rosbag2 SQLite3 database**:
   ```python
   from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
   
   reader = SequentialReader()
   storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
   converter_options = ConverterOptions(
       input_serialization_format='cdr',
       output_serialization_format='cdr'
   )
   reader.open(storage_options, converter_options)
   ```

2. **Deserialize messages**:
   ```python
   from rclpy.serialization import deserialize_message
   from rosidl_runtime_py.utilities import get_message
   
   msg_type = get_message(topic_type)
   msg = deserialize_message(serialized_data, msg_type)
   ```

3. **Define time origin (t₀)**:
   ```python
   # Primary method
   goal_msgs = [msg for msg in all_messages if msg.topic == '/goal_pose']
   t0 = goal_msgs[0].timestamp if goal_msgs else odometry_msgs[0].timestamp
   ```

4. **Detect arrival (t_arrival)**:
   ```python
   def find_arrival_time(poses, goal, t0, params):
       in_radius_start = None
       for i, pose in enumerate(poses):
           dist = euclidean_distance(pose, goal)
           if dist <= params.pos_radius:
               if in_radius_start is None:
                   in_radius_start = pose.timestamp
               dwell = pose.timestamp - in_radius_start
               if dwell >= params.pos_hold:
                   return pose.timestamp  # Arrival detected
           else:
               in_radius_start = None  # Reset if leaves radius
       return None  # No arrival (timeout)
   ```

5. **Crop all signals to [t₀, t_arrival]**:
   ```python
   filtered_msgs = [msg for msg in all_messages 
                    if t0 <= msg.timestamp <= t_arrival]
   ```

### Spatial Alignment (SE(2) Procrustes)

**Purpose**: Align odometry frame to AMCL (map) frame for tracking error computation

**Method**: Procrustes alignment minimizing squared error between AMCL and odometry poses

```python
import numpy as np

def procrustes_se2(odom_xy, amcl_xy):
    """
    Find optimal SE(2) transform: odom_aligned = R @ odom + t
    Minimizes: Σ ||R @ odom_i + t - amcl_i||²
    """
    # Centroids
    odom_mean = np.mean(odom_xy, axis=0)
    amcl_mean = np.mean(amcl_xy, axis=0)
    
    # Center data
    odom_centered = odom_xy - odom_mean
    amcl_centered = amcl_xy - amcl_mean
    
    # Covariance matrix
    H = odom_centered.T @ amcl_centered
    
    # SVD
    U, S, Vt = np.linalg.svd(H)
    
    # Optimal rotation
    R = Vt.T @ U.T
    
    # Handle reflections (ensure det(R) = 1)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    # Optimal translation
    t = amcl_mean - R @ odom_mean
    
    return R, t

# Apply to all odometry data
odom_aligned = [R @ pose[:2] + t for pose in odom_poses]
```

**Validation**: Residual RMS should be <0.05 m for good localization

### Robust Global Plan Selection

**Problem**: Late-arriving short plans (during goal approach) bias efficiency metrics

**Solution**: Select most geometrically extensive plan in window [t₀, t₀ + Δ]

```python
def select_robust_plan(plan_messages, t0, delta=2.0):
    """
    Select longest plan published in [t0, t0+delta] window.
    Rationale: Early plans are more representative of full trajectory.
    """
    candidates = [p for p in plan_messages 
                  if t0 <= p.timestamp <= t0 + delta]
    
    if not candidates:
        # Fallback: use first available plan
        return plan_messages[0] if plan_messages else None
    
    # Compute geometric length for each plan
    lengths = [path_length(p.poses) for p in candidates]
    
    # Return plan with maximum length
    return candidates[np.argmax(lengths)]

def path_length(poses):
    """Euclidean length of polyline."""
    if len(poses) < 2:
        return 0.0
    return sum(euclidean_distance(poses[i], poses[i-1]) 
               for i in range(1, len(poses)))
```

**Impact**: Reduces efficiency variance by ~30% compared to "last plan" method

---

## Statistical Analysis

### Descriptive Statistics

**Per-Trial Metrics** (270 rows in `metrics_full.csv`):
- All 45+ metrics computed for each trial
- Stored in CSV format for maximum compatibility

**Summary Statistics** (aggregated):
- **Median**: Robust central tendency (less sensitive to outliers than mean)
- **IQR** (Interquartile Range): Robust dispersion (Q3 - Q1)
- **Mean**: Average (for comparison with literature)
- **Std** (Standard Deviation): Dispersion around mean
- **Min/Max**: Range

**Aggregation Levels**:
1. By controller (3 rows: DWB, RPP, MPPI)
2. By controller × direction (6 rows)
3. By controller × scene (9 rows)

### Inferential Statistics

#### Scheirer-Ray-Hare Test (Nonparametric 2-Way ANOVA on Ranks)

**Purpose**: Test main effects of Controller, Direction, and their interaction

**Hypotheses**:
- H₀_A: No effect of Controller
- H₀_B: No effect of Direction
- H₀_AB: No interaction between Controller and Direction

**Test Statistic**:
```
H_A = (12 / (N(N+1))) × Σᵢ (Rᵢ² / nᵢ) - 3(N+1)
H_B = (12 / (N(N+1))) × Σⱼ (Cⱼ² / nⱼ) - 3(N+1)
H_AB = (12 / (N(N+1))) × Σᵢⱼ (Rᵢⱼ² / nᵢⱼ) - 3(N+1) - H_A - H_B

Where:
- N = total sample size (90 per scene)
- Rᵢ = sum of ranks for level i of factor A
- nᵢ = sample size for level i
- Rᵢⱼ = sum of ranks for cell (i,j)
```

**Tie Correction**:
```
C = 1 - Σ(t³ - t) / (N³ - N)
H_corrected = H / C

Where t = size of each tie group
```

**P-Value**: χ² approximation with df = (levels - 1)
- df_A = 2 (3 controllers - 1)
- df_B = 1 (2 directions - 1)
- df_AB = 2 (2 × 1 interaction)

**Effect Size**:
```
ε² = H / (N - 1)

Interpretation:
- Small: ε² ≥ 0.01
- Medium: ε² ≥ 0.06
- Large: ε² ≥ 0.14
```

**Implementation** (see `inferencial.py`):
```python
from scipy.stats import rankdata, chi2

def scheirer_ray_hare(df, y, factor_a, factor_b, tie_correct=True):
    # Rank all values (average ranks for ties)
    ranks = rankdata(df[y], method='average')
    df['_rank'] = ranks
    N = len(df)
    
    # Sum of ranks by factor levels
    R_i = df.groupby(factor_a)['_rank'].sum()
    n_i = df.groupby(factor_a)['_rank'].count()
    
    C_j = df.groupby(factor_b)['_rank'].sum()
    n_j = df.groupby(factor_b)['_rank'].count()
    
    R_ij = df.groupby([factor_a, factor_b])['_rank'].agg(['sum', 'count'])
    
    # Compute H statistics
    H_A = (12 / (N * (N+1))) * sum(R_i**2 / n_i) - 3*(N+1)
    H_B = (12 / (N * (N+1))) * sum(C_j**2 / n_j) - 3*(N+1)
    H_AB_total = (12 / (N * (N+1))) * sum(R_ij['sum']**2 / R_ij['count']) - 3*(N+1)
    H_AB = H_AB_total - H_A - H_B
    
    # Tie correction
    if tie_correct:
        C = tie_correction_factor(df[y])
        H_A /= C
        H_B /= C
        H_AB /= C
    
    # P-values
    p_A = chi2.sf(H_A, df=len(R_i)-1)
    p_B = chi2.sf(H_B, df=len(C_j)-1)
    p_AB = chi2.sf(H_AB, df=(len(R_i)-1)*(len(C_j)-1))
    
    # Effect sizes
    eps2_A = H_A / (N - 1)
    eps2_B = H_B / (N - 1)
    eps2_AB = H_AB / (N - 1)
    
    return {
        'H_A': H_A, 'p_A': p_A, 'eps2_A': eps2_A,
        'H_B': H_B, 'p_B': p_B, 'eps2_B': eps2_B,
        'H_AB': H_AB, 'p_AB': p_AB, 'eps2_AB': eps2_AB
    }
```

#### Mann-Whitney U Test (Post-Hoc Pairwise Comparisons)

**Purpose**: Identify which specific controller pairs differ significantly

**When Applied**: Only if Controller main effect significant (p < 0.05) in Scheirer-Ray-Hare

**Stratification**:
- If interaction significant (p < 0.05): Test within each direction separately
- If interaction not significant: Pool across directions

**Pairwise Comparisons**: 3 pairs (DWB vs RPP, DWB vs MPPI, RPP vs MPPI)

**Test Statistic** (Mann-Whitney U):
```python
from scipy.stats import mannwhitneyu

def mannwhitney_pairwise(x, y, alternative='two-sided'):
    U, p = mannwhitneyu(x, y, alternative=alternative, method='auto')
    return U, p
```

**Multiple Comparison Correction**: Holm step-down procedure
```python
def holm_correction(p_values):
    """
    Holm step-down correction for FWER control.
    Adjusts p-values to control family-wise error rate at α.
    """
    m = len(p_values)
    sorted_indices = np.argsort(p_values)
    sorted_p = np.array(p_values)[sorted_indices]
    
    adjusted_p = np.zeros(m)
    running_max = 0.0
    
    for k in range(m):
        factor = (m - k)
        adj = min(1.0, factor * sorted_p[k])
        running_max = max(running_max, adj)
        adjusted_p[k] = running_max
    
    # Map back to original order
    result = np.empty(m)
    result[sorted_indices] = adjusted_p
    
    return result.tolist()
```

**Effect Size** (Cliff's Delta):
```python
def cliffs_delta(x, y):
    """
    Cliff's delta: δ = P(X > Y) - P(X < Y)
    
    Interpretation:
    - |δ| < 0.147: Negligible
    - 0.147 ≤ |δ| < 0.33: Small
    - 0.33 ≤ |δ| < 0.474: Medium
    - |δ| ≥ 0.474: Large
    """
    n_x = len(x)
    n_y = len(y)
    
    greater = sum(1 for xi in x for yj in y if xi > yj)
    less = sum(1 for xi in x for yj in y if xi < yj)
    
    return (greater - less) / (n_x * n_y)
```

**Significance Criterion**: p_holm < 0.05

---

## Reproducibility

### Determinism Sources

**Deterministic Components**:
1. ✅ Robot kinematics (fixed differential drive model)
2. ✅ Sensor placement (LiDAR always at 0.2 m height)
3. ✅ World geometry (SDF files are static)
4. ✅ Nav2 parameters (YAML files version-controlled)
5. ✅ Analysis pipeline (Python scripts deterministic)

**Non-Deterministic Components**:
1. ❌ AMCL particle initialization (random particle spread)
2. ❌ MPPI sampling (stochastic rollouts)
3. ❌ Floating-point arithmetic (minor CPU-dependent variations)
4. ❌ Simulation timing (minor OS scheduler effects)

### Minimizing Variability

**AMCL Initialization**:
- Always start at same pose: (0, 0, 0)
- Use tight initial covariance: σ_xy = 0.5 m, σ_yaw = 15°
- Wait for convergence before starting navigation (particle spread < 0.1 m)

**MPPI Temperature**:
- Fixed temperature = 0.3 (moderate stochasticity)
- No adaptation during trial

**Simulation Timing**:
- Real-time factor = 1.0 (enforced)
- Physics timestep = 0.001 s (consistent)
- Use simulation clock (`/clock` topic) for all timing

**Expected Variability**:
- Temporal metrics: ±5-10% coefficient of variation (CV)
- Spatial metrics: ±3-5% CV
- Count metrics: 0-2 count difference (nearly deterministic)

### Verification Protocol

To verify reproducibility:

1. **Re-run 5 trials** for each controller in Scene 1
2. **Compare medians**:
   ```python
   original_median = 15.23  # seconds (time-to-goal)
   reproduced_median = 15.18
   relative_diff = abs(reproduced - original) / original
   # Expect: relative_diff < 0.10 (10%)
   ```
3. **Check statistical consistency**:
   - Rank order preserved (RPP fastest, then DWB, then MPPI)
   - Effect sizes within 20% (Cliff's δ ± 0.1)

---

## Limitations and Threats to Validity

### Internal Validity

**Threats**:
1. **Simulation Fidelity**: Gazebo approximations (soft contacts, simplified friction)
   - *Mitigation*: Use validated Husky model, calibrate physics parameters
2. **Nav2 Version Dependency**: Different Nav2 releases may yield different results
   - *Mitigation*: Document exact versions (Humble), use containerization (future work)

**Assessment**: Internal validity is **high** due to controlled environment and consistent parameterization

### External Validity

**Threats**:
1. **Simulation-to-Reality Gap**: Real robots have actuator delays, wear, battery effects
   - *Mitigation*: Focus on *relative* controller comparison (lessens sim2real impact)
2. **Platform-Specific**: Results apply to differential-drive robots (Husky-class)
   - *Generalization*: May not extend to holonomic or Ackermann platforms
3. **Scene Specificity**: Three scenes may not cover all real-world navigation contexts
   - *Future Work*: Expand to outdoor unstructured, multi-agent scenarios

**Assessment**: External validity is **moderate**; results are indicative but require real-world validation

### Statistical Conclusion Validity

**Threats**:
1. **Sample Size**: n=15 per cell may miss small effects
   - *Mitigation*: Use nonparametric tests (fewer assumptions), report effect sizes
2. **Multiple Comparisons**: 45 metrics × 3 scenes = 135 tests → inflated Type I error
   - *Mitigation*: Apply Holm correction, focus on pre-registered hypotheses

**Assessment**: Statistical conclusion validity is **high** due to robust methods and correction procedures

---

## Contact

For questions about methodology or replication:
- **Email**: claudio.urrea@usach.cl, kevin.valencia@usach.cl
- **GitHub Issues**: https://github.com/ClaudioUrrea/DWB-RPP-MPPI/issues

---

**Last Updated**: January 26, 2026  
**Authors**: Claudio Urrea, Kevin Valencia-Aragón  
**Institution**: University of Santiago of Chile (USACH)
