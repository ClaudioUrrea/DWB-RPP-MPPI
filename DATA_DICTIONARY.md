# Data Dictionary

## Overview

This document provides complete definitions for all metrics, variables, and data fields used in the benchmark analysis. All metrics are computed from ROS 2 rosbag2 recordings (SQLite3 .db3 format) using the `analyze_bags_extended.py` pipeline.

## Table of Contents

1. [Trial Metadata](#trial-metadata)
2. [Temporal Metrics](#temporal-metrics)
3. [Spatial Metrics](#spatial-metrics)
4. [Tracking Accuracy Metrics](#tracking-accuracy-metrics)
5. [Safety Metrics](#safety-metrics)
6. [Control Metrics](#control-metrics)
7. [Smoothness Metrics](#smoothness-metrics)
8. [Velocity Profile Metrics](#velocity-profile-metrics)
9. [ROS 2 Topics Reference](#ros-2-topics-reference)
10. [Computation Methods](#computation-methods)

---

## Trial Metadata

### `bag_name`
- **Type**: String
- **Description**: Filename of the rosbag2 recording
- **Format**: `{controller}_{direction}_{trial}_{timestamp}.db3`
- **Example**: `dwb_forward_001_1234567890.db3`

### `controller`
- **Type**: String (Categorical)
- **Values**: `DWB`, `RPP`, `MPPI`
- **Description**: Local navigation controller used for this trial
- **Details**:
  - **DWB**: Dynamic Window Approach-based controller
  - **RPP**: Regulated Pure Pursuit controller
  - **MPPI**: Model Predictive Path Integral controller

### `direction`
- **Type**: String (Categorical)
- **Values**: `forward`, `return`
- **Description**: Navigation direction
- **Details**:
  - `forward`: Start (S) → Goal (G) leg
  - `return`: Goal (G) → Start (S) leg

### `trial`
- **Type**: Integer
- **Range**: 1-15
- **Description**: Repetition number for this controller-direction combination
- **Purpose**: Statistical robustness (15 independent runs per condition)

### `scene`
- **Type**: String (Categorical)
- **Values**: `scene1`, `scene2`, `scene3`
- **Description**: Evaluation scenario identifier
- **Details**:
  - **Scene 1**: Structured corridor with dynamic obstacle
  - **Scene 2**: Semi-structured environment with slopes (±15°)
  - **Scene 3**: Narrow corridor with tight clearances

---

## Temporal Metrics

### `time_to_goal_s`
- **Type**: Float
- **Unit**: Seconds (s)
- **Description**: Total navigation time from goal publication to arrival
- **Computation**: `t_arrival - t_0`
  - `t_0`: Timestamp of first `/goal_pose` message
  - `t_arrival`: First instant when robot enters arrival radius and dwells for required time
- **Arrival Criteria** (default):
  - Position radius: 0.50 m from goal
  - Dwell time: 0.30 s continuous presence
  - Yaw tolerance: Optional (15° default, not enforced by default)
- **Interpretation**: Lower is better (faster execution)

### `t0_unix`
- **Type**: Float
- **Unit**: Unix timestamp (seconds since epoch)
- **Description**: Absolute time when navigation task started
- **Source**: ROS 2 simulation clock (`/clock` topic)

### `t_arr_unix`
- **Type**: Float
- **Unit**: Unix timestamp (seconds since epoch)
- **Description**: Absolute time when robot arrived at goal
- **Computation**: `t0_unix + time_to_goal_s`

---

## Spatial Metrics

### `global_plan_len_m`
- **Type**: Float
- **Unit**: Meters (m)
- **Description**: Geometric length of the global reference plan
- **Computation**: Sum of Euclidean distances between consecutive waypoints in selected global plan
- **Plan Selection**: Most geometrically extensive plan published in window [t₀, t₀ + 2.0s]
- **Purpose**: Reference path length for efficiency calculations

### `actual_path_len_m`
- **Type**: Float
- **Unit**: Meters (m)
- **Description**: Total distance traveled by the robot
- **Computation**: Integration of odometry linear velocity over time
  ```
  actual_path = ∫[t0, tarr] |v(t)| dt
  ```
- **Source**: `/platform/odom/filtered` topic
- **Interpretation**: Should approximate `global_plan_len_m` for efficient navigation

### `efficiency_pos_pct`
- **Type**: Float
- **Unit**: Percentage (%)
- **Description**: Position-based path efficiency
- **Computation**: 
  ```
  efficiency_pos = 100 × (global_plan_len_m / actual_path_len_m)
  ```
- **Range**: 0-100% (can exceed 100% if actual path is shorter than plan)
- **Interpretation**: 
  - 100%: Perfect efficiency (actual path = planned path length)
  - < 100%: Robot traveled farther than necessary
  - \> 100%: Robot found shorter path (rare with global planner)

### `goal_dist_final_m`
- **Type**: Float
- **Unit**: Meters (m)
- **Description**: Final Euclidean distance from robot to goal at arrival detection
- **Computation**: `sqrt((x_final - x_goal)² + (y_final - y_goal)²)`
- **Expected Value**: ≤ arrival radius threshold (0.50 m default)
- **Purpose**: Verify arrival detection accuracy

---

## Tracking Accuracy Metrics

### `tracking_rmse_m`
- **Type**: Float
- **Unit**: Meters (m)
- **Description**: Root mean square lateral deviation from global plan
- **Computation**:
  ```
  For each pose (x_i, y_i) in robot trajectory:
    1. Project point onto global plan polyline
    2. Compute perpendicular distance d_i
  
  tracking_rmse = sqrt(mean(d_i²))
  ```
- **Interpretation**: Lower is better (tighter path following)
- **Typical Values**:
  - Good: < 0.10 m
  - Moderate: 0.10-0.20 m
  - Poor: > 0.20 m

### `tracking_max_m`
- **Type**: Float
- **Unit**: Meters (m)
- **Description**: Maximum lateral deviation from global plan during execution
- **Computation**: `max(d_i)` where `d_i` is perpendicular distance to plan at pose `i`
- **Purpose**: Identify worst-case tracking errors
- **Safety Implication**: Large values may indicate collision risk in constrained spaces

### `tracking_p95_m`
- **Type**: Float
- **Unit**: Meters (m)
- **Description**: 95th percentile lateral deviation from global plan
- **Computation**: 95th percentile of all `d_i` values
- **Purpose**: Robust measure of typical large deviations (less sensitive to outliers than max)

### `heading_rmse_rad`
- **Type**: Float
- **Unit**: Radians (rad)
- **Description**: Root mean square angular deviation from global plan orientation
- **Computation**:
  ```
  For each pose with yaw θ_i:
    1. Find corresponding plan segment with orientation θ_plan
    2. Compute angular error: e_i = normalize(θ_i - θ_plan) to [-π, π]
  
  heading_rmse = sqrt(mean(e_i²))
  ```
- **Interpretation**: Lower is better (better heading alignment)
- **Conversion**: 0.1 rad ≈ 5.7°

### `tracking_integral_m2`
- **Type**: Float
- **Unit**: Meter-squared-seconds (m²·s)
- **Description**: Integrated squared lateral tracking error over time
- **Computation**: 
  ```
  tracking_integral = ∫[t0, tarr] d(t)² dt
  ```
  Approximated via trapezoidal integration
- **Purpose**: Cumulative tracking performance metric (penalizes both magnitude and duration of errors)

---

## Safety Metrics

### `min_clearance_m`
- **Type**: Float
- **Unit**: Meters (m)
- **Description**: Minimum distance to obstacles during navigation
- **Computation**: 
  ```
  For each LiDAR scan:
    min_range = min(scan.ranges)
  
  min_clearance = min(all min_ranges)
  ```
- **Source**: `/sensors/lidar2d_0/scan` topic
- **Safety Threshold**: Configurable (default: 0.35 m)
- **Interpretation**: 
  - < 0.20 m: High collision risk
  - 0.20-0.40 m: Moderate risk
  - \> 0.40 m: Safe clearance

### `clearance_below_safe_pct`
- **Type**: Float
- **Unit**: Percentage (%)
- **Description**: Percentage of time spent below safe clearance threshold
- **Computation**:
  ```
  clearance_below_safe = 100 × (time_below_threshold / total_time)
  ```
- **Threshold**: 0.35 m (default, configurable)
- **Interpretation**: 
  - 0%: Always maintained safe distance
  - < 10%: Occasional close approaches
  - \> 20%: Frequent proximity to obstacles (high risk)

### `costmap_high_cost_pct`
- **Type**: Float
- **Unit**: Percentage (%)
- **Description**: Percentage of trajectory spent in high-cost costmap regions
- **Computation**:
  ```
  For each local costmap update:
    If cost at robot position > threshold (200 default):
      count as high-cost sample
  
  costmap_high_cost_pct = 100 × (high_cost_samples / total_samples)
  ```
- **Source**: `/local_costmap/costmap` topic
- **Cost Scale**: 0-254 (ROS 2 costmap convention)
  - 0: Free space
  - 1-127: Low cost
  - 128-252: Inflation region
  - 253: Inscribed (collision imminent)
  - 254: Lethal (obstacle)
- **Interpretation**: Higher values indicate more time in hazardous regions

### `costmap_avg_cost`
- **Type**: Float
- **Unit**: Arbitrary units (0-254)
- **Description**: Average costmap cost experienced during navigation
- **Computation**: Mean of costmap costs at robot position across all samples
- **Purpose**: General measure of navigation difficulty

### `costmap_max_cost`
- **Type**: Integer
- **Unit**: Arbitrary units (0-254)
- **Description**: Maximum costmap cost encountered
- **Purpose**: Identify if robot entered inscribed/lethal zones

---

## Control Metrics

### `cmd_effort_au`
- **Type**: Float
- **Unit**: Arbitrary units (a.u.)
- **Description**: Total control effort (command intensity proxy)
- **Computation**:
  ```
  cmd_effort = ∫[t0, tarr] v_cmd(t)² dt + ∫[t0, tarr] ω_cmd(t)² dt
  ```
  - v_cmd: Linear velocity command (m/s)
  - ω_cmd: Angular velocity command (rad/s)
- **Source**: `/cmd_vel` topic
- **Purpose**: Energy consumption proxy (NOT direct physical energy)
- **Interpretation**: Lower values indicate smoother, less aggressive control

### `cmd_lin_L1`
- **Type**: Float
- **Unit**: Meters (m)
- **Description**: L1 norm of linear velocity commands (total linear motion magnitude)
- **Computation**: 
  ```
  cmd_lin_L1 = ∫[t0, tarr] |v_cmd(t)| dt
  ```
- **Interpretation**: Higher values indicate more aggressive linear motion

### `cmd_lin_L2`
- **Type**: Float
- **Unit**: Arbitrary units (m²/s)
- **Description**: L2 norm of linear velocity commands (smoothness indicator)
- **Computation**:
  ```
  cmd_lin_L2 = sqrt(∫[t0, tarr] v_cmd(t)² dt)
  ```
- **Purpose**: Penalizes large velocity commands more than L1

### `cmd_ang_L1`
- **Type**: Float
- **Unit**: Radians (rad)
- **Description**: L1 norm of angular velocity commands
- **Computation**:
  ```
  cmd_ang_L1 = ∫[t0, tarr] |ω_cmd(t)| dt
  ```
- **Interpretation**: Higher values indicate more turning activity

### `cmd_ang_L2`
- **Type**: Float
- **Unit**: Arbitrary units (rad²/s)
- **Description**: L2 norm of angular velocity commands
- **Computation**:
  ```
  cmd_ang_L2 = sqrt(∫[t0, tarr] ω_cmd(t)² dt)
  ```

### `acc_lin_L1`
- **Type**: Float
- **Unit**: Meters per second (m/s)
- **Description**: L1 norm of linear acceleration (jerk indicator)
- **Computation**:
  ```
  For each timestep i:
    a_i = (v_i - v_{i-1}) / Δt
  
  acc_lin_L1 = ∫[t0, tarr] |a(t)| dt
  ```
- **Source**: Odometry linear velocity (`/platform/odom/filtered`)
- **Purpose**: Measures actuation abruptness (comfort metric)

### `acc_lin_L2`
- **Type**: Float
- **Unit**: Arbitrary units (m²/s²)
- **Description**: L2 norm of linear acceleration
- **Computation**:
  ```
  acc_lin_L2 = sqrt(∫[t0, tarr] a(t)² dt)
  ```

---

## Smoothness Metrics

### `stopped_time_s`
- **Type**: Float
- **Unit**: Seconds (s)
- **Description**: Total time spent stationary or near-stationary
- **Computation**:
  ```
  For each timestep:
    If |v_odom(t)| < threshold (0.03 m/s default):
      count as stopped
  
  stopped_time = sum(stopped timesteps) × Δt
  ```
- **Source**: Odometry linear velocity
- **Interpretation**: 
  - 0 s: No stops (ideal smooth motion)
  - < 2 s: Occasional pauses
  - \> 5 s: Frequent stops (inefficient)

### `heading_oscillations_count`
- **Type**: Integer
- **Unit**: Count (#)
- **Description**: Number of sign changes in angular velocity
- **Computation**:
  ```
  oscillations = 0
  For each timestep i:
    If sign(ω_i) ≠ sign(ω_{i-1}):
      oscillations += 1
  ```
- **Source**: Odometry angular velocity (`/platform/odom/filtered`)
- **Purpose**: Detect lateral "wiggling" behavior
- **Interpretation**:
  - < 10: Smooth motion
  - 10-20: Moderate oscillations
  - \> 20: Excessive wiggling (poor trajectory smoothness)

### `cmd_jerk_lin`
- **Type**: Float
- **Unit**: Meters per second cubed (m/s³)
- **Description**: Mean absolute jerk of linear velocity commands
- **Computation**:
  ```
  For each timestep i:
    jerk_i = (v_cmd_i - v_cmd_{i-1}) / Δt
  
  cmd_jerk_lin = mean(|jerk_i|)
  ```
- **Source**: `/cmd_vel` topic
- **Purpose**: Command smoothness indicator (lower is smoother)

### `cmd_jerk_ang`
- **Type**: Float
- **Unit**: Radians per second squared (rad/s²)
- **Description**: Mean absolute jerk of angular velocity commands
- **Computation**: Analogous to `cmd_jerk_lin` for angular velocity

---

## Velocity Profile Metrics

### `vel_frac_stopped_pct`
- **Type**: Float
- **Unit**: Percentage (%)
- **Description**: Fraction of time in "stopped" velocity bin
- **Computation**: 
  ```
  vel_frac_stopped = 100 × (time with v < 0.05 m/s) / total_time
  ```

### `vel_frac_slow_pct`
- **Type**: Float
- **Unit**: Percentage (%)
- **Description**: Fraction of time in "slow" velocity bin
- **Computation**:
  ```
  vel_frac_slow = 100 × (time with 0.05 ≤ v ≤ 0.20 m/s) / total_time
  ```

### `vel_frac_fast_pct`
- **Type**: Float
- **Unit**: Percentage (%)
- **Description**: Fraction of time in "fast" velocity bin
- **Computation**:
  ```
  vel_frac_fast = 100 × (time with v > 0.20 m/s) / total_time
  ```

### `vel_mean_mps`
- **Type**: Float
- **Unit**: Meters per second (m/s)
- **Description**: Mean linear velocity during navigation
- **Computation**: Average of odometry linear velocity samples

### `vel_max_mps`
- **Type**: Float
- **Unit**: Meters per second (m/s)
- **Description**: Maximum linear velocity achieved

### `vel_p95_mps`
- **Type**: Float
- **Unit**: Meters per second (m/s)
- **Description**: 95th percentile linear velocity
- **Purpose**: Robust measure of typical high-speed operation

---

## ROS 2 Topics Reference

### Topics Used in Analysis

| Topic | Message Type | Frequency | Purpose |
|-------|-------------|-----------|---------|
| `/a200_0000/platform/odom/filtered` | `nav_msgs/Odometry` | 50 Hz | Robot pose and velocity (filtered) |
| `/a200_0000/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 2 Hz | Localization estimate |
| `/a200_0000/plan` | `nav_msgs/Path` | Variable | Global plan from planner |
| `/a200_0000/received_global_plan` | `nav_msgs/Path` | Variable | Plan received by controller |
| `/a200_0000/goal_pose` | `geometry_msgs/PoseStamped` | On demand | Navigation goal |
| `/a200_0000/cmd_vel` | `geometry_msgs/Twist` | 20-30 Hz | Velocity commands to robot |
| `/a200_0000/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | 5 Hz | Local obstacle costmap |
| `/a200_0000/sensors/lidar2d_0/scan` | `sensor_msgs/LaserScan` | 30 Hz | LiDAR range measurements |
| `/clock` | `rosgraph_msgs/Clock` | 1000 Hz | Simulation time |

### Topic Name Convention

- **Namespace**: `/a200_0000` (Clearpath Husky robot identifier)
- **Hierarchy**: 
  - `/platform/*`: Core platform sensors/state
  - `/sensors/*`: Sensor data streams
  - `/*_pose`: Pose/localization topics
  - `/cmd_*`: Command topics

---

## Computation Methods

### Temporal Synchronization

1. **Time Origin (t₀)**:
   - Primary: First timestamp in `/goal_pose` topic
   - Fallback: First odometry message (if goal missing)

2. **Arrival Detection**:
   ```python
   def check_arrival(pose, goal, radius=0.5, hold_time=0.3):
       distance = euclidean_distance(pose, goal)
       if distance <= radius:
           if time_in_radius >= hold_time:
               return True
       return False
   ```

3. **Signal Cropping**:
   - All topics filtered to [t₀, t_arrival]
   - Ensures consistent temporal window across metrics

### Spatial Alignment (SE(2) Procrustes)

Purpose: Align odometry frame to AMCL (map) frame for tracking error computation.

```python
def align_odom_to_amcl(odom_poses, amcl_poses):
    # 1. Extract (x, y) coordinates
    odom_xy = [(p.x, p.y) for p in odom_poses]
    amcl_xy = [(p.x, p.y) for p in amcl_poses]
    
    # 2. Find optimal SE(2) transform (rotation + translation)
    #    Minimizes: Σ ||R·odom_i + t - amcl_i||²
    R, t = procrustes_2d(odom_xy, amcl_xy)
    
    # 3. Apply transform to all odometry data
    odom_aligned = [R @ p + t for p in odom_poses]
    
    return odom_aligned, R, t
```

### Plan Selection (Robust Method)

Avoids bias from short transient plans:

```python
def select_global_plan(plan_messages, t0, window=2.0):
    # Collect all plans in window [t0, t0 + window]
    candidate_plans = [p for p in plan_messages 
                       if t0 <= p.timestamp <= t0 + window]
    
    # Select plan with maximum geometric length
    best_plan = max(candidate_plans, key=lambda p: path_length(p.poses))
    
    return best_plan
```

**Rationale**: Late-arriving short plans (e.g., during goal approach) can artificially inflate efficiency metrics. Using the longest plan in the initial window provides a stable reference.

### Point-to-Polyline Projection

Used for lateral tracking error computation:

```python
def project_point_to_polyline(point, polyline):
    min_distance = float('inf')
    best_projection = None
    
    for i in range(len(polyline) - 1):
        A = polyline[i]
        B = polyline[i + 1]
        
        # Project point onto segment AB
        AB = B - A
        AP = point - A
        
        # Parameter t ∈ [0, 1] along segment
        t = np.clip(np.dot(AP, AB) / np.dot(AB, AB), 0, 1)
        
        # Closest point on segment
        projection = A + t * AB
        distance = np.linalg.norm(point - projection)
        
        if distance < min_distance:
            min_distance = distance
            best_projection = projection
    
    return best_projection, min_distance
```

### Integration Methods

Trapezoidal rule used for all time-integral metrics:

```python
def integrate_trapezoidal(values, timestamps):
    integral = 0.0
    for i in range(1, len(values)):
        dt = timestamps[i] - timestamps[i-1]
        integral += 0.5 * (values[i] + values[i-1]) * dt
    return integral
```

Example for control effort:
```python
cmd_effort = integrate_trapezoidal(v_cmd**2, t) + integrate_trapezoidal(omega_cmd**2, t)
```

### Statistical Aggregation

#### Per-Trial Metrics
- Each `.db3` file → One row in `metrics_full.csv`
- All metrics computed independently per trial

#### Summary Statistics
- **Grouping**: By `(controller)` or `(controller, direction)`
- **Aggregates**:
  - `median`: Robust central tendency
  - `iqr`: Interquartile range (Q3 - Q1)
  - `mean`: Average (for comparison)
  - `std`: Standard deviation
  - `min`, `max`: Range
  - `count`: Number of valid trials

Example summary computation:
```python
summary = df.groupby(['controller', 'direction']).agg({
    'time_to_goal_s': ['median', 'iqr', 'mean', 'std'],
    'tracking_rmse_m': ['median', 'iqr', 'mean', 'std'],
    # ... other metrics
})
```

---

## Data Quality Indicators

### Missing Data Handling

- **NaN Values**: Indicate metric could not be computed (e.g., no valid plan, LiDAR failure)
- **Zero Values**: May indicate actual zero (stopped) or data gap - check context
- **Inf Values**: Typically indicate division by zero or missing denominator

### Expected Value Ranges (Sanity Checks)

| Metric | Expected Range | Suspicious If |
|--------|----------------|---------------|
| `time_to_goal_s` | 5-120 s | < 3 s or > 300 s |
| `tracking_rmse_m` | 0.05-0.50 m | > 1.0 m |
| `min_clearance_m` | 0.10-2.0 m | < 0.05 m (likely sensor noise) |
| `efficiency_pos_pct` | 70-105% | < 50% or > 120% |
| `stopped_time_s` | 0-20 s | > 30 s |
| `heading_oscillations_count` | 5-50 | > 100 |

### Validation Checks

1. **Temporal Consistency**: `t_arr_unix > t0_unix`
2. **Spatial Consistency**: `actual_path_len_m ≥ goal_dist_final_m`
3. **Efficiency Bounds**: `efficiency_pos_pct` should rarely exceed 110%
4. **Velocity Fractions**: `vel_frac_stopped + vel_frac_slow + vel_frac_fast ≈ 100%`

---

## File Format Specifications

### CSV Output Format

**Filename**: `metrics_full.csv`

**Structure**:
```
controller,direction,trial,time_to_goal_s,tracking_rmse_m,...
DWB,forward,1,15.234,0.132,...
DWB,forward,2,14.891,0.145,...
...
```

**Properties**:
- Header row with metric names
- One trial per row
- Float precision: 6 decimal places (default)
- Missing values: Empty cells or `NaN`
- Encoding: UTF-8

### Parquet Output Format (Optional)

**Advantages**:
- 10-20× smaller file size
- Faster loading in pandas/R
- Preserves data types

**Schema**:
```python
schema = {
    'controller': 'category',
    'direction': 'category',
    'trial': 'int32',
    'time_to_goal_s': 'float32',
    'tracking_rmse_m': 'float32',
    # ... all metrics
}
```

---

## Reproducibility Notes

### Random Sources (None)

All metrics are **deterministic** given:
- Input rosbag2 file
- Configuration parameters (arrival thresholds, etc.)
- Analysis script version

### Configuration Parameters

Key parameters affecting metric values:

```python
DEFAULT_CONFIG = {
    "pos_radius": 0.50,        # Arrival detection radius (m)
    "pos_hold": 0.30,          # Arrival dwell time (s)
    "yaw_deg": 15.0,           # Yaw tolerance (degrees)
    "stop_speed": 0.03,        # Stopped velocity threshold (m/s)
    "clearance_safe": 0.35,    # Safe clearance threshold (m)
    "high_cost": 200,          # High-cost costmap threshold
    "plan_select_window_s": 2.0,  # Plan selection window (s)
}
```

### Version Information

- **Python**: 3.10+
- **NumPy**: 1.24+
- **Pandas**: 2.0+
- **ROS 2**: Humble Hawksbill
- **Analysis Script**: `analyze_bags_extended.py` (version in repository)

---

## Change Log

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-01 | Initial data dictionary for MDPI Systems publication |

---

## Contact

For questions about metric definitions or computation methods:
- **GitHub Issues**: [https://github.com/ClaudioUrrea/DWB-RPP-MPPI/issues](https://github.com/ClaudioUrrea/DWB-RPP-MPPI/issues)
- **Email**: claudio.urrea@usach.cl
