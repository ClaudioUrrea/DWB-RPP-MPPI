# Metrics Reference

## Mathematical Formulations and Computation Methods

This document provides complete mathematical definitions and computation algorithms for all 45+ performance metrics used in the benchmark. For high-level descriptions, see [DATA_DICTIONARY.md](DATA_DICTIONARY.md).

---

## Table of Contents

1. [Notation and Conventions](#notation-and-conventions)
2. [Temporal Metrics](#temporal-metrics)
3. [Spatial and Efficiency Metrics](#spatial-and-efficiency-metrics)
4. [Tracking Accuracy Metrics](#tracking-accuracy-metrics)
5. [Safety and Collision Proximity Metrics](#safety-and-collision-proximity-metrics)
6. [Control Effort Metrics](#control-effort-metrics)
7. [Smoothness and Comfort Metrics](#smoothness-and-comfort-metrics)
8. [Velocity Profile Metrics](#velocity-profile-metrics)
9. [Implementation Notes](#implementation-notes)

---

## Notation and Conventions

### Coordinate Frames

- **Map Frame** (`map`): Fixed global reference frame (origin at map origin)
- **Odometry Frame** (`odom`): Local frame integrated from wheel encoders
- **Base Link Frame** (`base_link`): Robot's body-fixed frame (center of wheelbase)

### State Variables

- **Pose**: $\mathbf{p}_t = (x_t, y_t, \theta_t) \in SE(2)$ at time $t$
  - $x_t, y_t$: Position in map frame (meters)
  - $\theta_t$: Heading angle (radians, $\theta \in [-\pi, \pi]$)

- **Velocity**: $\mathbf{v}_t = (v_t, \omega_t)$ at time $t$
  - $v_t$: Linear velocity (m/s)
  - $\omega_t$: Angular velocity (rad/s)

- **Commands**: $\mathbf{u}_t = (v^{\text{cmd}}_t, \omega^{\text{cmd}}_t)$ at time $t$
  - $v^{\text{cmd}}_t$: Commanded linear velocity (m/s)
  - $\omega^{\text{cmd}}_t$: Commanded angular velocity (rad/s)

### Time Variables

- $t_0$: Navigation start time (goal publication timestamp)
- $t_{\text{arr}}$: Arrival time (first instant meeting arrival criteria)
- $\Delta t$: Time step between samples (variable, typically 0.02-0.05 s)
- $T = t_{\text{arr}} - t_0$: Total navigation duration

### Path Representations

- **Global Plan**: $\mathcal{P} = \{\mathbf{p}_1, \mathbf{p}_2, \ldots, \mathbf{p}_M\}$ where $M$ is number of waypoints
- **Executed Trajectory**: $\mathcal{T} = \{\mathbf{p}_{t_0}, \mathbf{p}_{t_0 + \Delta t}, \ldots, \mathbf{p}_{t_{\text{arr}}}\}$ with $N$ samples

### Distance Functions

- **Euclidean Distance**: $d(\mathbf{p}_1, \mathbf{p}_2) = \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}$
- **Angular Distance**: $d_\theta(\theta_1, \theta_2) = |\text{normalize}(\theta_2 - \theta_1)|$ where normalize maps to $[-\pi, \pi]$

---

## Temporal Metrics

### Time to Goal

**Symbol**: $t_{\text{arr}}$  
**Unit**: seconds (s)  
**Definition**: Total navigation time from goal publication to arrival

$$
t_{\text{goal}} = t_{\text{arr}} - t_0
$$

**Arrival Criteria**:
$$
\begin{aligned}
d(\mathbf{p}_t, \mathbf{p}_{\text{goal}}) &\leq r_{\text{pos}} \quad &\text{(position within radius)} \\
\Delta t_{\text{dwell}} &\geq \tau_{\text{hold}} \quad &\text{(dwell time sustained)}
\end{aligned}
$$

**Default Parameters**:
- $r_{\text{pos}} = 0.50$ m (arrival radius)
- $\tau_{\text{hold}} = 0.30$ s (dwell time)

**Computation Algorithm**:
```python
def compute_time_to_goal(trajectory, goal, t0, r_pos=0.50, tau_hold=0.30):
    in_radius_start = None
    
    for pose in trajectory:
        dist = euclidean_distance(pose.position, goal.position)
        
        if dist <= r_pos:
            if in_radius_start is None:
                in_radius_start = pose.timestamp
            
            dwell = pose.timestamp - in_radius_start
            
            if dwell >= tau_hold:
                return pose.timestamp - t0  # Success
        else:
            in_radius_start = None  # Reset
    
    return None  # Timeout (arrival not detected)
```

### Stopped Time

**Symbol**: $t_{\text{stop}}$  
**Unit**: seconds (s)  
**Definition**: Total time spent stationary or near-stationary

$$
t_{\text{stop}} = \sum_{i=1}^{N-1} \Delta t_i \cdot \mathbb{1}_{|v_i| < v_{\text{threshold}}}
$$

where $\mathbb{1}_{\{\cdot\}}$ is the indicator function and $v_{\text{threshold}} = 0.03$ m/s (default).

**Interpretation**:
- $t_{\text{stop}} = 0$: Continuous motion (ideal)
- $t_{\text{stop}} < 2$ s: Occasional brief pauses
- $t_{\text{stop}} > 5$ s: Frequent stops (inefficient, possible obstacle blocking)

---

## Spatial and Efficiency Metrics

### Global Plan Length

**Symbol**: $L_{\text{plan}}$  
**Unit**: meters (m)  
**Definition**: Geometric length of the global reference path

$$
L_{\text{plan}} = \sum_{j=1}^{M-1} d(\mathbf{p}_j, \mathbf{p}_{j+1})
$$

where $\mathcal{P} = \{\mathbf{p}_1, \ldots, \mathbf{p}_M\}$ is the global plan.

**Robust Plan Selection**:
To avoid bias from short transient plans near goal:

$$
\mathcal{P}^* = \arg\max_{\mathcal{P}_k \in \mathcal{C}} L(\mathcal{P}_k)
$$

where $\mathcal{C} = \{\mathcal{P}_k : t_0 \leq t(\mathcal{P}_k) \leq t_0 + \Delta_{\text{window}}\}$ and $\Delta_{\text{window}} = 2.0$ s.

### Actual Path Length

**Symbol**: $L_{\text{actual}}$  
**Unit**: meters (m)  
**Definition**: Total distance traveled by robot

$$
L_{\text{actual}} = \int_{t_0}^{t_{\text{arr}}} |v(t)| \, dt \approx \sum_{i=1}^{N-1} |v_i| \cdot \Delta t_i
$$

where $v(t)$ is the linear velocity from odometry.

**Discrete Approximation**:
```python
def compute_actual_path_length(odometry_data):
    length = 0.0
    for i in range(1, len(odometry_data)):
        v = odometry_data[i].twist.linear.x
        dt = odometry_data[i].timestamp - odometry_data[i-1].timestamp
        length += abs(v) * dt
    return length
```

### Position Efficiency

**Symbol**: $\eta_{\text{pos}}$  
**Unit**: percentage (%)  
**Definition**: Ratio of plan length to actual path length

$$
\eta_{\text{pos}} = 100 \times \frac{L_{\text{plan}}}{L_{\text{actual}}}
$$

**Interpretation**:
- $\eta_{\text{pos}} = 100\%$: Perfect execution (actual path = planned path length)
- $\eta_{\text{pos}} < 100\%$: Robot traveled farther than necessary (detours, oscillations)
- $\eta_{\text{pos}} > 100\%$: Robot found shorter path (rare, indicates replanning or plan suboptimality)

**Typical Values**:
- Excellent: $\eta_{\text{pos}} \geq 95\%$
- Good: $85\% \leq \eta_{\text{pos}} < 95\%$
- Moderate: $70\% \leq \eta_{\text{pos}} < 85\%$
- Poor: $\eta_{\text{pos}} < 70\%$

---

## Tracking Accuracy Metrics

### Lateral Tracking RMSE

**Symbol**: $\text{RMSE}_{\perp}$  
**Unit**: meters (m)  
**Definition**: Root mean square perpendicular deviation from global plan

$$
\text{RMSE}_{\perp} = \sqrt{\frac{1}{N} \sum_{i=1}^{N} d_{\perp}^2(\mathbf{p}_i, \mathcal{P})}
$$

where $d_{\perp}(\mathbf{p}, \mathcal{P})$ is the minimum perpendicular distance from point $\mathbf{p}$ to plan $\mathcal{P}$.

**Point-to-Polyline Projection**:
For a point $\mathbf{p} = (x_p, y_p)$ and plan segment $[\mathbf{a}, \mathbf{b}]$:

$$
\begin{aligned}
\mathbf{v} &= \mathbf{b} - \mathbf{a} \\
\mathbf{w} &= \mathbf{p} - \mathbf{a} \\
c &= \frac{\mathbf{w} \cdot \mathbf{v}}{\mathbf{v} \cdot \mathbf{v}} \\
\lambda &= \max(0, \min(1, c)) \quad &\text{(clamp to segment)} \\
\mathbf{q} &= \mathbf{a} + \lambda \mathbf{v} \quad &\text{(projection point)} \\
d_{\perp} &= \|\mathbf{p} - \mathbf{q}\|
\end{aligned}
$$

**Algorithm**:
```python
def lateral_tracking_rmse(trajectory, plan):
    squared_errors = []
    
    for pose in trajectory:
        # Project pose onto plan
        min_dist = float('inf')
        
        for i in range(len(plan) - 1):
            a = plan[i].position
            b = plan[i+1].position
            
            # Segment vector
            vx, vy = b.x - a.x, b.y - a.y
            
            # Point-to-a vector
            wx, wy = pose.x - a.x, pose.y - a.y
            
            # Projection parameter
            v_dot_v = vx**2 + vy**2
            if v_dot_v > 0:
                c = (wx*vx + wy*vy) / v_dot_v
                lam = max(0, min(1, c))
            else:
                lam = 0
            
            # Projection point
            qx = a.x + lam * vx
            qy = a.y + lam * vy
            
            # Distance
            dist = sqrt((pose.x - qx)**2 + (pose.y - qy)**2)
            min_dist = min(min_dist, dist)
        
        squared_errors.append(min_dist**2)
    
    return sqrt(mean(squared_errors))
```

### Tracking Error Percentiles

**95th Percentile** ($\text{P95}_{\perp}$):
$$
\text{P95}_{\perp} = \text{quantile}_{0.95}(\{d_{\perp}(\mathbf{p}_i, \mathcal{P}) : i = 1, \ldots, N\})
$$

**Maximum Error** ($d_{\perp}^{\max}$):
$$
d_{\perp}^{\max} = \max_{i=1,\ldots,N} d_{\perp}(\mathbf{p}_i, \mathcal{P})
$$

### Heading Tracking RMSE

**Symbol**: $\text{RMSE}_{\theta}$  
**Unit**: radians (rad)  
**Definition**: Root mean square angular deviation from plan orientation

$$
\text{RMSE}_{\theta} = \sqrt{\frac{1}{N} \sum_{i=1}^{N} (\text{normalize}(\theta_i - \theta_{\text{plan},i}))^2}
$$

where $\theta_{\text{plan},i}$ is the orientation of the plan segment closest to $\mathbf{p}_i$.

**Normalize Function**:
$$
\text{normalize}(\theta) = (\theta + \pi) \mod 2\pi - \pi \quad \text{(maps to } [-\pi, \pi])
$$

---

## Safety and Collision Proximity Metrics

### Minimum Clearance

**Symbol**: $c_{\min}$  
**Unit**: meters (m)  
**Definition**: Minimum distance to obstacles during navigation

$$
c_{\min} = \min_{t \in [t_0, t_{\text{arr}}]} \min_{j=1,\ldots,K_t} r_{t,j}
$$

where $r_{t,j}$ is the $j$-th LiDAR range measurement at time $t$, and $K_t$ is the number of beams.

**Computation**:
```python
def minimum_clearance(lidar_scans):
    global_min = float('inf')
    
    for scan in lidar_scans:
        # Filter invalid ranges (too close, too far, or NaN)
        valid_ranges = [r for r in scan.ranges 
                        if scan.range_min <= r <= scan.range_max]
        
        if valid_ranges:
            scan_min = min(valid_ranges)
            global_min = min(global_min, scan_min)
    
    return global_min
```

### Time Below Safe Clearance

**Symbol**: $t_{c < c_{\text{safe}}}$  
**Unit**: seconds (s)  
**Definition**: Duration spent closer to obstacles than safe threshold

$$
t_{c < c_{\text{safe}}} = \sum_{i=1}^{N-1} \Delta t_i \cdot \mathbb{1}_{c_i < c_{\text{safe}}}
$$

where $c_i = \min_j r_{i,j}$ is the minimum clearance at sample $i$, and $c_{\text{safe}} = 0.35$ m (default).

**Percentage Formulation**:
$$
\%_{c < c_{\text{safe}}} = 100 \times \frac{t_{c < c_{\text{safe}}}}{T}
$$

### Costmap-Based Safety Metrics

**High-Cost Exposure Time**:
$$
t_{\text{high cost}} = \sum_{i=1}^{N-1} \Delta t_i \cdot \mathbb{1}_{\text{cost}(\mathbf{p}_i) > \tau_{\text{cost}}}
$$

where $\text{cost}(\mathbf{p})$ is the costmap value at position $\mathbf{p}$, and $\tau_{\text{cost}} = 200$ (default threshold on 0-254 scale).

**Average Costmap Cost**:
$$
\bar{C} = \frac{1}{N} \sum_{i=1}^{N} \text{cost}(\mathbf{p}_i)
$$

---

## Control Effort Metrics

### Command Effort (Control Intensity Proxy)

**Symbol**: $J_u$  
**Unit**: arbitrary units (a.u.)  
**Definition**: Integrated squared velocity commands

$$
J_u = \int_{t_0}^{t_{\text{arr}}} \left( (v^{\text{cmd}}(t))^2 + (\omega^{\text{cmd}}(t))^2 \right) dt
$$

**Discrete Approximation** (trapezoidal rule):
$$
J_u \approx \sum_{i=1}^{N-1} \frac{\Delta t_i}{2} \left[ (v^{\text{cmd}}_i)^2 + (v^{\text{cmd}}_{i+1})^2 + (\omega^{\text{cmd}}_i)^2 + (\omega^{\text{cmd}}_{i+1})^2 \right]
$$

**Interpretation**:
- Lower $J_u$: Smoother, less aggressive control
- Higher $J_u$: More dynamic, potentially energy-intensive control

**Note**: This is **not** direct energy consumption (requires torque measurements), but a reasonable proxy for comparative analysis.

### L1 and L2 Norms

**Linear Command L1**:
$$
\|\mathbf{v}^{\text{cmd}}\|_{L_1} = \int_{t_0}^{t_{\text{arr}}} |v^{\text{cmd}}(t)| \, dt \approx \sum_{i=1}^{N-1} |v^{\text{cmd}}_i| \cdot \Delta t_i
$$

**Linear Command L2**:
$$
\|\mathbf{v}^{\text{cmd}}\|_{L_2} = \sqrt{\int_{t_0}^{t_{\text{arr}}} (v^{\text{cmd}}(t))^2 \, dt} \approx \sqrt{\sum_{i=1}^{N-1} (v^{\text{cmd}}_i)^2 \cdot \Delta t_i}
$$

**Angular Command L1**:
$$
\|\boldsymbol{\omega}^{\text{cmd}}\|_{L_1} = \int_{t_0}^{t_{\text{arr}}} |\omega^{\text{cmd}}(t)| \, dt \approx \sum_{i=1}^{N-1} |\omega^{\text{cmd}}_i| \cdot \Delta t_i
$$

**Angular Command L2**:
$$
\|\boldsymbol{\omega}^{\text{cmd}}\|_{L_2} = \sqrt{\int_{t_0}^{t_{\text{arr}}} (\omega^{\text{cmd}}(t))^2 \, dt} \approx \sqrt{\sum_{i=1}^{N-1} (\omega^{\text{cmd}}_i)^2 \cdot \Delta t_i}
$$

**Usage**:
- $L_1$ norm: Total "amount" of command (magnitude-weighted time)
- $L_2$ norm: Emphasizes large commands more (quadratic penalty)

### Acceleration-Based Effort

**Linear Acceleration L1**:
$$
\|\mathbf{a}\|_{L_1} = \int_{t_0}^{t_{\text{arr}}} |a(t)| \, dt \approx \sum_{i=2}^{N-1} \left| \frac{v_i - v_{i-1}}{\Delta t_i} \right| \cdot \Delta t_i
$$

**Linear Acceleration L2**:
$$
\|\mathbf{a}\|_{L_2} = \sqrt{\int_{t_0}^{t_{\text{arr}}} a(t)^2 \, dt} \approx \sqrt{\sum_{i=2}^{N-1} \left( \frac{v_i - v_{i-1}}{\Delta t_i} \right)^2 \cdot \Delta t_i}
$$

---

## Smoothness and Comfort Metrics

### Heading Oscillations

**Symbol**: $N_{\text{osc}}$  
**Unit**: count (#)  
**Definition**: Number of sign changes in angular velocity

$$
N_{\text{osc}} = \sum_{i=2}^{N-1} \mathbb{1}_{\text{sign}(\omega_i) \neq \text{sign}(\omega_{i-1})}
$$

**Algorithm**:
```python
def count_heading_oscillations(angular_velocities, threshold=0.01):
    """
    Count sign changes in angular velocity.
    threshold: Ignore near-zero velocities to avoid noise.
    """
    count = 0
    prev_sign = None
    
    for omega in angular_velocities:
        if abs(omega) < threshold:
            continue  # Skip near-zero values
        
        curr_sign = 1 if omega > 0 else -1
        
        if prev_sign is not None and curr_sign != prev_sign:
            count += 1
        
        prev_sign = curr_sign
    
    return count
```

**Interpretation**:
- $N_{\text{osc}} < 10$: Smooth, monotonic turning
- $10 \leq N_{\text{osc}} \leq 20$: Moderate oscillations (acceptable)
- $N_{\text{osc}} > 20$: Excessive wiggling (poor trajectory quality)

### Jerk (Rate of Acceleration Change)

**Linear Jerk**:
$$
j(t) = \frac{da(t)}{dt} = \frac{d^2 v(t)}{dt^2}
$$

**Discrete Approximation**:
$$
j_i = \frac{a_{i+1} - a_i}{\Delta t_i} = \frac{1}{\Delta t_i} \left( \frac{v_{i+1} - v_i}{\Delta t_{i+1}} - \frac{v_i - v_{i-1}}{\Delta t_i} \right)
$$

**RMS Jerk** (smoothness indicator):
$$
\text{RMS}_j = \sqrt{\frac{1}{N-2} \sum_{i=2}^{N-2} j_i^2}
$$

**P95 Jerk** (robust measure excluding outliers):
$$
\text{P95}_j = \text{quantile}_{0.95}(\{|j_i| : i = 2, \ldots, N-2\})
$$

---

## Velocity Profile Metrics

### Velocity Bins

Define three velocity bins:
- **Stopped**: $v < 0.05$ m/s
- **Slow**: $0.05 \leq v \leq 0.20$ m/s
- **Fast**: $v > 0.20$ m/s

**Time Fractions**:
$$
\begin{aligned}
f_{\text{stop}} &= \frac{1}{T} \sum_{i=1}^{N-1} \Delta t_i \cdot \mathbb{1}_{v_i < 0.05} \\
f_{\text{slow}} &= \frac{1}{T} \sum_{i=1}^{N-1} \Delta t_i \cdot \mathbb{1}_{0.05 \leq v_i \leq 0.20} \\
f_{\text{fast}} &= \frac{1}{T} \sum_{i=1}^{N-1} \Delta t_i \cdot \mathbb{1}_{v_i > 0.20}
\end{aligned}
$$

**Normalization Check**:
$$
f_{\text{stop}} + f_{\text{slow}} + f_{\text{fast}} = 1.0
$$

### Velocity Statistics

**Mean Velocity**:
$$
\bar{v} = \frac{1}{T} \int_{t_0}^{t_{\text{arr}}} v(t) \, dt \approx \frac{1}{T} \sum_{i=1}^{N-1} v_i \cdot \Delta t_i
$$

**95th Percentile Velocity**:
$$
v_{95} = \text{quantile}_{0.95}(\{v_i : i = 1, \ldots, N\})
$$

**Maximum Velocity**:
$$
v_{\max} = \max_{i=1,\ldots,N} v_i
$$

---

## Implementation Notes

### Numerical Integration

**Trapezoidal Rule** (used throughout):
$$
\int_{t_0}^{t_{\text{arr}}} f(t) \, dt \approx \sum_{i=1}^{N-1} \frac{f(t_i) + f(t_{i+1})}{2} \cdot (t_{i+1} - t_i)
$$

**Python Implementation**:
```python
def trapezoidal_integrate(values, timestamps):
    integral = 0.0
    for i in range(len(values) - 1):
        dt = timestamps[i+1] - timestamps[i]
        avg = (values[i] + values[i+1]) / 2.0
        integral += avg * dt
    return integral
```

### Handling Missing Data

**Missing LiDAR Scans**:
- If scan is missing or invalid, use previous valid scan
- If no valid scans exist, set clearance = NaN (exclude from statistics)

**Missing Velocity Commands**:
- Interpolate linearly between adjacent valid commands
- If gap > 0.5 s, mark as data quality issue

**Missing Odometry**:
- If odometry is missing, mark trial as invalid (cannot compute metrics)
- Odometry is critical (primary pose source for metrics)

### Robustness to Outliers

**Why Use Median/IQR**:
- Navigation metrics often have heavy tails (e.g., timeout trials)
- Median is robust to outliers (50% breakdown point)
- IQR (Q3 - Q1) is robust dispersion measure

**When to Use Mean/Std**:
- When normality is verified (e.g., via Shapiro-Wilk test)
- For compatibility with literature (many papers report mean ± std)
- Include both in summary statistics for transparency

### Computational Complexity

**Per-Trial Analysis**:
- Time complexity: $O(N \cdot M)$ where $N$ = trajectory samples, $M$ = plan waypoints
- Typical values: $N \approx 500$-$2000$, $M \approx 50$-$200$
- Processing time: ~5-10 seconds per rosbag2 file on modern CPU

**Bottlenecks**:
1. **Point-to-polyline projection**: $O(N \cdot M)$ (dominant)
2. **SE(2) Procrustes alignment**: $O(N)$ (SVD is fast for 2D)
3. **Rosbag2 deserialization**: $O(N)$ (I/O bound)

**Optimization**:
- Use spatial indexing (KD-tree) for plan: $O(N \log M)$ instead of $O(N \cdot M)$
- Vectorize operations with NumPy (10-100× speedup over loops)

---

## Validation and Sanity Checks

### Physical Constraints

All metrics should satisfy:

$$
\begin{aligned}
0 &\leq t_{\text{goal}} \leq 120 \text{ s} \quad &\text{(timeout limit)} \\
0 &\leq L_{\text{actual}} < \infty \quad &\text{(non-negative path length)} \\
0 &\leq \eta_{\text{pos}} \leq 150\% \quad &\text{(efficiency typically < 110\%)} \\
0 &\leq \text{RMSE}_{\perp} < L_{\text{plan}} \quad &\text{(error < plan length)} \\
0.05 &\leq c_{\min} \leq 30 \text{ m} \quad &\text{(LiDAR range limits)} \\
0 &\leq N_{\text{osc}} < 200 \quad &\text{(unreasonably high if > 100)}
\end{aligned}
$$

### Cross-Metric Consistency

**Efficiency vs Path Length**:
$$
L_{\text{actual}} = \frac{L_{\text{plan}}}{\eta_{\text{pos}} / 100}
$$

**Time vs Mean Velocity**:
$$
\bar{v} \approx \frac{L_{\text{actual}}}{T}
$$

**Stopped Time vs Total Time**:
$$
t_{\text{stop}} \leq T
$$

**Velocity Fractions**:
$$
f_{\text{stop}} + f_{\text{slow}} + f_{\text{fast}} = 1.0 \pm 0.01 \quad \text{(tolerance for discretization)}
$$

---

## Example Calculation

### Scenario
- Robot navigates 5.0 m path in 12.5 seconds
- Actual path length: 5.8 m (due to obstacle avoidance)
- Trajectory has 625 samples (50 Hz odometry)
- Plan has 42 waypoints

### Computed Metrics

**Time to Goal**:
$$
t_{\text{goal}} = 12.5 \text{ s}
$$

**Efficiency**:
$$
\eta_{\text{pos}} = 100 \times \frac{5.0}{5.8} = 86.2\%
$$

**Mean Velocity**:
$$
\bar{v} = \frac{5.8}{12.5} = 0.464 \text{ m/s}
$$

**Tracking RMSE** (example):
- Sample 100 lateral deviations: $\{d_{\perp,i}\}$
- Suppose $d_{\perp,i} \in [0.02, 0.25]$ m
- $\text{RMSE}_{\perp} = \sqrt{\frac{1}{625} \sum d_{\perp,i}^2} = 0.143$ m

**Control Effort** (example):
- Linear command L2: $\|\mathbf{v}^{\text{cmd}}\|_{L_2} = 5.85$ m (integrated over 12.5 s)
- Angular command L2: $\|\boldsymbol{\omega}^{\text{cmd}}\|_{L_2} = 8.42$ rad (integrated)
- Total: $J_u = 5.85 + 8.42 = 14.27$ a.u.

---

## References

1. Fox, D., Burgard, W., & Thrun, S. (1997). The dynamic window approach to collision avoidance. *IEEE Robotics & Automation Magazine*, 4(1), 23-33.

2. Macenski, S., Singh, S., Martín, F., & Ginés, J. (2023). Regulated Pure Pursuit for robot path tracking. *Autonomous Robots*, 47(6), 685-694.

3. Williams, G., Aldrich, A., & Theodorou, E. (2017). Model Predictive Path Integral Control: From Theory to Parallel Computation. *Journal of Guidance, Control, and Dynamics*, 40(2), 344-357.

4. Scheirer, C. J., Ray, W. S., & Hare, N. (1976). The analysis of ranked data derived from completely randomized factorial designs. *Biometrics*, 32(2), 429-434.

5. Cliff, N. (1993). Dominance statistics: Ordinal analyses to answer ordinal questions. *Psychological Bulletin*, 114(3), 494-509.

---

## Contact

For questions about metric definitions or computation:
- **Email**: claudio.urrea@usach.cl, kevin.valencia@usach.cl
- **GitHub Issues**: https://github.com/ClaudioUrrea/DWB-RPP-MPPI/issues

---

**Last Updated**: January 26, 2026  
**Authors**: Claudio Urrea, Kevin Valencia-Aragón  
**Institution**: University of Santiago of Chile (USACH)
