# Troubleshooting Guide

## Common Issues and Solutions

This guide addresses frequent problems encountered during installation, experimentation, and data analysis for the Nav2 Controller Benchmark.

---

## Table of Contents

1. [Installation Issues](#installation-issues)
2. [Simulation and Gazebo](#simulation-and-gazebo)
3. [Navigation and Controllers](#navigation-and-controllers)
4. [Data Collection and Rosbag2](#data-collection-and-rosbag2)
5. [Analysis and Metrics Computation](#analysis-and-metrics-computation)
6. [Performance and Resource Issues](#performance-and-resource-issues)
7. [Data Quality and Validation](#data-quality-and-validation)

---

## Installation Issues

### Issue: "ros2: command not found"

**Symptoms**:
```bash
$ ros2 --version
ros2: command not found
```

**Cause**: ROS 2 environment not sourced

**Solutions**:

1. **Manual Sourcing** (temporary):
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Automatic Sourcing** (permanent):
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Verify Installation**:
   ```bash
   ls /opt/ros/humble/setup.bash
   # Should exist if ROS 2 installed correctly
   ```

4. **If File Missing** → Reinstall ROS 2:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

---

### Issue: "Package 'ros-humble-navigation2' not found"

**Symptoms**:
```bash
$ sudo apt install ros-humble-navigation2
Package ros-humble-navigation2 is not available
```

**Causes**:
1. ROS 2 apt repository not configured
2. Package name typo
3. APT cache outdated

**Solutions**:

1. **Add ROS 2 Repository** (if missing):
   ```bash
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
   
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
     http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

2. **Update APT Cache**:
   ```bash
   sudo apt update
   ```

3. **Try Full Package Name**:
   ```bash
   sudo apt install ros-humble-nav2-bringup
   ```

4. **Check Available Packages**:
   ```bash
   apt search ros-humble-nav2 | grep nav2
   ```

---

### Issue: Python ModuleNotFoundError for 'rclpy'

**Symptoms**:
```python
ModuleNotFoundError: No module named 'rclpy'
```

**Cause**: Python not finding ROS 2 packages

**Solutions**:

1. **Source ROS 2 Environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   python3 -c "import rclpy; print('Success')"
   ```

2. **Check Python Path**:
   ```bash
   python3 -c "import sys; print('\\n'.join(sys.path))"
   # Should include /opt/ros/humble/lib/python3.10/site-packages
   ```

3. **Add to PYTHONPATH** (if missing):
   ```bash
   export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH
   echo "export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:\$PYTHONPATH" >> ~/.bashrc
   ```

4. **Use ROS 2 Python** (not system Python):
   ```bash
   /opt/ros/humble/bin/python3 your_script.py
   ```

---

### Issue: colcon build fails with "No CMAKE_CXX_COMPILER"

**Symptoms**:
```
CMake Error: CMAKE_CXX_COMPILER not set
```

**Cause**: C++ compiler not installed

**Solution**:
```bash
sudo apt install build-essential cmake g++
```

**Verify**:
```bash
g++ --version
# Should show: g++ (Ubuntu 11.4.0-1ubuntu1~22.04) 11.4.0
```

---

## Simulation and Gazebo

### Issue: Gazebo Window is Blank or Black

**Symptoms**:
- Gazebo opens but shows empty/black 3D viewport
- No robot or environment visible
- Console shows: "Failed to load SDF file"

**Causes**:
1. World file not found
2. Model resources not in Gazebo path
3. GPU driver issues

**Solutions**:

1. **Check World File Exists**:
   ```bash
   ls ~/ros2_ws/src/kevin_nav2_experiments/worlds/warehouse.sdf
   ```

2. **Verify Gazebo Resource Path**:
   ```bash
   echo $IGN_GAZEBO_RESOURCE_PATH
   # Should include: ~/.ignition/fuel
   ```

3. **Add Resource Path** (if missing):
   ```bash
   export IGN_GAZEBO_RESOURCE_PATH=~/.ignition/fuel:$IGN_GAZEBO_RESOURCE_PATH
   echo "export IGN_GAZEBO_RESOURCE_PATH=~/.ignition/fuel:\$IGN_GAZEBO_RESOURCE_PATH" >> ~/.bashrc
   ```

4. **Copy World Files to Gazebo Fuel**:
   ```bash
   mkdir -p ~/.ignition/fuel/fuel.ignitionrobotics.org/ClaudioUrrea/models
   cp ~/ros2_ws/src/kevin_nav2_experiments/worlds/* \
     ~/.ignition/fuel/fuel.ignitionrobotics.org/ClaudioUrrea/models/
   ```

5. **Try Software Rendering** (if GPU issues):
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   ign gazebo warehouse.sdf
   ```

6. **Check Gazebo Logs**:
   ```bash
   tail -50 ~/.ignition/gazebo/6/log/*.log
   # Look for "Failed to load" or "SDF error" messages
   ```

---

### Issue: Gazebo Runs Very Slowly (<20 FPS)

**Symptoms**:
- Simulation frame rate drops below 20 FPS
- Real-time factor < 0.8
- Robot motion is jerky

**Causes**:
1. Insufficient CPU/GPU resources
2. Too many physics updates per frame
3. Complex collision meshes

**Solutions**:

1. **Reduce Physics Update Rate**:
   Edit world file to decrease physics frequency:
   ```xml
   <physics name="default_physics" type="ode">
     <max_step_size>0.01</max_step_size>  <!-- Was 0.001 -->
     <real_time_factor>1.0</real_time_factor>
   </physics>
   ```

2. **Run Headless** (no GUI):
   ```bash
   ign gazebo -s warehouse.sdf  # Server-only mode
   ```

3. **Simplify Models**:
   - Reduce LiDAR resolution: 360 beams → 180 beams
   - Disable shadows in rendering
   - Use simpler collision geometries

4. **Close Background Applications**:
   ```bash
   # Free RAM
   sudo sync && echo 3 | sudo tee /proc/sys/vm/drop_caches
   
   # Kill unnecessary processes
   killall -9 firefox chrome spotify
   ```

5. **Increase Swap** (if RAM limited):
   ```bash
   sudo fallocate -l 4G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile
   ```

6. **Use Lower-Resolution Monitor** (reduces GPU load):
   ```bash
   xrandr --output HDMI-1 --mode 1280x720
   ```

---

### Issue: Robot Spawns in Wrong Position

**Symptoms**:
- Robot appears at (0, 0, 0) instead of intended start position
- Robot falls through ground
- Robot orientation is incorrect

**Causes**:
1. Initial pose not set in launch file
2. AMCL overriding spawn pose
3. World coordinate frame mismatch

**Solutions**:

1. **Set Initial Pose in Launch File**:
   ```python
   # In nav2_local_only.launch.py
   Node(
       package='nav2_amcl',
       executable='amcl',
       parameters=[{
           'set_initial_pose': True,
           'initial_pose_x': 0.0,
           'initial_pose_y': 0.0,
           'initial_pose_yaw': 0.0
       }]
   )
   ```

2. **Publish Initial Pose Manually**:
   ```bash
   ros2 topic pub /a200_0000/initialpose geometry_msgs/PoseWithCovarianceStamped \
     "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \
     orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" --once
   ```

3. **Check Ground Plane Height in SDF**:
   ```xml
   <model name="ground_plane">
     <pose>0 0 0 0 0 0</pose>  <!-- Ensure z=0 -->
   </model>
   ```

4. **Verify Map-Odom Transform**:
   ```bash
   ros2 run tf2_ros tf2_echo map odom
   # Should show stable transform (not drifting)
   ```

---

## Navigation and Controllers

### Issue: "No valid control could be found"

**Symptoms**:
```
[controller_server]: No valid control could be found. 
[controller_server]: Stopping robot.
```

**Causes**:
1. All sampled trajectories collide with obstacles
2. Controller parameters too conservative
3. Costmap shows false obstacles

**Solutions**:

**For DWB**:
1. **Increase Velocity Sampling**:
   ```yaml
   vx_samples: 30  # Was 20
   vth_samples: 60  # Was 40
   ```

2. **Reduce Obstacle Cost Weight**:
   ```yaml
   BaseObstacle.scale: 0.01  # Was 0.02
   ```

3. **Increase Simulation Horizon**:
   ```yaml
   sim_time: 2.5  # Was 1.7 seconds
   ```

**For RPP**:
1. **Enable Reversing** (if stuck):
   ```yaml
   allow_reversing: true
   ```

2. **Reduce Minimum Approach Velocity**:
   ```yaml
   min_approach_linear_velocity: 0.01  # Was 0.05
   ```

3. **Increase Lookahead Distance**:
   ```yaml
   lookahead_dist: 1.0  # Was 0.6
   ```

**For MPPI**:
1. **Increase Batch Size**:
   ```yaml
   batch_size: 3000  # Was 2000 (more exploration)
   ```

2. **Increase Temperature**:
   ```yaml
   temperature: 0.5  # Was 0.3 (more stochasticity)
   ```

3. **Reduce Constraint Critic Weight**:
   ```yaml
   ConstraintCritic.cost_weight: 2.0  # Was 4.0
   ```

**General**:
4. **Check Costmap for False Obstacles**:
   ```bash
   ros2 topic echo /a200_0000/local_costmap/costmap --field data
   # Look for unexpected high-cost cells
   ```

5. **Visualize in RViz**:
   ```bash
   rviz2 -d ~/ros2_ws/src/kevin_nav2_experiments/rviz/nav2_default.rviz
   # Check costmap, global plan, and robot footprint
   ```

---

### Issue: AMCL Localization Diverges

**Symptoms**:
- Robot pose jumps suddenly (>1 m)
- Tracking RMSE > 1.0 m
- Particle cloud spreads widely
- Navigation fails with "Extrapolation into the future" errors

**Causes**:
1. Insufficient LiDAR features for localization
2. Incorrect initial pose
3. Wheel odometry drift
4. Map-world coordinate frame mismatch

**Solutions**:

1. **Reinitialize AMCL with Correct Pose**:
   ```bash
   ros2 topic pub /a200_0000/initialpose geometry_msgs/PoseWithCovarianceStamped \
     "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \
     orientation: {w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, \
                                          0.0, 0.25, 0.0, 0.0, 0.0, 0.0, \
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.06853]}}" --once
   ```

2. **Increase Particle Count**:
   ```yaml
   amcl:
     min_particles: 1000  # Was 500
     max_particles: 3000  # Was 2000
   ```

3. **Adjust Odometry Noise Parameters** (if drift suspected):
   ```yaml
   amcl:
     odom_alpha1: 0.4  # Increase if rotation drifts
     odom_alpha2: 0.4
     odom_alpha3: 0.4  # Increase if translation drifts
     odom_alpha4: 0.4
   ```

4. **Verify LiDAR is Publishing**:
   ```bash
   ros2 topic hz /a200_0000/sensors/lidar2d_0/scan
   # Should show ~30 Hz
   
   ros2 topic echo /a200_0000/sensors/lidar2d_0/scan --field ranges | head -20
   # Should show range values (not all NaN or inf)
   ```

5. **Check TF Tree**:
   ```bash
   ros2 run tf2_tools view_frames
   # Open frames.pdf and verify map->odom->base_link chain
   ```

6. **Reduce AMCL Update Frequency** (if computational bottleneck):
   ```yaml
   amcl:
     update_min_d: 0.5  # Was 0.2 (update less frequently)
     update_min_a: 0.785  # Was 0.5236 (45° instead of 30°)
   ```

---

### Issue: Controller Oscillates Excessively

**Symptoms**:
- Robot "wiggles" back and forth
- Heading oscillation count > 50
- Angular velocity sign changes rapidly
- Navigation time 2-3× expected

**Controller-Specific Solutions**:

**DWB**:
```yaml
# Increase path-following bias
PathAlign.scale: 48.0  # Was 32.0
PathDist.scale: 48.0   # Was 32.0

# Reduce goal distance bias (less aggressive approach)
GoalDist.scale: 16.0   # Was 24.0

# Increase oscillation penalty
Oscillation.scale: 2.0  # Was 1.0
```

**RPP**:
```yaml
# Increase lookahead distance (smoother turns)
lookahead_dist: 0.9      # Was 0.6
min_lookahead_dist: 0.6  # Was 0.4

# Reduce angular acceleration
max_angular_accel: 2.0   # Was 3.2

# Enable velocity-scaled lookahead
use_velocity_scaled_lookahead_dist: true
lookahead_time: 2.0      # Was 1.5
```

**MPPI**:
```yaml
# Increase temperature (smoother exploration)
temperature: 0.5         # Was 0.3

# Reduce batch size (less noisy samples)
batch_size: 1500         # Was 2000

# Increase path alignment weight
PathAlignCritic.cost_weight: 15.0  # Was 10.0
PathFollowCritic.cost_weight: 7.0  # Was 5.0
```

**General Diagnostics**:
```bash
# Plot angular velocity over time
python3 -c "
import pandas as pd
import matplotlib.pyplot as plt

# Load rosbag2 data (simplified)
# omega = extract_angular_velocity_from_bag('trial.db3')

plt.plot(omega)
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.title('Oscillation Behavior')
plt.axhline(0, color='r', linestyle='--')
plt.show()
"
```

---

## Data Collection and Rosbag2

### Issue: Rosbag2 Files Not Created

**Symptoms**:
- `go_to_goal.py` finishes but no .db3 file appears
- Results directory is empty
- No error messages shown

**Causes**:
1. Incorrect results directory path
2. Permission denied (directory not writable)
3. rosbag2 writer not initialized

**Solutions**:

1. **Check Directory Exists and is Writable**:
   ```bash
   mkdir -p ~/ros2_ws/test_results
   chmod 755 ~/ros2_ws/test_results
   ls -ld ~/ros2_ws/test_results
   # Should show: drwxr-xr-x
   ```

2. **Test rosbag2 Recording Manually**:
   ```bash
   cd ~/ros2_ws/test_results
   ros2 bag record -o test_bag /a200_0000/platform/odom/filtered
   # Press Ctrl+C after 5 seconds
   ls -lh
   # Should show test_bag directory with .db3 file
   ```

3. **Check Disk Space**:
   ```bash
   df -h ~
   # Ensure >5 GB free
   ```

4. **Run with Debug Output**:
   ```bash
   python3 go_to_goal.py --debug \
     --controller rpp \
     --trial 1 \
     --results-dir ~/ros2_ws/test_results \
     --goal 3.5 2.0 0.0
   # Check for error messages in console
   ```

---

### Issue: "Dropped X messages on topic Y"

**Symptoms**:
```
[rosbag2_storage_sqlite3]: Dropped 142 messages on topic /sensors/lidar2d_0/scan
```

**Causes**:
1. Disk I/O too slow (HDD bottleneck)
2. rosbag2 buffer overflow
3. QoS mismatch between publisher and subscriber

**Solutions**:

1. **Increase rosbag2 Cache Size**:
   ```python
   # In go_to_goal.py
   writer.open(
       storage_options,
       converter_options,
       max_cache_size=200000000  # 200 MB (was 100 MB)
   )
   ```

2. **Use SSD Instead of HDD**:
   ```bash
   # Move results to SSD
   mkdir -p /mnt/ssd/results
   ln -s /mnt/ssd/results ~/ros2_ws/results
   ```

3. **Reduce Recording Rate for High-Frequency Topics**:
   ```bash
   # Throttle LiDAR scans to 10 Hz instead of 30 Hz
   ros2 topic hz /a200_0000/sensors/lidar2d_0/scan --rate 10
   ```

4. **Use QoS Overrides**:
   ```python
   from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
   
   qos = QoSProfile(
       reliability=ReliabilityPolicy.BEST_EFFORT,  # Accept drops
       durability=DurabilityPolicy.VOLATILE,
       depth=10
   )
   
   writer.create_topic(topic_metadata, qos)
   ```

5. **Compress rosbag2 Files** (slower writing, smaller size):
   ```python
   storage_options = StorageOptions(
       uri=bag_path,
       storage_id='sqlite3',
       compression_mode='zstd',  # Enable compression
       compression_level=3
   )
   ```

---

### Issue: Rosbag2 Files Are Unusually Small (<30 MB)

**Symptoms**:
- .db3 file is 5-15 MB (expected: 50-60 MB)
- Analysis fails with "Insufficient data"
- ros2 bag info shows duration <5 seconds

**Causes**:
1. Navigation finished too quickly (< 5 seconds) → unlikely
2. Recording stopped prematurely
3. Topics not publishing during navigation

**Solutions**:

1. **Check Bag Duration**:
   ```bash
   ros2 bag info trial.db3
   # Look for: Duration: X.Y s
   # Expected: 10-120 seconds per trial
   ```

2. **Verify Topics Were Recorded**:
   ```bash
   ros2 bag info trial.db3 | grep "Topic information"
   # Should list all 9 expected topics
   ```

3. **Check Message Counts**:
   ```bash
   ros2 bag info trial.db3
   # /platform/odom/filtered: ~500-6000 messages (50 Hz × duration)
   # /sensors/lidar2d_0/scan: ~300-3600 messages (30 Hz × duration)
   ```

4. **Ensure Recording Starts Before Navigation**:
   ```python
   # In go_to_goal.py
   writer.open(...)
   time.sleep(1.0)  # Allow rosbag2 to initialize
   publish_goal()   # Then start navigation
   ```

5. **Check for Early Termination**:
   Look for errors in terminal output:
   - Timeout reached
   - Collision detected
   - Nav2 crash

---

## Analysis and Metrics Computation

### Issue: analyze_bags_extended.py Fails with "No module named 'rosbag2_py'"

**Symptoms**:
```python
ModuleNotFoundError: No module named 'rosbag2_py'
```

**Cause**: ROS 2 environment not sourced before running analysis

**Solution**:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 analyze_bags_extended.py ...
```

**Alternative**: Use shebang with ROS 2 Python:
```python
#!/opt/ros/humble/bin/python3
# First line of analyze_bags_extended.py
```

---

### Issue: "KeyError: 't_arrival_used_s'" in Analysis

**Symptoms**:
```python
KeyError: 't_arrival_used_s'
```

**Cause**: Metric computation failed (robot didn't reach goal)

**Solutions**:

1. **Check Success Rate**:
   ```python
   import pandas as pd
   df = pd.read_csv('metrics_full.csv')
   success_rate = df['success_used'].sum() / len(df)
   print(f"Success rate: {success_rate:.1%}")
   
   # Identify failed trials
   failed = df[df['success_used'] == 0]
   print(f"Failed trials: {len(failed)}")
   print(failed[['controller', 'direction', 'trial']])
   ```

2. **Exclude Failed Trials**:
   ```python
   df_success = df[df['success_used'] == 1]
   # Continue analysis with successful trials only
   ```

3. **Investigate Failed Trials**:
   ```bash
   # Replay bag to see what went wrong
   ros2 bag play failed_trial.db3
   ```

4. **Adjust Arrival Criteria** (if timeouts common):
   ```bash
   python3 analyze_bags_extended.py \
     --pos-radius 0.75  # Increase from 0.50 (more lenient)
     --pos-hold 0.20    # Decrease from 0.30 (faster detection)
     ...
   ```

---

### Issue: Tracking RMSE is Unreasonably High (>1.0 m)

**Symptoms**:
- `tracking_rms_pos_m` values are 1.5-5.0 m
- Much higher than expected (0.1-0.2 m)
- Consistent across all trials

**Causes**:
1. Plan selection issue (wrong plan used as reference)
2. Frame alignment failure (odometry not aligned to map)
3. Localization divergence (AMCL lost track)

**Solutions**:

1. **Check Plan Selection**:
   ```python
   # In analyze_bags_extended.py, add debug print
   print(f"Selected plan length: {global_plan_len_m:.2f} m")
   print(f"Plan timestamp: {plan.header.stamp}")
   ```

2. **Verify Frame Alignment**:
   ```python
   # Check Procrustes alignment residual
   aligned_odom = procrustes_se2(odom_poses, amcl_poses)
   residual_rms = sqrt(mean((aligned - amcl)**2))
   print(f"Alignment residual: {residual_rms:.3f} m")
   # Expected: <0.05 m for good localization
   ```

3. **Visualize Trajectory vs Plan**:
   ```python
   import matplotlib.pyplot as plt
   
   plt.plot([p.x for p in plan], [p.y for p in plan], 'b-', label='Plan')
   plt.plot([p.x for p in traj], [p.y for p in traj], 'r--', label='Trajectory')
   plt.legend()
   plt.axis('equal')
   plt.show()
   ```

4. **Adjust Plan Selection Window**:
   ```bash
   python3 analyze_bags_extended.py \
     --plan-select-window-s 3.0  # Increase from 2.0
     ...
   ```

---

## Performance and Resource Issues

### Issue: System Runs Out of RAM During Experiments

**Symptoms**:
- "Cannot allocate memory" errors
- System becomes unresponsive
- Gazebo crashes with SIGKILL

**Causes**:
1. MPPI batch size too large (high memory usage)
2. Multiple trials running simultaneously
3. Insufficient RAM for Nav2 + Gazebo

**Solutions**:

1. **Reduce MPPI Batch Size**:
   ```yaml
   mppi:
     batch_size: 1000  # Reduce from 2000
   ```

2. **Increase Swap Space**:
   ```bash
   # Create 8 GB swap file
   sudo fallocate -l 8G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile
   
   # Make permanent
   echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
   ```

3. **Monitor RAM Usage**:
   ```bash
   # Terminal 1: Run experiment
   python3 auto_experiment_runner.py ...
   
   # Terminal 2: Monitor resources
   watch -n 1 free -h
   # If RAM usage >90%, terminate and reduce batch size
   ```

4. **Close Unnecessary Applications**:
   ```bash
   killall -9 firefox chrome spotify slack
   ```

5. **Run Trials Sequentially** (not in parallel):
   - Ensure `auto_experiment_runner.py` waits for each trial to finish
   - Do not run multiple benchmark scripts simultaneously

---

### Issue: Disk Space Fills Up During Data Collection

**Symptoms**:
```
No space left on device
```

**Causes**:
- 270 trials × 60 MB/trial = ~16 GB
- Plus analysis outputs, figures, logs

**Solutions**:

1. **Check Disk Space Before Starting**:
   ```bash
   df -h ~
   # Ensure >30 GB free
   ```

2. **Clean Up Previous Results**:
   ```bash
   # Archive old results
   tar -czf results_backup_$(date +%Y%m%d).tar.gz ~/ros2_ws/results/
   
   # Move to external drive
   mv results_backup_*.tar.gz /mnt/external/
   
   # Delete original
   rm -rf ~/ros2_ws/results/
   ```

3. **Use External Drive for Results**:
   ```bash
   mkdir -p /mnt/external/benchmark_results
   ln -s /mnt/external/benchmark_results ~/ros2_ws/results
   ```

4. **Compress rosbag2 Files**:
   ```bash
   # After collection, compress with zstd
   for bag in *.db3; do
       zstd -19 $bag -o ${bag}.zst
       rm $bag  # Remove original
   done
   ```

5. **Delete Logs and Temporary Files**:
   ```bash
   rm -rf ~/.ros/log/*
   rm -rf ~/.ignition/gazebo/*/log/*
   ```

---

## Data Quality and Validation

### Issue: High Variance in Temporal Metrics (CV > 30%)

**Symptoms**:
- Coefficient of variation (std/mean) > 0.30 for time-to-goal
- Some trials take 2-3× longer than others
- Inconsistent performance within same controller

**Causes**:
1. AMCL initialization variance
2. MPPI stochastic sampling
3. Dynamic obstacle timing (Scene 1)
4. System load variations

**Solutions**:

1. **Compute Coefficient of Variation**:
   ```python
   import pandas as pd
   df = pd.read_csv('metrics_full.csv')
   
   cv = df.groupby('controller')['t_arrival_used_s'].std() / \
        df.groupby('controller')['t_arrival_used_s'].mean()
   
   print(f"CV for time-to-goal:\n{cv}")
   # Expected: <0.20 for well-controlled experiments
   ```

2. **Use Median Instead of Mean** (more robust):
   ```python
   summary = df.groupby('controller').agg({
       't_arrival_used_s': ['median', 'mean', 'std']
   })
   ```

3. **Increase Repetitions** (if CV > 0.30):
   ```bash
   # Re-run with 20 repetitions instead of 15
   python3 auto_experiment_runner.py --trials 20 ...
   ```

4. **Ensure Consistent System Load**:
   - Close background applications
   - Disable automatic updates: `sudo systemctl disable unattended-upgrades`
   - Use fixed CPU governor: `sudo cpupower frequency-set -g performance`

5. **Check for Outliers**:
   ```python
   import matplotlib.pyplot as plt
   
   plt.boxplot([df[df['controller'] == c]['t_arrival_used_s'] 
                for c in ['DWB', 'RPP', 'MPPI']])
   plt.xticks([1, 2, 3], ['DWB', 'RPP', 'MPPI'])
   plt.ylabel('Time to Goal (s)')
   plt.title('Outlier Detection')
   plt.show()
   ```

---

### Issue: Success Rate < 100%

**Symptoms**:
- Some trials timeout without reaching goal
- `success_used` column has 0 values
- Overall success rate 85-95% instead of 100%

**Causes**:
1. Nav2 unable to find valid path (obstacles blocking)
2. AMCL localization divergence
3. Controller gets stuck oscillating
4. Timeout too short (< total time needed)

**Solutions**:

1. **Identify Failed Trials**:
   ```python
   import pandas as pd
   df = pd.read_csv('metrics_full.csv')
   
   failed = df[df['success_used'] == 0]
   print(f"Failed: {len(failed)} / {len(df)} ({len(failed)/len(df):.1%})")
   print(failed[['controller', 'scene', 'direction', 'trial']])
   ```

2. **Replay Failed Bags**:
   ```bash
   ros2 bag play failed_trial.db3
   # Watch in Gazebo/RViz to see what went wrong
   ```

3. **Increase Timeout**:
   ```python
   # In go_to_goal.py
   TIMEOUT = 180  # Increase from 120 seconds
   ```

4. **Relax Controller Parameters** (temporarily):
   - Increase DWB simulation horizon
   - Enable RPP reversing
   - Increase MPPI temperature

5. **Re-run Failed Trials**:
   ```bash
   # Manually re-run specific trials that failed
   python3 go_to_goal.py --controller dwb --trial 7 ...
   python3 return_to_start.py --controller dwb --trial 7 ...
   ```

6. **Analyze Failure Patterns**:
   ```python
   # Check if failures are controller-specific
   failure_rate = df.groupby('controller')['success_used'].apply(lambda x: 1 - x.mean())
   print(f"Failure rate by controller:\n{failure_rate}")
   ```

---

## Getting Help

### Diagnostic Information to Include

When reporting issues, please provide:

1. **System Information**:
   ```bash
   uname -a
   lsb_release -a
   ros2 doctor --report
   ```

2. **Package Versions**:
   ```bash
   ros2 pkg list | grep nav2
   dpkg -l | grep ros-humble-navigation2
   ```

3. **Logs**:
   ```bash
   # ROS 2 logs
   cat ~/.ros/log/latest/nav2_controller_server/*.log
   
   # Gazebo logs
   cat ~/.ignition/gazebo/6/log/*.log
   ```

4. **Error Messages**: Copy full error output (not just last line)

5. **Configuration Files**: Attach YAML files used

### Contact

- **GitHub Issues**: https://github.com/ClaudioUrrea/DWB-RPP-MPPI/issues
- **Email**: claudio.urrea@usach.cl, kevin.valencia@usach.cl
- **ROS Discourse**: https://discourse.ros.org/ (tag: nav2)

---

**Last Updated**: January 26, 2026  
**Authors**: Claudio Urrea, Kevin Valencia-Aragón  
**Institution**: University of Santiago of Chile (USACH)
