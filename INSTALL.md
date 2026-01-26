# Installation Guide

## System Requirements

### Minimum Requirements
- **CPU**: Intel Core i5 (4 cores) or AMD Ryzen 5
- **RAM**: 8 GB (16 GB recommended for MPPI controller)
- **Storage**: 15 GB free space
  - 5 GB for ROS 2 Humble + Nav2
  - 5 GB for Gazebo Ignition Fortress
  - 3 GB for simulation worlds and dependencies
  - 2 GB for workspace and experiments
- **GPU**: Not required (software rendering supported)
- **Internet**: Required for initial setup and package downloads

### Recommended Requirements
- **CPU**: Intel Core i7 (6+ cores) or AMD Ryzen 7
- **RAM**: 16 GB
- **Storage**: 30 GB SSD
- **GPU**: NVIDIA GTX 1050 or equivalent (for visualization)
- **Display**: 1920×1080 or higher

### Software Requirements
- **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish)
  - 64-bit architecture required
  - Fresh installation recommended (minimal virtual machine overhead)
- **Kernel**: Linux kernel 5.15 or newer
- **Python**: 3.10+ (included in Ubuntu 22.04)

---

## Step-by-Step Installation

### Step 1: System Preparation

#### 1.1 Update System Packages
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl gnupg lsb-release git build-essential
```

#### 1.2 Install Development Tools
```bash
sudo apt install -y \
  python3-pip \
  python3-dev \
  python3-venv \
  cmake \
  pkg-config \
  libssl-dev \
  libeigen3-dev
```

---

### Step 2: Install ROS 2 Humble

#### 2.1 Add ROS 2 Repository
```bash
# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package index
sudo apt update
```

#### 2.2 Install ROS 2 Humble Desktop (Full)
```bash
sudo apt install -y ros-humble-desktop
```

**Installation time**: ~10-15 minutes depending on internet speed.

#### 2.3 Install ROS 2 Development Tools
```bash
sudo apt install -y \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool
```

#### 2.4 Initialize rosdep
```bash
sudo rosdep init
rosdep update
```

#### 2.5 Setup Environment (Add to ~/.bashrc)
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Verify Installation**:
```bash
ros2 --version
# Expected output: ros2 cli version: 0.18.x
```

---

### Step 3: Install Gazebo Ignition Fortress

#### 3.1 Add Gazebo Repository
```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable \
  $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'

wget https://packages.osrfoundation.org/gazebo.gpg -O - | sudo apt-key add -

sudo apt update
```

#### 3.2 Install Gazebo Ignition Fortress
```bash
sudo apt install -y ignition-fortress
```

**Installation time**: ~5-10 minutes.

#### 3.3 Install Gazebo-ROS 2 Bridge
```bash
sudo apt install -y ros-humble-ros-gz
```

**Verify Installation**:
```bash
ign gazebo --version
# Expected output: Ignition Gazebo, version 6.x.x
```

---

### Step 4: Install Navigation2

#### 4.1 Install Nav2 Packages
```bash
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-msgs \
  ros-humble-nav2-core \
  ros-humble-nav2-common \
  ros-humble-nav2-dwb-controller \
  ros-humble-nav2-regulated-pure-pursuit-controller \
  ros-humble-nav2-mppi-controller
```

**Installation time**: ~3-5 minutes.

#### 4.2 Verify Nav2 Installation
```bash
ros2 pkg list | grep nav2
# Should show multiple nav2_* packages
```

---

### Step 5: Create and Setup Workspace

#### 5.1 Create ROS 2 Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

#### 5.2 Clone Repository
```bash
git clone https://github.com/ClaudioUrrea/DWB-RPP-MPPI.git kevin_nav2_experiments
cd kevin_nav2_experiments
```

#### 5.3 Install Python Dependencies
```bash
pip3 install -r requirements.txt
```

**requirements.txt contents**:
```
numpy>=1.24.0
pandas>=2.0.0
matplotlib>=3.7.0
scipy>=1.10.0
rosbags>=0.9.15
```

#### 5.4 Install Additional ROS 2 Dependencies
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

### Step 6: Build Workspace

#### 6.1 Source ROS 2 Environment
```bash
source /opt/ros/humble/setup.bash
```

#### 6.2 Build Workspace
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

**Build time**: ~2-5 minutes (first build).

**Troubleshooting Build Errors**:
- If errors occur, check that all dependencies are installed
- Try cleaning build: `rm -rf build/ install/ log/`
- Rebuild: `colcon build --symlink-install`

#### 6.3 Source Workspace
```bash
source ~/ros2_ws/install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

**Verify Workspace**:
```bash
ros2 pkg list | grep kevin_nav2
# Should show kevin_nav2_experiments package
```

---

### Step 7: Configure Simulation Environments

#### 7.1 Copy World Files to Gazebo Resource Path
```bash
mkdir -p ~/.ignition/fuel/fuel.ignitionrobotics.org/ClaudioUrrea/models
cp -r ~/ros2_ws/src/kevin_nav2_experiments/worlds/* \
  ~/.ignition/fuel/fuel.ignitionrobotics.org/ClaudioUrrea/models/
```

#### 7.2 Verify World Files
```bash
ls ~/ros2_ws/src/kevin_nav2_experiments/worlds/
# Expected output: warehouse.sdf, cave_circuit_02_subset_r75_ignfix.sdf, final_event_07.sdf
```

---

### Step 8: Test Installation

#### 8.1 Launch Simulation (Terminal 1)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch kevin_nav2_experiments nav2_local_only.launch.py \
  namespace:=a200_0000 \
  use_sim_time:=true \
  params_file:=~/ros2_ws/src/kevin_nav2_experiments/configs/nav2_rpp_fair.yaml \
  world:=warehouse
```

**Expected behavior**: Gazebo window opens showing Husky robot in warehouse environment.

#### 8.2 Check ROS 2 Topics (Terminal 2)
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
```

**Expected topics include**:
- `/a200_0000/platform/odom/filtered`
- `/a200_0000/amcl_pose`
- `/a200_0000/cmd_vel`
- `/a200_0000/plan`
- `/a200_0000/local_costmap/costmap`
- `/clock`

#### 8.3 Test Navigation Command (Terminal 3)
```bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/kevin_nav2_experiments/scripts/go_to_goal.py \
  --controller rpp \
  --trial 1 \
  --results-dir ~/ros2_ws/test_results \
  --goal 3.5 2.0 0.0
```

**Expected behavior**: Robot navigates to goal, rosbag2 file created in `~/ros2_ws/test_results/`.

#### 8.4 Verify Rosbag2 Recording
```bash
ls ~/ros2_ws/test_results/
# Should show .db3 file(s)

# Inspect bag contents
ros2 bag info ~/ros2_ws/test_results/rpp_forward_001_*.db3
```

---

## Configuration Files

### Controller Configuration Files Location
```
~/ros2_ws/src/kevin_nav2_experiments/configs/
├── nav2_dwb_fair.yaml
├── nav2_rpp_fair.yaml
└── nav2_mppi_fair.yaml
```

### World Files Location
```
~/ros2_ws/src/kevin_nav2_experiments/worlds/
├── warehouse.sdf
├── warehouse.yaml
├── cave_circuit_02_subset_r75_ignfix.sdf
├── map_name.pgm
├── map_name.yaml
├── final_event_07.sdf
└── map5.yaml
```

### Scripts Location
```
~/ros2_ws/src/kevin_nav2_experiments/scripts/
├── analyze_bags_extended.py
├── auto_experiment_runner.py
├── go_to_goal.py
└── return_to_start.py
```

---

## Environment Variables

### Required Environment Variables

Add to `~/.bashrc`:
```bash
# ROS 2 Setup
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Gazebo Setup
export IGNITION_VERSION=fortress
export GZ_VERSION=fortress

# Python Path (if needed)
export PYTHONPATH="${PYTHONPATH}:~/ros2_ws/install/lib/python3.10/site-packages"

# Optional: Increase Gazebo physics update rate for smoother simulation
export IGN_GAZEBO_RESOURCE_PATH=~/.ignition/fuel
```

Reload bashrc:
```bash
source ~/.bashrc
```

---

## Verification Checklist

After completing installation, verify:

- [ ] `ros2 --version` shows Humble
- [ ] `ign gazebo --version` shows Fortress 6.x
- [ ] `ros2 pkg list | grep nav2` shows Nav2 packages
- [ ] `ros2 topic list` shows topics when simulation is running
- [ ] Gazebo simulation window opens without errors
- [ ] Robot model appears in Gazebo
- [ ] RViz2 shows costmap and robot model (if using RViz)
- [ ] Python scripts execute without import errors
- [ ] Rosbag2 files are created in results directory
- [ ] `ros2 bag info` command works on recorded bags

---

## Troubleshooting

### Issue: Gazebo Fails to Launch
**Symptoms**: "Failed to load SDF file" or blank Gazebo window

**Solutions**:
1. Check world file paths:
   ```bash
   ls ~/ros2_ws/src/kevin_nav2_experiments/worlds/
   ```
2. Verify Gazebo resource path:
   ```bash
   echo $IGN_GAZEBO_RESOURCE_PATH
   ```
3. Reinstall Gazebo:
   ```bash
   sudo apt remove --purge ignition-fortress
   sudo apt install ignition-fortress
   ```

### Issue: "No module named 'rclpy'"
**Symptoms**: Python import errors for ROS 2 libraries

**Solutions**:
1. Verify ROS 2 environment is sourced:
   ```bash
   echo $ROS_DISTRO  # Should output: humble
   ```
2. Re-source workspace:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ```
3. Check Python path:
   ```bash
   python3 -c "import rclpy; print(rclpy.__file__)"
   ```

### Issue: Nav2 Nodes Fail to Start
**Symptoms**: "Could not load controller" or parameter errors

**Solutions**:
1. Verify controller configuration files exist:
   ```bash
   ls ~/ros2_ws/src/kevin_nav2_experiments/configs/
   ```
2. Check parameter file syntax (YAML formatting):
   ```bash
   python3 -c "import yaml; yaml.safe_load(open('configs/nav2_rpp_fair.yaml'))"
   ```
3. Ensure Nav2 packages are installed:
   ```bash
   ros2 pkg list | grep nav2_regulated_pure_pursuit
   ```

### Issue: Rosbag2 Recording Fails
**Symptoms**: No .db3 files created, or empty files

**Solutions**:
1. Check disk space:
   ```bash
   df -h ~
   ```
2. Verify results directory has write permissions:
   ```bash
   ls -ld ~/ros2_ws/test_results
   ```
3. Check rosbag2 storage backend:
   ```bash
   ros2 bag info --storage sqlite3
   ```

### Issue: AMCL Localization Diverges
**Symptoms**: Robot position jumps, tracking errors are huge

**Solutions**:
1. Publish initial pose (if not automatically set):
   ```bash
   ros2 topic pub /a200_0000/initialpose geometry_msgs/PoseWithCovarianceStamped \
     '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}}}}'
   ```
2. Check map files are loaded correctly
3. Verify LiDAR is publishing scan data:
   ```bash
   ros2 topic hz /a200_0000/sensors/lidar2d_0/scan
   ```

### Issue: Low Simulation Frame Rate
**Symptoms**: Gazebo runs slowly (<30 FPS), jerky motion

**Solutions**:
1. Reduce Gazebo real-time factor:
   ```bash
   # In launch file, set max_step_size="0.01" and real_time_factor="1.0"
   ```
2. Disable unnecessary Gazebo sensors/plugins
3. Close other applications to free RAM
4. Use software rendering if GPU is unavailable:
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   ```

---

## Uninstallation

### Complete Removal

#### Remove Workspace
```bash
rm -rf ~/ros2_ws
```

#### Remove ROS 2
```bash
sudo apt remove --purge ros-humble-*
sudo apt autoremove
```

#### Remove Gazebo
```bash
sudo apt remove --purge ignition-*
sudo apt autoremove
```

#### Remove Configuration Files
```bash
rm -rf ~/.ignition
rm -rf ~/.ros
```

#### Clean APT Cache
```bash
sudo apt clean
```

---

## Additional Resources

### Official Documentation
- **ROS 2 Humble**: https://docs.ros.org/en/humble/
- **Navigation2**: https://docs.nav2.org/
- **Gazebo Ignition Fortress**: https://gazebosim.org/docs/fortress

### Community Support
- **ROS 2 Discourse**: https://discourse.ros.org/
- **Nav2 GitHub Discussions**: https://github.com/ros-planning/navigation2/discussions
- **Gazebo Community**: https://community.gazebosim.org/

### Tutorials
- ROS 2 Tutorials: https://docs.ros.org/en/humble/Tutorials.html
- Nav2 Tutorials: https://docs.nav2.org/tutorials/index.html
- Gazebo Tutorials: https://gazebosim.org/docs/fortress/tutorials

---

## Next Steps

After successful installation:

1. **Read Experiment Guide**: See `EXPERIMENT_GUIDE.md` for running benchmarks
2. **Review Metrics**: See `DATA_DICTIONARY.md` for understanding output metrics
3. **Download Dataset**: Visit FigShare (https://doi.org/10.6084/m9.figshare.31150735) for reference data
4. **Join Community**: Star the GitHub repository and contribute improvements!

---

**Prepared by**: Claudio Urrea, Kevin Valencia-Aragón  
**Last Updated**: January 26, 2026  
**Tested on**: Ubuntu 22.04 LTS with ROS 2 Humble (Humble Hawksbill)
