# Theoretical Analysis and Systematic Comparison of Local Navigation Control Strategies in Semi-Structured Environments

[![DOI](https://img.shields.io/badge/DOI-10.6084%2Fm9.figshare.31150735-blue)](https://doi.org/10.6084/m9.figshare.31150735)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)

## Overview

This repository contains the complete source code, simulation configurations, and experimental infrastructure supporting the research paper **"Theoretical Analysis and Systematic Comparison of Local Navigation Control Strategies in Semi-Structured Environments: A Systems Approach"** published in *Systems* (MDPI).

The benchmark systematically evaluates three local navigation controllers from the ROS 2 Navigation2 (Nav2) stack:
- **DWB** (Dynamic Window Approach-based controller)
- **RPP** (Regulated Pure Pursuit)
- **MPPI** (Model Predictive Path Integral control)

across three distinct semi-structured scenarios designed to stress different aspects of local navigation performance.

### Key Features

- **Reproducible Benchmarking Framework**: Fully automated experiment orchestration using ROS 2 Humble and Gazebo Ignition Fortress
- **Comprehensive Metrics Suite**: 30+ performance indicators including tracking accuracy, execution time, control effort, oscillations, and costmap-based safety metrics
- **Fair Comparison Protocol**: Identical upstream perception, localization, and global planning across all controllers
- **Open Data Compliance**: Complete datasets (raw trajectories, rosbag2 recordings, processed metrics) available on FigShare in FAIR-compliant formats

## Citation

If you use this code or dataset in your research, please cite:

```bibtex
@article{urrea2026theoretical,
  title={Theoretical Analysis and Systematic Comparison of Local Navigation Control Strategies in Semi-Structured Environments: A Systems Approach},
  author={Urrea, Claudio and Valencia-Arag{\'o}n, Kevin},
  journal={Systems},
  volume={XX},
  number={X},
  pages={XXX},
  year={2026},
  publisher={MDPI},
  doi={XXX}
}
```

## Repository Structure

```
.
├── README.md                          # This file
├── LICENSE                            # MIT License
├── INSTALL.md                         # Detailed installation instructions
├── EXPERIMENT_GUIDE.md                # Step-by-step experiment execution guide
├── DATA_DICTIONARY.md                 # Complete description of all metrics and data fields
├── scripts/
│   ├── analyze_bags_extended.py      # Main rosbag2 analysis pipeline (885 lines)
│   ├── auto_experiment_runner.py     # Automated experiment orchestration
│   ├── go_to_goal.py                 # Navigation task script (S→G leg)
│   └── return_to_start.py            # Navigation task script (G→S leg)
├── configs/
│   ├── nav2_dwb_fair.yaml            # DWB controller configuration
│   ├── nav2_rpp_fair.yaml            # RPP controller configuration
│   └── nav2_mppi_fair.yaml           # MPPI controller configuration
├── worlds/
│   ├── warehouse.sdf                 # Scene 1: Structured corridor with dynamic obstacle
│   ├── warehouse.yaml                # Scene 1: Map configuration
│   ├── cave_circuit_02_subset_r75_ignfix.sdf  # Scene 2: Semi-structured slopes
│   ├── map_name.pgm                  # Scene 2: Occupancy grid map
│   ├── map_name.yaml                 # Scene 2: Map metadata
│   ├── final_event_07.sdf            # Scene 3: Narrow corridor
│   └── map5.yaml                     # Scene 3: Map configuration
├── figures/
│   ├── scene1_goal.png               # Scene 1 visualization
│   ├── scene2_goal.png               # Scene 2 visualization
│   ├── scene3_goal.png               # Scene 3 visualization
│   ├── FIgure2.pdf                   # Experimental methodology flowchart
│   ├── Figure3.pdf                   # Controller performance comparison
│   ├── fig_results_osc_effort.pdf    # Oscillations and control effort analysis
│   └── Figure_5.pdf                  # Additional results visualization
├── requirements.txt                   # Python dependencies
└── docs/
    ├── METHODOLOGY.md                # Detailed methodology description
    ├── METRICS_REFERENCE.md          # Complete metrics computation reference
    └── TROUBLESHOOTING.md            # Common issues and solutions
```

## Quick Start

### Prerequisites

- **Operating System**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill (full desktop installation)
- **Gazebo**: Ignition Fortress
- **Python**: 3.10 or higher
- **Disk Space**: ~10 GB for simulation environments and dependencies
- **RAM**: Minimum 8 GB (16 GB recommended for MPPI)

### Installation

1. **Clone the repository**:
```bash
git clone https://github.com/ClaudioUrrea/DWB-RPP-MPPI.git
cd DWB-RPP-MPPI
```

2. **Install ROS 2 Humble** (if not already installed):
```bash
# Follow official ROS 2 installation guide:
# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
```

3. **Install Python dependencies**:
```bash
pip install -r requirements.txt
```

4. **Build the workspace**:
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

For detailed installation instructions including system configuration, Nav2 setup, and troubleshooting, see [INSTALL.md](INSTALL.md).

### Running a Single Experiment

1. **Launch simulation and Nav2** (separate terminal):
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch kevin_nav2_experiments nav2_local_only.launch.py \
    namespace:=a200_0000 \
    use_sim_time:=true \
    params_file:=/path/to/nav2_rpp_fair.yaml
```

2. **Execute navigation task** (separate terminal):
```bash
source ~/ros2_ws/install/setup.bash
python3 scripts/go_to_goal.py \
    --controller rpp \
    --trial 1 \
    --results-dir ./results \
    --goal 3.5 2.0 0.0
```

3. **Analyze recorded data**:
```bash
python3 scripts/analyze_bags_extended.py \
    --results-dir ./results \
    --out-csv ./metrics_full.csv \
    --out-summary ./summary_full.csv \
    --controllers RPP
```

### Running Full Benchmark Suite

For automated execution of all 270 trials (3 controllers × 3 scenes × 2 directions × 15 repetitions):

```bash
python3 scripts/auto_experiment_runner.py \
    --trials 15 \
    --controllers dwb rpp mppi \
    --results-dir ./results \
    --goal 3.5 2.0 0.0
```

See [EXPERIMENT_GUIDE.md](EXPERIMENT_GUIDE.md) for complete experimental protocol details.

## Experimental Methodology

### Evaluation Scenarios

1. **Scene 1 - Structured Corridor**: Tests dynamic obstacle avoidance with transient costmap inflation
2. **Scene 2 - Semi-Structured Slopes**: Challenges perception-limited navigation on uneven terrain (±15° slopes)
3. **Scene 3 - Narrow Corridor**: Evaluates tight-space navigation with minimal clearance margins

Each scene is evaluated bidirectionally (Start→Goal and Goal→Start) with 15 independent repetitions per controller-direction pair.

### Performance Metrics (30+ indicators)

#### Navigation Efficiency
- **Time to Goal** (s): Total execution time from goal publication to arrival
- **Global Plan Length** (m): Reference path geometric length
- **Actual Path Length** (m): Robot's executed trajectory length
- **Position Efficiency** (%): Ratio of plan length to actual path length

#### Tracking Accuracy
- **Lateral RMSE** (m): Root mean square perpendicular deviation from global plan
- **Heading RMSE** (rad): Angular deviation from plan orientation
- **Max Lateral Error** (m): Worst-case deviation from reference path

#### Safety & Smoothness
- **Minimum Clearance** (m): Closest approach to obstacles
- **High-Cost Exposure** (% of time): Fraction of trajectory in hazardous costmap regions (cost > 200)
- **Stopped Time** (s): Total time with velocity < 0.03 m/s
- **Heading Oscillations** (#): Count of angular velocity sign changes

#### Control Effort
- **Command Effort** (a.u.): Integrated squared linear and angular velocity commands
- **Linear Command L1/L2**: Intensity and smoothness proxies for translation
- **Angular Command L1/L2**: Intensity and smoothness proxies for rotation
- **Acceleration L1/L2**: Jerk-related smoothness indicators

See [DATA_DICTIONARY.md](DATA_DICTIONARY.md) for complete metric definitions and computation methods.

## Data Availability

### Raw Simulation Data (FigShare)
Complete experimental dataset including:
- **270 rosbag2 recordings** (SQLite3 .db3 format) with ROS 2 topics:
  - Odometry (`/platform/odom/filtered`)
  - AMCL localization (`/amcl_pose`)
  - Global plans (`/plan`, `/received_global_plan`)
  - Velocity commands (`/cmd_vel`)
  - LiDAR scans (`/sensors/lidar2d_0/scan`)
  - Local costmaps (`/local_costmap/costmap`)
  - Simulation clock (`/clock`)
- **Processed metrics CSV** (270 rows × 45 columns)
- **Summary statistics** (aggregated by controller and direction)
- **Trajectory visualizations** (PNG format)

**Access**: [https://doi.org/10.6084/m9.figshare.31150735](https://doi.org/10.6084/m9.figshare.31150735)

**Formats**: 
- CSV (human-readable, Excel/LibreOffice compatible)
- Parquet (optimized for Python/R data analysis pipelines)

All datasets include comprehensive metadata complying with FAIR principles (Findable, Accessible, Interoperable, Reusable).

### Data Organization

```
figshare_dataset/
├── rosbags/
│   ├── scene1/
│   │   ├── dwb_forward_001_timestamp.db3
│   │   ├── dwb_return_001_timestamp.db3
│   │   ├── ... (90 files total)
│   ├── scene2/ (90 files)
│   └── scene3/ (90 files)
├── metrics/
│   ├── metrics_scene1_full.csv
│   ├── metrics_scene2_full.csv
│   ├── metrics_scene3_full.csv
│   ├── summary_by_controller.csv
│   └── summary_by_controller_direction.csv
├── visualizations/
│   ├── trajectories_scene1.png
│   ├── trajectories_scene2.png
│   ├── trajectories_scene3.png
│   ├── boxplots_tracking_rmse.png
│   ├── boxplots_time_to_goal.png
│   └── ... (additional analysis figures)
└── metadata/
    ├── dataset_description.json
    ├── file_manifest.csv
    └── data_dictionary.json
```

## Key Results Summary

### Controller Performance Profiles

| Controller | Strengths | Optimal Use Cases |
|------------|-----------|-------------------|
| **RPP** | Fastest time-to-goal, smooth commands | Uncluttered environments, speed-prioritized missions |
| **DWB** | Superior angular stability, consistent tracking | Scenes requiring precise path following |
| **MPPI** | Lowest control effort, minimal oscillations | Long missions prioritizing energy efficiency |

### Scene-Specific Findings

- **Scene 1** (Dynamic Obstacles): RPP achieves 15-20% faster traversal than DWB/MPPI
- **Scene 2** (Slopes): All controllers exhibit similar tracking RMSE (~0.12-0.15 m), indicating **perception-limited regime**
- **Scene 3** (Narrow Corridors): DWB shows 2.5× lower heading oscillations than MPPI

**Critical Insight**: In sloped terrain (Scene 2), lateral tracking errors compress across all controllers, revealing that **2D LiDAR-induced costmap bias dominates** over controller-specific logic. This motivates future work on elevation-aware perception (2.5D mapping, IMU fusion, ground segmentation).

## Reproducing Published Results

1. **Download the FigShare dataset**:
```bash
wget https://figshare.com/ndownloader/files/XXXXXX -O dataset.zip
unzip dataset.zip -d ./figshare_data
```

2. **Run analysis pipeline on downloaded data**:
```bash
python3 scripts/analyze_bags_extended.py \
    --results-dir ./figshare_data/rosbags/scene1 \
    --out-csv ./reproduced_metrics_scene1.csv \
    --out-summary ./reproduced_summary_scene1.csv \
    --controllers DWB RPP MPPI
```

3. **Verify against published metrics**:
```bash
# Compare your reproduced results with published summary
diff ./reproduced_summary_scene1.csv ./figshare_data/metrics/summary_by_controller.csv
```

Expected difference: Machine precision only (< 1e-6 for floating-point metrics).

## Extending the Benchmark

### Adding New Controllers

1. Create YAML configuration in `configs/`:
```yaml
# configs/nav2_my_controller.yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "my_controller_namespace::MyController"
      # ... controller-specific parameters
```

2. Update `auto_experiment_runner.py`:
```python
CONTROLLERS = {
    "dwb": "/path/to/nav2_dwb_fair.yaml",
    "rpp": "/path/to/nav2_rpp_fair.yaml",
    "mppi": "/path/to/nav2_mppi_fair.yaml",
    "my_controller": "/path/to/nav2_my_controller.yaml",  # Add here
}
```

3. Run experiments:
```bash
python3 scripts/auto_experiment_runner.py \
    --trials 15 \
    --controllers my_controller \
    --results-dir ./results_new_controller
```

### Adding New Scenes

1. Create Gazebo world file (`.sdf`) in `worlds/`
2. Generate corresponding map files (`.pgm`, `.yaml`)
3. Update launch configuration to load new world
4. Run standard experimental protocol

See [EXTENDING_BENCHMARK.md](docs/EXTENDING_BENCHMARK.md) for detailed guidance.

## Technical Details

### Controller Configurations (Fair Comparison)

All controllers use **identical upstream perception and planning**:
- AMCL localization (500 particles, KLD sampling)
- NavFn global planner (A* search)
- 2D planar LiDAR (360°, 0.25° resolution, 30 Hz)
- Costmap parameters: 
  - Resolution: 0.05 m/cell
  - Inflation radius: 0.55 m
  - Robot footprint: 0.99 m × 0.67 m rectangle
  - Update frequency: 5 Hz

Controller-specific parameters were tuned for fair comparison:
- **DWB**: 13 critics enabled (path alignment, goal distance, obstacles, etc.)
- **RPP**: Adaptive lookahead (0.4-0.9 m), velocity scaling, inflation cost regulation
- **MPPI**: 2000 samples, 0.1s horizon, temperature 0.3, critics for path following and obstacles

See individual YAML files in `configs/` for complete parameter listings.

### Metrics Computation Pipeline

The analysis pipeline (`analyze_bags_extended.py`) implements:

1. **Temporal Synchronization**: 
   - Time origin (t₀) defined by goal publication
   - Arrival detection via radius threshold (0.5 m) with dwell time (0.3 s)
   - All signals cropped to [t₀, t_arrival]

2. **Spatial Alignment**:
   - SE(2) Procrustes registration between odometry and AMCL frames
   - Robust plan selection (longest plan in first 2.0s window)
   - Point-to-polyline projection for lateral tracking error

3. **Statistical Robustness**:
   - Median and IQR reported (robust to outliers)
   - Scheirer-Ray-Hare test (nonparametric 2-way ANOVA)
   - Mann-Whitney U for pairwise comparisons
   - Effect sizes (ε² for rank-based ANOVA)

See [METRICS_REFERENCE.md](docs/METRICS_REFERENCE.md) for mathematical formulations.

## Contributing

We welcome contributions! Areas of particular interest:

1. **Controller Implementations**: Additional Nav2 controllers (TEB, MPC variants)
2. **Perception Improvements**: 2.5D costmaps, elevation mapping integration
3. **New Scenarios**: Outdoor unstructured environments, multi-agent settings
4. **Analysis Tools**: Statistical visualization, parameter sensitivity analysis
5. **Documentation**: Translations, tutorial videos, Jupyter notebook demos

Please open an issue or pull request on GitHub. See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## License

This project is licensed under the MIT License - see [LICENSE](LICENSE) file for details.

Code portions adapted from ROS 2 Navigation2 stack (Apache 2.0 license) are clearly marked in source files.

## Authors

- **Claudio Urrea** - Principal Investigator - [ORCID: 0000-0001-7197-8928](https://orcid.org/0000-0001-7197-8928)
- **Kevin Valencia-Aragón** - Research Engineer - [ORCID: 0000-0002-3552-1709](https://orcid.org/0000-0002-3552-1709)

**Affiliation**: Faculty of Engineering, University of Santiago of Chile (USACH), Santiago, Chile

## Acknowledgments

This work was supported by the Faculty of Engineering of the University of Santiago of Chile (USACH). We thank the ROS 2 Navigation2 community and Gazebo development team for their open-source contributions.

## Contact

For questions about the code or dataset, please:
1. Open an issue on GitHub: [https://github.com/ClaudioUrrea/DWB-RPP-MPPI/issues](https://github.com/ClaudioUrrea/DWB-RPP-MPPI/issues)
2. Email: claudio.urrea@usach.cl (Claudio Urrea) or kevin.valencia@usach.cl (Kevin Valencia-Aragón)

## Related Publications

- Macenski, S., et al. "The Marathon 2: A Navigation System." IEEE/RSJ IROS 2020.
- Williams, G., et al. "Information-Theoretic Model Predictive Control." IEEE Transactions on Robotics 2018.
- Fox, D., Burgard, W., Thrun, S. "The Dynamic Window Approach to Collision Avoidance." IEEE RAM 1997.

---

**Last Updated**: January 2026  
**Repository**: [https://github.com/ClaudioUrrea/DWB-RPP-MPPI](https://github.com/ClaudioUrrea/DWB-RPP-MPPI)  
**Dataset DOI**: [https://doi.org/10.6084/m9.figshare.31150735](https://doi.org/10.6084/m9.figshare.31150735)  
**Paper DOI**: [To be assigned upon publication]
