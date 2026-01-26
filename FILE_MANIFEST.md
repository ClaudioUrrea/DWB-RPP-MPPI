# File Manifest - FigShare Dataset

## Dataset Information
- **Title**: Comprehensive Navigation Benchmark Dataset: DWB, RPP, and MPPI Controllers
- **DOI**: https://doi.org/10.6084/m9.figshare.31150735
- **Version**: 1.0
- **Date**: 2026-01-26
- **Authors**: Claudio Urrea, Kevin Valencia-Aragón

## File Organization

This manifest documents all files included in the FigShare dataset deposit, their organization, sizes, formats, and checksums (SHA-256) for integrity verification.

---

## Directory Structure Summary

```
figshare_dataset/
├── README.txt                          # Plain text dataset overview
├── LICENSE.txt                         # MIT License full text
├── CITATION.cff                        # Citation File Format metadata
├── rosbags/                            # Raw simulation data (270 files)
│   ├── scene1/ (90 files, ~15 GB)
│   ├── scene2/ (90 files, ~15 GB)
│   └── scene3/ (90 files, ~15 GB)
├── metrics/                            # Processed metrics (12 files)
│   ├── CSV format (human-readable)
│   └── Parquet format (optimized)
├── statistics/                         # Statistical analysis (6 files)
├── visualizations/                     # Plots and figures (20+ files)
├── metadata/                           # Dataset metadata and configs
│   ├── dataset_description.json
│   ├── file_manifest.csv (this file in CSV format)
│   ├── data_dictionary.json
│   ├── controller_configs/
│   └── world_files/
└── documentation/                      # Guides and tutorials
    ├── DATA_DICTIONARY.md
    ├── REPLICATION_GUIDE.md
    └── ANALYSIS_TUTORIAL.ipynb
```

---

## File Listing with Metadata

### Root Level Files

| File | Size | Format | Description | SHA-256 Checksum |
|------|------|--------|-------------|------------------|
| README.txt | 8.2 KB | Plain text | Dataset overview and quick start guide | [To be computed] |
| LICENSE.txt | 1.1 KB | Plain text | MIT License full text | [To be computed] |
| CITATION.cff | 2.3 KB | YAML (CFF) | Machine-readable citation metadata | [To be computed] |

---

### Rosbags Directory (270 files)

#### Scene 1: Structured Corridor (90 files, ~15 GB)

**DWB Controller (30 files, ~5 GB)**

| File | Size | Description | SHA-256 |
|------|------|-------------|---------|
| dwb_forward_001_1704067200.db3 | 55.3 MB | DWB forward trial 1 | [To be computed] |
| dwb_forward_002_1704067350.db3 | 54.8 MB | DWB forward trial 2 | [To be computed] |
| ... (13 more forward files) | ~55 MB each | DWB forward trials 3-15 | [Individual checksums] |
| dwb_return_001_1704067500.db3 | 56.1 MB | DWB return trial 1 | [To be computed] |
| dwb_return_002_1704067650.db3 | 55.7 MB | DWB return trial 2 | [To be computed] |
| ... (13 more return files) | ~55 MB each | DWB return trials 3-15 | [Individual checksums] |

**RPP Controller (30 files, ~5 GB)**

| File | Size | Description | SHA-256 |
|------|------|-------------|---------|
| rpp_forward_001_1704067800.db3 | 52.4 MB | RPP forward trial 1 | [To be computed] |
| rpp_forward_002_1704067950.db3 | 51.9 MB | RPP forward trial 2 | [To be computed] |
| ... (13 more forward files) | ~52 MB each | RPP forward trials 3-15 | [Individual checksums] |
| rpp_return_001_1704068100.db3 | 53.2 MB | RPP return trial 1 | [To be computed] |
| rpp_return_002_1704068250.db3 | 52.8 MB | RPP return trial 2 | [To be computed] |
| ... (13 more return files) | ~52 MB each | RPP return trials 3-15 | [Individual checksums] |

**MPPI Controller (30 files, ~5 GB)**

| File | Size | Description | SHA-256 |
|------|------|-------------|---------|
| mppi_forward_001_1704068400.db3 | 58.7 MB | MPPI forward trial 1 | [To be computed] |
| mppi_forward_002_1704068550.db3 | 59.2 MB | MPPI forward trial 2 | [To be computed] |
| ... (13 more forward files) | ~58 MB each | MPPI forward trials 3-15 | [Individual checksums] |
| mppi_return_001_1704068700.db3 | 57.9 MB | MPPI return trial 1 | [To be computed] |
| mppi_return_002_1704068850.db3 | 58.4 MB | MPPI return trial 2 | [To be computed] |
| ... (13 more return files) | ~58 MB each | MPPI return trials 3-15 | [Individual checksums] |

#### Scene 2: Semi-Structured Slopes (90 files, ~15 GB)

**Structure**: Identical to Scene 1 (DWB/RPP/MPPI, forward/return, 15 trials each)

**File Naming**: `{controller}_{direction}_{trial}_scene2_{timestamp}.db3`

**Average File Sizes**:
- DWB: 56-58 MB per file
- RPP: 53-55 MB per file
- MPPI: 60-62 MB per file

#### Scene 3: Narrow Corridor (90 files, ~15 GB)

**Structure**: Identical to Scene 1 (DWB/RPP/MPPI, forward/return, 15 trials each)

**File Naming**: `{controller}_{direction}_{trial}_scene3_{timestamp}.db3`

**Average File Sizes**:
- DWB: 54-56 MB per file
- RPP: 51-53 MB per file
- MPPI: 57-59 MB per file

---

### Metrics Directory (12 files)

| File | Size | Format | Rows × Columns | Description | SHA-256 |
|------|------|--------|----------------|-------------|---------|
| metrics_scene1_full.csv | 125 KB | CSV | 90 × 45 | All Scene 1 trials | [To be computed] |
| metrics_scene1_full.parquet | 28 KB | Parquet | 90 × 45 | Scene 1 (compressed) | [To be computed] |
| metrics_scene2_full.csv | 127 KB | CSV | 90 × 45 | All Scene 2 trials | [To be computed] |
| metrics_scene2_full.parquet | 29 KB | Parquet | 90 × 45 | Scene 2 (compressed) | [To be computed] |
| metrics_scene3_full.csv | 124 KB | CSV | 90 × 45 | All Scene 3 trials | [To be computed] |
| metrics_scene3_full.parquet | 28 KB | Parquet | 90 × 45 | Scene 3 (compressed) | [To be computed] |
| metrics_combined_all_scenes.csv | 375 KB | CSV | 270 × 45 | All trials combined | [To be computed] |
| metrics_combined_all_scenes.parquet | 82 KB | Parquet | 270 × 45 | Combined (compressed) | [To be computed] |
| summary_by_controller.csv | 2.8 KB | CSV | 3 × 30 | Aggregated by controller | [To be computed] |
| summary_by_controller_direction.csv | 5.4 KB | CSV | 6 × 30 | By controller + direction | [To be computed] |
| summary_by_controller_scene.csv | 8.1 KB | CSV | 9 × 30 | By controller + scene | [To be computed] |
| metrics_column_descriptions.json | 18.5 KB | JSON | - | Metadata for all 45 metrics | [To be computed] |

**CSV Columns (45 total)**:
- Metadata: bag_name, controller, direction, trial, scene
- Temporal: time_to_goal_s, t0_unix, t_arr_unix, stopped_time_s
- Spatial: global_plan_len_m, actual_path_len_m, efficiency_pos_pct, goal_dist_final_m
- Tracking: tracking_rmse_m, tracking_max_m, tracking_p95_m, heading_rmse_rad, tracking_integral_m2
- Safety: min_clearance_m, clearance_below_safe_pct, costmap_high_cost_pct, costmap_avg_cost, costmap_max_cost
- Control: cmd_effort_au, cmd_lin_L1, cmd_lin_L2, cmd_ang_L1, cmd_ang_L2, acc_lin_L1, acc_lin_L2
- Smoothness: heading_oscillations_count, cmd_jerk_lin, cmd_jerk_ang
- Velocity: vel_frac_stopped_pct, vel_frac_slow_pct, vel_frac_fast_pct, vel_mean_mps, vel_max_mps, vel_p95_mps
- Additional: (see DATA_DICTIONARY.md for complete list)

---

### Statistics Directory (6 files)

| File | Size | Format | Description | SHA-256 |
|------|------|--------|-------------|---------|
| anova_results_scene1.csv | 3.2 KB | CSV | Scheirer-Ray-Hare test results for Scene 1 | [To be computed] |
| anova_results_scene2.csv | 3.3 KB | CSV | Scheirer-Ray-Hare test results for Scene 2 | [To be computed] |
| anova_results_scene3.csv | 3.1 KB | CSV | Scheirer-Ray-Hare test results for Scene 3 | [To be computed] |
| pairwise_comparisons_scene1.csv | 8.7 KB | CSV | Mann-Whitney U tests for Scene 1 | [To be computed] |
| pairwise_comparisons_scene2.csv | 8.9 KB | CSV | Mann-Whitney U tests for Scene 2 | [To be computed] |
| pairwise_comparisons_scene3.csv | 8.6 KB | CSV | Mann-Whitney U tests for Scene 3 | [To be computed] |

**ANOVA Results Columns**:
- metric: Performance indicator name
- effect: Controller, Direction, or Controller×Direction
- H: Scheirer-Ray-Hare H statistic
- p_value: Statistical significance (p < 0.05 indicates significance)
- epsilon_squared: Effect size (0.01=small, 0.06=medium, 0.14=large)

**Pairwise Comparison Columns**:
- metric: Performance indicator name
- controller_A: First controller
- controller_B: Second controller
- direction: Forward or return
- U_statistic: Mann-Whitney U statistic
- p_value: Two-tailed p-value
- median_A: Median value for controller A
- median_B: Median value for controller B
- effect_size: Rank-biserial correlation

---

### Visualizations Directory (20+ files)

#### Trajectory Plots (9 files, ~15 MB total)

| File | Size | Format | Description | SHA-256 |
|------|------|--------|-------------|---------|
| trajectories_scene1_dwb.png | 1.8 MB | PNG | Scene 1 trajectories - DWB | [To be computed] |
| trajectories_scene1_rpp.png | 1.7 MB | PNG | Scene 1 trajectories - RPP | [To be computed] |
| trajectories_scene1_mppi.png | 1.9 MB | PNG | Scene 1 trajectories - MPPI | [To be computed] |
| trajectories_scene2_dwb.png | 2.1 MB | PNG | Scene 2 trajectories - DWB | [To be computed] |
| trajectories_scene2_rpp.png | 2.0 MB | PNG | Scene 2 trajectories - RPP | [To be computed] |
| trajectories_scene2_mppi.png | 2.2 MB | PNG | Scene 2 trajectories - MPPI | [To be computed] |
| trajectories_scene3_dwb.png | 1.6 MB | PNG | Scene 3 trajectories - DWB | [To be computed] |
| trajectories_scene3_rpp.png | 1.5 MB | PNG | Scene 3 trajectories - RPP | [To be computed] |
| trajectories_scene3_mppi.png | 1.7 MB | PNG | Scene 3 trajectories - MPPI | [To be computed] |

**Resolution**: 1920×1080 pixels (300 DPI)  
**Content**: Robot trajectories overlaid on costmap with start/goal markers

#### Comparative Box Plots (12 files, ~8 MB total)

| File | Size | Description | SHA-256 |
|------|------|-------------|---------|
| boxplot_time_to_goal_by_scene.png | 650 KB | Time-to-goal comparison | [To be computed] |
| boxplot_tracking_rmse_by_scene.png | 680 KB | Lateral tracking error | [To be computed] |
| boxplot_control_effort_by_scene.png | 670 KB | Control effort comparison | [To be computed] |
| boxplot_oscillations_by_scene.png | 640 KB | Heading oscillations | [To be computed] |
| boxplot_clearance_by_scene.png | 660 KB | Minimum clearance | [To be computed] |
| boxplot_efficiency_by_scene.png | 655 KB | Path efficiency | [To be computed] |
| boxplot_stopped_time_by_scene.png | 645 KB | Stopped time comparison | [To be computed] |
| heatmap_correlation_scene1.png | 780 KB | Metric correlations - Scene 1 | [To be computed] |
| heatmap_correlation_scene2.png | 785 KB | Metric correlations - Scene 2 | [To be computed] |
| heatmap_correlation_scene3.png | 775 KB | Metric correlations - Scene 3 | [To be computed] |
| radar_chart_controller_profiles.png | 720 KB | Multi-metric controller comparison | [To be computed] |
| scatter_efficiency_vs_time.png | 690 KB | Efficiency-speed tradeoff | [To be computed] |

**Resolution**: 1600×1200 pixels (300 DPI)  
**Format**: PNG with transparent background where applicable

---

### Metadata Directory

#### Main Metadata Files

| File | Size | Format | Description | SHA-256 |
|------|------|--------|-------------|---------|
| dataset_description.json | 12.4 KB | JSON | Complete dataset metadata (schema.org) | [To be computed] |
| file_manifest.csv | 45.7 KB | CSV | This manifest in CSV format | [To be computed] |
| data_dictionary.json | 38.2 KB | JSON | Machine-readable metric definitions | [To be computed] |

#### Controller Configuration Files (3 files)

| File | Size | Format | Description | SHA-256 |
|------|------|--------|-------------|---------|
| nav2_dwb_fair.yaml | 8.5 KB | YAML | DWB controller parameters | [To be computed] |
| nav2_rpp_fair.yaml | 8.3 KB | YAML | RPP controller parameters | [To be computed] |
| nav2_mppi_fair.yaml | 10.1 KB | YAML | MPPI controller parameters | [To be computed] |

#### World Files (7 files)

| File | Size | Format | Description | SHA-256 |
|------|------|--------|-------------|---------|
| warehouse.sdf | 7.5 KB | XML (SDF) | Scene 1 Gazebo world | [To be computed] |
| warehouse.yaml | 512 B | YAML | Scene 1 map configuration | [To be computed] |
| cave_circuit_02_subset_r75_ignfix.sdf | 2.0 KB | XML (SDF) | Scene 2 Gazebo world | [To be computed] |
| map_name.pgm | 1.5 MB | PGM | Scene 2 occupancy grid | [To be computed] |
| map_name.yaml | 512 B | YAML | Scene 2 map metadata | [To be computed] |
| final_event_07.sdf | 4.5 KB | XML (SDF) | Scene 3 Gazebo world | [To be computed] |
| map5.yaml | 512 B | YAML | Scene 3 map configuration | [To be computed] |

---

### Documentation Directory (3 files)

| File | Size | Format | Description | SHA-256 |
|------|------|--------|-------------|---------|
| DATA_DICTIONARY.md | 67.8 KB | Markdown | Complete metric definitions and computation methods | [To be computed] |
| REPLICATION_GUIDE.md | 42.3 KB | Markdown | Step-by-step experiment replication | [To be computed] |
| ANALYSIS_TUTORIAL.ipynb | 156 KB | Jupyter Notebook | Interactive data analysis tutorial | [To be computed] |

**ANALYSIS_TUTORIAL.ipynb contents**:
- Loading CSV/Parquet data into pandas
- Basic statistical summaries
- Visualization examples (matplotlib, seaborn)
- Hypothesis testing workflow
- Custom metric computation

---

## Checksum Verification

### Generating Checksums

To verify file integrity, compute SHA-256 checksums:

**Linux/macOS**:
```bash
cd figshare_dataset
find . -type f -exec sha256sum {} \; > checksums.txt
```

**Python** (cross-platform):
```python
import hashlib
from pathlib import Path

def compute_sha256(filepath):
    sha256 = hashlib.sha256()
    with open(filepath, 'rb') as f:
        for chunk in iter(lambda: f.read(4096), b''):
            sha256.update(chunk)
    return sha256.hexdigest()

for file in Path('figshare_dataset').rglob('*'):
    if file.is_file():
        print(f"{compute_sha256(file)}  {file}")
```

### Verifying Downloaded Dataset

After downloading from FigShare, verify integrity:

```bash
# Compare checksums
sha256sum -c checksums.txt

# Expected output: All files should show "OK"
```

If any checksums mismatch, re-download affected files.

---

## File Format Specifications

### Rosbag2 SQLite3 (.db3)

**Format**: SQLite3 database (version 3)  
**Schema**: ROS 2 rosbag2 standard schema  
**Encoding**: Binary (ROS 2 CDR serialization)  
**Compression**: None (raw messages)

**Tables**:
- `messages`: Serialized ROS 2 messages
- `topics`: Topic metadata (name, type, serialization)
- `metadata`: Bag creation time, duration, message count

**Access Methods**:
- ROS 2 CLI: `ros2 bag info`, `ros2 bag play`
- Python: `rosbag2_py` library
- SQL: Direct SQLite queries (advanced)

### CSV Files

**Format**: Comma-separated values (RFC 4180)  
**Encoding**: UTF-8  
**Line Endings**: Unix (LF, \n)  
**Decimal Separator**: Period (.)  
**Null Values**: Empty string or "NaN"  
**Header**: First row contains column names  

### Parquet Files

**Format**: Apache Parquet (version 2.0)  
**Compression**: Snappy  
**Schema**: Embedded (self-describing)  
**Null Representation**: Parquet null type  

**Advantages**:
- 70-80% smaller than CSV
- 10-20× faster loading
- Preserves data types

### PNG Images

**Format**: Portable Network Graphics  
**Color Space**: sRGB  
**Bit Depth**: 8 bits per channel  
**Compression**: Lossless (PNG compression level 9)  
**Metadata**: Embedded creation date, software version

### JSON Files

**Format**: JavaScript Object Notation (ECMA-404)  
**Encoding**: UTF-8  
**Formatting**: Pretty-printed (4-space indentation)  
**Schema**: Documented in dataset_description.json

### YAML Files

**Format**: YAML 1.2  
**Encoding**: UTF-8  
**Indentation**: 2 spaces  
**Usage**: ROS 2 parameter files, map configurations

---

## Dataset Statistics Summary

| Category | Count | Total Size | Average Size |
|----------|-------|------------|--------------|
| Rosbag2 files | 270 | ~45 GB | ~170 MB |
| Metrics files (CSV) | 8 | 785 KB | 98 KB |
| Metrics files (Parquet) | 4 | 167 KB | 42 KB |
| Statistics files | 6 | 36 KB | 6 KB |
| Visualization files | 21 | ~23 MB | ~1.1 MB |
| Metadata files | 13 | ~4 MB | ~310 KB |
| Documentation files | 3 | 266 KB | 89 KB |
| **Total** | **325** | **~50 GB** | **~154 MB** |

---

## Version Control

This file manifest corresponds to dataset version 1.0 (initial release).

Future versions will maintain backward compatibility and document changes in a `CHANGELOG.md` file.

---

## Contact for Data Issues

If you encounter issues with file integrity, missing files, or corrupted downloads:

1. **Check FigShare Status**: https://status.figshare.com/
2. **Report Issue**: 
   - Email: claudio.urrea@usach.cl
   - Subject: "FigShare Dataset Issue - [describe problem]"
   - Include: Affected filename, checksum mismatch details, download timestamp
3. **GitHub Issues**: https://github.com/ClaudioUrrea/DWB-RPP-MPPI/issues

---

**Manifest Version**: 1.0  
**Generated**: 2026-01-26  
**Authors**: Claudio Urrea, Kevin Valencia-Aragón  
**DOI**: https://doi.org/10.6084/m9.figshare.31150735
