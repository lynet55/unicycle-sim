# Implementation Summary: ROS Bag Recording & Plotting

## Changes Made

### 1. Added Reference Trajectory Publishing

**Modified Files:**
- `catkin_ws/src/turtlebot_traj_ctrl/include/controller/trajectory_controller.h`
  - Added `reference_publisher` member variable

- `catkin_ws/src/turtlebot_traj_ctrl/src/trajectory_controller.cpp`
  - Initialized reference publisher for topic `/turtlebot/reference`
  - Added code to publish reference trajectory (x_ref, y_ref) at each control step
  - Uses `TurtlebotState` message type for consistency

**New Topic:**
- `/turtlebot/reference` (type: `turtlebot_simulator/TurtlebotState`)
  - Publishes the desired reference trajectory at each timestep
  - Fields: `x` (x_ref), `y` (y_ref), `w` (unused, set to 0)

### 2. Automatic ROS Bag Recording

**Modified Files:**
- `catkin_ws/src/turtlebot_simulator/launch/simulator.launch`
  - Added `rosbag record` node to automatically record data during simulation
  - Records to: `/workspace/rosbags/simulation_*.bag` (auto-timestamped)

**Topics Recorded:**
- `/turtlebot/state` - Actual robot state (x, y, w)
- `/turtlebot/reference` - Reference trajectory (x_ref, y_ref)
- `/control_commands` - Control inputs (v_cmd, omega_cmd)

**Storage Location:**
- Inside container: `/workspace/rosbags/`
- Host machine: `./rosbags/` (in project root)
- Files are automatically timestamped

### 3. Python Plotting Script

**New File:**
- `plot_rosbag.py` - Comprehensive plotting script with multiple features

**Features:**
- Reads ROS bag files and extracts all relevant data
- Generates 4 publication-quality plots:
  1. **2D Trajectory Plot**: Scatter plot showing actual path (color-coded by time) overlaid with reference trajectory
  2. **X Position Tracking**: Time series of X and X_ref
  3. **Y Position Tracking**: Time series of Y and Y_ref
  4. **Control Commands**: Dual-axis plot of v_cmd and ω_cmd vs time

**Command-line Interface:**
```bash
# Plot most recent bag file
python3 plot_rosbag.py --latest

# Plot specific bag file
python3 plot_rosbag.py rosbags/simulation_YYYY-MM-DD-HH-MM-SS.bag

# Custom directory
python3 plot_rosbag.py --latest --dir /path/to/rosbags
```

**Output:**
- Interactive matplotlib window with all plots
- Saved PNG file with high resolution (300 DPI)
- Console output with tracking error statistics:
  - X/Y error: mean, std, max
  - Position error magnitude
  - Control command ranges

### 4. Documentation

**New Files:**
- `PLOTTING_GUIDE.md` - Comprehensive guide for using the plotting system
- `IMPLEMENTATION_SUMMARY.md` - This file

**Modified Files:**
- `README.md` - Added plotting section and reference to guide

**New Directory:**
- `rosbags/` - Storage location for recorded bag files

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     ROS System                               │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────┐    /control_commands    ┌──────────────┐ │
│  │              │◄────────────────────────│              │ │
│  │  Simulator   │                         │  Controller  │ │
│  │              │    /turtlebot/state     │              │ │
│  │              │────────────────────────►│              │ │
│  └──────────────┘                         └──────────────┘ │
│         │                                        │          │
│         │ /turtlebot/state                      │          │
│         │                    /turtlebot/reference│          │
│         │                                        │          │
│         └────────────┬───────────────────────────┘          │
│                      │                                      │
│                      ▼                                      │
│               ┌─────────────┐                              │
│               │   rosbag    │                              │
│               │  recorder   │                              │
│               └─────────────┘                              │
│                      │                                      │
└──────────────────────┼──────────────────────────────────────┘
                       │
                       ▼
                 ┌──────────┐
                 │ .bag file│
                 └──────────┘
                       │
                       ▼
            ┌────────────────────┐
            │  plot_rosbag.py    │
            └────────────────────┘
                       │
                       ▼
              ┌────────────────┐
              │  Plots + Stats │
              └────────────────┘
```

## Data Flow

1. **Controller** computes reference trajectory and publishes to `/turtlebot/reference`
2. **Controller** computes control commands and publishes to `/control_commands`
3. **Simulator** receives control commands and integrates dynamics
4. **Simulator** publishes actual state to `/turtlebot/state`
5. **rosbag recorder** captures all three topics automatically
6. **Plotting script** post-processes bag file and generates visualizations

## Next Steps to Use

1. **Rebuild workspace** (required after controller changes):
   ```bash
   docker exec ros-noetic-workspace bash -c "cd /workspace && source /opt/ros/noetic/setup.bash && catkin_make"
   ```

2. **Run simulation** (rosbag recording starts automatically):
   ```bash
   # Terminal 1
   docker exec ros-noetic-workspace bash -c "source /workspace/devel/setup.bash && roslaunch turtlebot_simulator simulator.launch"
   
   # Terminal 2
   docker exec ros-noetic-workspace bash -c "source /workspace/devel/setup.bash && roslaunch turtlebot_traj_ctrl trajectory_controller.launch"
   ```

3. **Let it run** for 10-20 seconds, then stop with Ctrl+C

4. **Plot results**:
   ```bash
   python3 plot_rosbag.py --latest
   ```

## Dependencies

The plotting script requires:
- `rosbag` (part of ROS)
- `matplotlib`
- `numpy`

Install with:
```bash
pip3 install matplotlib numpy rosbag
```

## Benefits

✅ **Automatic Recording**: No need to manually start rosbag  
✅ **Timestamped Files**: Each run gets a unique filename  
✅ **Complete Data**: All relevant topics captured  
✅ **Easy Visualization**: One command to generate all plots  
✅ **Quantitative Analysis**: Automatic error statistics  
✅ **Publication Ready**: High-resolution plots with proper labels  
✅ **Flexible**: Can plot any bag file, not just the latest  

## Troubleshooting

If plots don't show reference trajectory:
1. Rebuild the workspace (see step 1 above)
2. Check topic is published: `rostopic list | grep reference`
3. Check bag contains data: `rosbag info rosbags/your_file.bag`

If no bag files created:
1. Check rosbags directory exists
2. Check permissions on rosbags directory
3. Look for errors in simulator terminal output

