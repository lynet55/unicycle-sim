# ROS Bag Recording and Plotting Guide

## Overview
The system is now configured to automatically record rosbag data when you run the simulator. The plotting script allows you to visualize the results.

## What's Recorded
The rosbag automatically records three topics:
- `/turtlebot/state` - Actual robot state (x, y, w)
- `/turtlebot/reference` - Reference trajectory (x_ref, y_ref)
- `/control_commands` - Control inputs (v_cmd, omega_cmd)

## Usage

### 1. Run the Simulation (Automatic Recording)

**Terminal 1 - Start Simulator with Recording:**
```bash
docker exec ros-noetic-workspace bash -c "source /workspace/devel/setup.bash && roslaunch turtlebot_simulator simulator.launch"
```

**Terminal 2 - Start Trajectory Controller:**
```bash
docker exec ros-noetic-workspace bash -c "source /workspace/devel/setup.bash && roslaunch turtlebot_traj_ctrl trajectory_controller.launch"
```

The rosbag recording starts automatically with the simulator and saves files to `/workspace/rosbags/` (which maps to `./rosbags/` on your host machine).

### 2. Stop the Simulation
Press `Ctrl+C` in both terminals. The rosbag will be saved automatically.

### 3. Plot the Results

**Option A: Plot the most recent rosbag**
```bash
python3 plot_rosbag.py --latest
```

**Option B: Plot a specific rosbag file**
```bash
python3 plot_rosbag.py rosbags/simulation_2024-12-28-10-30-45.bag
```

**Option C: Specify custom rosbags directory**
```bash
python3 plot_rosbag.py --latest --dir /path/to/rosbags
```

### 4. View Results
The script will:
- Display 4 plots in a matplotlib window:
  1. **2D Trajectory** - Scatter plot of actual path (color-coded by time) overlaid with reference trajectory
  2. **X Position Tracking** - X and X_ref vs time
  3. **Y Position Tracking** - Y and Y_ref vs time
  4. **Control Commands** - v_cmd and ω_cmd vs time
- Save the plots as a PNG file (same name as bag file with `_plots.png` extension)
- Print tracking error statistics to the console

## Requirements

Install required Python packages (if not already installed):
```bash
pip3 install rosbag matplotlib numpy
```

## Directory Structure
```
mobile-robots/
├── rosbags/                      # Rosbag files stored here
│   ├── simulation_*.bag          # Recorded bag files
│   └── simulation_*_plots.png    # Generated plots
├── plot_rosbag.py                # Plotting script
└── PLOTTING_GUIDE.md             # This file
```

## Troubleshooting

### No bag files found
- Make sure you've run the simulation at least once
- Check that the `rosbags/` directory exists
- Verify the rosbag node is running: `rostopic echo /rosout | grep rosbag`

### Empty plots
- Ensure the simulation ran for sufficient time (at least a few seconds)
- Check that topics are being published: `rostopic list`
- Verify messages are being recorded: `rosbag info rosbags/your_bag_file.bag`

### Missing reference trajectory
- Make sure you rebuild the workspace after the controller changes:
  ```bash
  docker exec ros-noetic-workspace bash -c "cd /workspace && source /opt/ros/noetic/setup.bash && catkin_make"
  ```

## Tips

1. **Recording Duration**: Let the simulation run for at least 1-2 trajectory periods for meaningful plots
2. **File Management**: Rosbag files can be large. Clean up old files periodically
3. **Multiple Runs**: Each run creates a new timestamped bag file, so you can compare different experiments
4. **Customize Plots**: Edit `plot_rosbag.py` to adjust plot styles, add more analysis, or change figure layout

## Example Workflow

```bash
# 1. Build the workspace (first time or after code changes)
docker exec ros-noetic-workspace bash -c "cd /workspace && source /opt/ros/noetic/setup.bash && catkin_make"

# 2. Start simulator (Terminal 1)
docker exec ros-noetic-workspace bash -c "source /workspace/devel/setup.bash && roslaunch turtlebot_simulator simulator.launch"

# 3. Start controller (Terminal 2)
docker exec ros-noetic-workspace bash -c "source /workspace/devel/setup.bash && roslaunch turtlebot_traj_ctrl trajectory_controller.launch"

# 4. Wait for simulation to run (10-20 seconds)

# 5. Stop both nodes (Ctrl+C in both terminals)

# 6. Plot the results
python3 plot_rosbag.py --latest
```

Enjoy your trajectory tracking analysis! 🚀

