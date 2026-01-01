# Mobile Robots - Build & Run Commands

 [*Report*](deliverable_report.md)

## Start Docker Container

```bash
docker compose up -d
```

## Build Workspace

```bash
docker exec ros-noetic-workspace bash -c "cd /workspace && source /opt/ros/noetic/setup.bash && catkin_make"
```

## Run Nodes

There are two ways to run the simulation, depending on your use case:

### Setup 1: Manual Control with Test Node

This setup launches the simulator with a test node for manual control via ROS service calls. Use this for testing and manually commanding the robot.

**Single Terminal:**
```bash
docker exec ros-noetic-workspace bash -c "source /workspace/devel/setup.bash && roslaunch turtlebot_simulator simulator.launch"
```

This launches:
- `turtlebot_simulator` - The robot simulator
- `test_node` - Manual command node (responds to service calls)
- `rosbag_recorder` - Records simulation data

**To send manual commands:**
```bash
# Set velocity command
rosservice call /add_step "parameter: 'v_cmd'
step_value: 1.0"

# Set angular velocity command
rosservice call /add_step "parameter: 'omega_cmd'
step_value: 0.1"
```

### Setup 2: Autonomous Trajectory Tracking

This setup runs the simulator with an autonomous trajectory controller that makes the robot follow a figure-8 path.

**Terminal 1 - Start Simulator Only:**
```bash
docker exec ros-noetic-workspace bash -c "source /workspace/devel/setup.bash && roslaunch turtlebot_simulator simulator.launch"
```

**Terminal 2 - Start Trajectory Controller:**
```bash
docker exec ros-noetic-workspace bash -c "source /workspace/devel/setup.bash && roslaunch turtlebot_traj_ctrl trajectory_controller.launch"
```

This configuration launches:
- `turtlebot_simulator` - The robot simulator (Terminal 1)
- `test_node` - Manual control (Terminal 1, but inactive)
- `trajectory_controller` - Autonomous trajectory tracking controller (Terminal 2)
- `rosbag_recorder` - Records simulation data (Terminal 1)

**Note:** Both `test_node` and `trajectory_controller` publish to the same `/control_commands` topic. The trajectory controller will override any manual commands when active.

## Interactive Shell (Optional)

```bash
docker exec -it ros-noetic-workspace bash
source /workspace/devel/setup.bash
```

## Plot Simulation Results

After running the simulation, plot the recorded data:

```bash
python3 plot_rosbag.py --latest
```

## Simulation Results

### Trajectory Animation

![Robot Trajectory Animation](catkin_ws/rosbags/simulation_2025-12-29-10-19-24_animation.gif)

The animation shows the robot (black circle) following the reference trajectory in real-time.

### Detailed Plots

![Simulation Analysis Plots](catkin_ws/rosbags/simulation_2025-12-29-10-19-24_plots.png)

The plots show:
- 2D trajectory tracking (actual vs reference)
- X and Y position tracking over time
- Angular velocity
- Linear and angular velocity commands

## Stop Container

```bash
docker compose down
```
