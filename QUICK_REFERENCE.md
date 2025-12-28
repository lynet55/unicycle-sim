# Quick Reference Card

## 🚀 Complete Workflow (Copy & Paste)

### 1️⃣ First Time Setup (One-time only)
```bash
# Start Docker
docker compose up -d

# Build workspace
docker exec ros-noetic-workspace bash -c "cd /workspace && source /opt/ros/noetic/setup.bash && catkin_make"

# Install Python dependencies (on host machine)
pip3 install matplotlib numpy
```

### 2️⃣ Run Simulation with Auto-Recording

**Terminal 1:**
```bash
docker exec ros-noetic-workspace bash -c "source /workspace/devel/setup.bash && roslaunch turtlebot_simulator simulator.launch"
```

**Terminal 2:**
```bash
docker exec ros-noetic-workspace bash -c "source /workspace/devel/setup.bash && roslaunch turtlebot_traj_ctrl trajectory_controller.launch"
```

⏱️ **Let it run for 10-20 seconds, then press Ctrl+C in both terminals**

### 3️⃣ Plot Results
```bash
python3 plot_rosbag.py --latest
```

---

## 📊 Plotting Options

```bash
# Most recent bag file
python3 plot_rosbag.py --latest

# Specific bag file
python3 plot_rosbag.py rosbags/simulation_2024-12-28-10-30-45.bag

# Help
python3 plot_rosbag.py --help
```

---

## 📁 Important Files

| File | Purpose |
|------|---------|
| `plot_rosbag.py` | Main plotting script |
| `rosbags/` | Recorded bag files storage |
| `PLOTTING_GUIDE.md` | Detailed documentation |
| `requirements.txt` | Python dependencies |

---

## 🔍 Debugging Commands

```bash
# Check if topics are publishing
docker exec ros-noetic-workspace bash -c "source /workspace/devel/setup.bash && rostopic list"

# Monitor a specific topic
docker exec ros-noetic-workspace bash -c "source /workspace/devel/setup.bash && rostopic echo /turtlebot/reference"

# Check bag file contents
rosbag info rosbags/simulation_YYYY-MM-DD-HH-MM-SS.bag

# List all bag files
ls -lht rosbags/
```

---

## 📈 What You'll Get

The plotting script generates:
1. **2D Trajectory**: Actual path vs reference (scatter + line)
2. **X Tracking**: X and X_ref vs time
3. **Y Tracking**: Y and Y_ref vs time  
4. **Control Commands**: v_cmd and ω_cmd vs time
5. **Statistics**: Error metrics printed to console
6. **PNG File**: High-res saved plot

---

## 🛠️ Rebuild After Code Changes

```bash
docker exec ros-noetic-workspace bash -c "cd /workspace && source /opt/ros/noetic/setup.bash && catkin_make"
```

---

## 🧹 Cleanup Old Bags

```bash
# List bags by size
ls -lhS rosbags/

# Remove old bags (keeps last 5)
cd rosbags && ls -t *.bag | tail -n +6 | xargs rm -f && cd ..
```

---

## 🆘 Common Issues

| Problem | Solution |
|---------|----------|
| No plots appear | Check `rosbags/` directory exists and has `.bag` files |
| Empty plots | Run simulation longer (15-20 seconds minimum) |
| No reference trajectory | Rebuild workspace after code changes |
| rosbag command not found | Run inside Docker container |
| Python import error | Install: `pip3 install matplotlib numpy` |

---

## 📞 Need Help?

See detailed guides:
- `PLOTTING_GUIDE.md` - Full plotting documentation
- `IMPLEMENTATION_SUMMARY.md` - Technical details
- `README.md` - General project info

