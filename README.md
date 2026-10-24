[Screencast from 2026-10-24 20-35-26.webm](https://github.com/user-attachments/assets/153d0ae6-365f-4894-ae3b-8a623582fe39)
# Fire Rover ROS 2 Simulation

A differential-drive fire-fighting rover simulation for ROS 2 Jazzy and Gazebo
Harmonic. The workspace contains robot description, control, navigation,
sensors, detection, and bringup packages, plus scripts for keyboard driving and
a simple simulated fire-and-water interaction.

## Requirements

- Ubuntu with ROS 2 Jazzy
- Gazebo Harmonic and `ros_gz`
- `colcon`, `xacro`, `robot_state_publisher`, and ROS 2 control/controller
  packages used by the rover description
- `nav_msgs` and `geometry_msgs` Python interfaces

Install any missing ROS packages with the ROS 2 Jazzy package manager. Build
from the workspace root:

```bash
cd ~/fire_rover_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Run

Open an interactive terminal and launch the simulation:

```bash
cd ~/fire_rover_ws
bash scripts/run_simulation.sh
```

The launcher starts Gazebo, spawns the rover and a fire target, activates the
drive controllers, and runs keyboard teleoperation. It assigns a separate ROS
domain and Gazebo partition to each launch to avoid conflicts with stale
simulation processes.

## Controls

- **Up / Down arrows:** drive forward / backward
- **Left / Right arrows:** rotate
- **Space:** start the water spray
- **Any other key:** stop the spray
- **Ctrl+C:** stop the simulation

The fire target appears at approximately `(2.5, 0)` in the Gazebo world. Face
the target and start spraying from within range. The fire is removed after the
water stream intersects it continuously for about two seconds. The flame,
smoke, and water stream are visual/gameplay effects; this is not a fluid or
thermal simulation.

## Notes

- The scripts expect the workspace at `~/fire_rover_ws`, matching the default
  workspace path used by the launcher.
- Fire detection, autonomous navigation, and realistic water physics are not
  implemented by the current simulation scripts.
- Press **Ctrl+C** in the launch terminal to stop the run and its child
  processes.
