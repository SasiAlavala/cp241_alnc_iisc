# Lab 1: PID control of TurtleBot3

Go-to-goal PID controller for a TurtleBot3 Burger, in simulation (ROS 2 Humble, Gazebo) and on
the physical robot (PhaseSpace motion capture, in MATLAB or Python).

| File | Purpose |
|---|---|
| `turtlebot_pid/turtlebot_pid/pid_node.py` | controller node for simulation |
| `turtlebot_pid/turtlebot_pid/plot_trajectory.py` | records `/odom` and plots the trajectory |
| `turtlebot_pid/turtlebot_pid/pid_node_mocap.py` | the same controller for the physical robot, from PhaseSpace markers |
| `turtlebot_pid/pid_ros_plot.m` | the same controller for the physical robot with MOCAP, in MATLAB |
| `turtlebot_pid/stop.m` | keeps sending zero velocity to the physical robot |
| `results/` | trajectory plots from simulation |

## Controller

Both versions use the same control law, at 10 Hz:

- Position error `ep` = distance to the goal, desired heading `theta_d = atan2(yd - y, xd - x)`,
  heading error `ea = theta_d - theta` wrapped to [-pi, pi]
- `v` = PID on `ep`, `w` = PID on `ea`
  - Distance: Kp = 0.30, Ki = 0.01, Kd = 0.05
  - Heading: Kp = 1.00, Ki = 0.01, Kd = 0.10
- `v` limited to [0, 0.2] m/s and multiplied by `max(0, cos(ea))`, so the robot turns towards the
  goal before driving; `w` limited to [-0.2, 0.2] rad/s
- Integrals capped and only updated when the output is not saturated (anti-windup); derivative
  skipped on the first step; `v` changes by at most 0.5 m/s^2
- The robot stops if the pose has not been updated for 0.5 s, and continues when it is back
- Stops within 2 cm of the goal in simulation, 5 cm on the physical robot

## Simulation

### 1. Set up every terminal

Run this line first in **every** new terminal (all terminals must use the same line):

Ubuntu 22.04, ROS 2 Humble in `/opt/ros`:

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && export ROS_LOCALHOST_ONLY=1
```

macOS, ROS 2 Humble from RoboStack (pixi environment in `~/ros2_ws`, zsh):

```zsh
eval "$(pixi shell-hook --manifest-path ~/ros2_ws/pixi.toml 2>/dev/null)" && source ~/ros2_ws/install/setup.zsh && export ROS_LOCALHOST_ONLY=1
```

`ROS_LOCALHOST_ONLY=1` keeps other ROS 2 computers on the same network (e.g. lab Wi-Fi) from
interfering with `/cmd_vel` and `/odom`.

### 2. Build (once, and after every change to the code)

```bash
cd ~/ros2_ws && colcon build --packages-select turtlebot_pid
```

Then run the setup line again (it sources the new build).

### 3. Run: three terminals, in this order

**Terminal 1, Gazebo:**

```bash
export TURTLEBOT3_MODEL=burger GAZEBO_IP=127.0.0.1 IGN_IP=127.0.0.1
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

`GAZEBO_IP` and `IGN_IP` keep Gazebo's own messaging on this computer; on networks that block
multicast (some campus Wi-Fi) the robot otherwise never spawns.

Wait for `Successfully spawned entity [burger]`. If it has not appeared after about a minute,
press Ctrl+C and run the launch command again.

**Terminal 2, trajectory recorder** (give it the same goal as terminal 3):

```bash
python3 ~/ros2_ws/src/cp241_alnc_iisc/lab1_pid/turtlebot_pid/turtlebot_pid/plot_trajectory.py --ros-args -p xd:=5.02 -p yd:=1.78
```

Wait for `Recording /odom ...`.

**Terminal 3, controller:**

```bash
ros2 run turtlebot_pid pid_node --ros-args -p xd:=5.02 -p yd:=1.78
```

When terminal 3 prints `GOAL REACHED!`:

1. Press Ctrl+C **once** in terminal 2 and wait for `Trajectory plot saved to .../trajectory.png`.
2. Press Ctrl+C in terminal 3. (Ctrl+C here at any time stops the robot.)
3. Open the plot: `open ~/trajectory.png` (macOS) or `xdg-open ~/trajectory.png` (Ubuntu).

For another goal, change `xd` and `yd` in **both** the terminal 2 and terminal 3 commands.

### 4. Next run, or start from another pose

Gazebo can stay open. In terminal 3, put the robot back at the origin:

```bash
ros2 service call /reset_world std_srvs/srv/Empty
```

For an arbitrary start pose, then drive the robot along an arc for 6 s and stop it:

```bash
timeout 6 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.5}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

macOS has no `timeout`; use this for the first line instead:

```zsh
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.5}}" & sleep 6; kill $!
```

Then start terminal 2 and terminal 3 again (step 3).

### Troubleshooting

| Problem | Fix |
|---|---|
| `command not found: ros2` / `colcon`, or `Package 'turtlebot_pid' not found` | Run the setup line (step 1) in that terminal; build once (step 2). |
| The robot never appears in Gazebo, or `Service /spawn_entity unavailable` | Ctrl+C in terminal 1 and run the launch command again. If the log shows `Exception sending a multicast message`, the `GAZEBO_IP`/`IGN_IP` line was not set in terminal 1. |
| Gazebo does not start, or behaves oddly after an earlier run | Ctrl+C everything, then `pkill -f gzserver; pkill -f gzclient` and start again. |
| `ros2 topic list` does not show `/odom` and `/cmd_vel` | That terminal was set up differently: use the same setup line everywhere. |
| The goal star in the plot is in the wrong place | Terminals 2 and 3 must get the same `xd`, `yd`. |
| No plot saved | Press Ctrl+C only once in terminal 2 and wait for the "saved" line. |
| The robot keeps moving after everything was closed | `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"` |
| Changes to `pid_node.py` have no effect | Build again (step 2), then the setup line. |

### Parameters of `pid_node`

All settings are ROS parameters (`--ros-args -p name:=value`):

| Parameter | Default | Meaning |
|---|---|---|
| `xd`, `yd` | 5.02, 1.78 | goal [m] |
| `tolerance` | 0.02 | stop this close to the goal [m] |
| `pose_topic` | `/odom` | topic with the robot pose |
| `pose_type` | `odom` | `odom` (nav_msgs/Odometry) or `pose_stamped` (geometry_msgs/PoseStamped) |
| `position_scale` | 1.0 | 0.001 if the pose is in millimetres |
| `heading_offset` | 0.0 | added to the measured heading [rad] |
| `pose_timeout` | 0.5 | stop the robot if no valid pose arrives for this long [s] |
| `max_accel` | 0.5 | limit on how fast v changes [m/s^2]; 0 = no limit |

Poses containing NaN or inf are ignored and count as no pose.

## Physical robot (MATLAB)

`turtlebot_pid/pid_ros_plot.m` is the course's MATLAB template with the controller above. It reads
the PhaseSpace pose from `/phasespace/pose` (`geometry_msgs/Pose2D`) and sends commands to
`/HWTB3_10/cmd_vel` on `ROS_DOMAIN_ID` 30. Needs MATLAB with the ROS Toolbox.

### Before the first run

1. **macOS only: network permission.** System Settings, Privacy & Security, Local Network: turn
   on MATLAB, then quit and reopen MATLAB. Without it MATLAB cannot see the MOCAP or the robot
   (`ros2 topic list` shows only `/parameter_events` and `/rosout`). If it still sees nothing,
   quit MATLAB and start it from a terminal that has the permission (e.g. VS Code's):
   `/Applications/MATLAB_R2026a.app/bin/matlab &`.
2. **Robot number.** The files use robot `HWTB3_10`. For another robot, change `/HWTB3_10/cmd_vel`
   in both `pid_ros_plot.m` and `stop.m`.
3. **Check the MOCAP pose** in the MATLAB Command Window:

   ```matlab
   setenv("ROS_DOMAIN_ID","30");
   node = ros2node("/pose_check");
   sub = ros2subscriber(node, "/phasespace/pose", "geometry_msgs/Pose2D");
   pause(2); sub.LatestMessage
   ```

   - `x`, `y` must be in metres (small numbers inside the arena), not millimetres.
   - Turn the robot by hand about 90 degrees to the left and run `sub.LatestMessage` again:
     `theta` must increase by about 1.57 (radians). Pushed forward, `x`, `y` must change in the
     direction the robot faces. If not, ask the TAs before running the controller.
   - Then `clear node sub`.
4. **Goal.** Pick a goal inside the MOCAP area; for the first run, about 0.5 m from the robot.
5. Open `stop.m` in a second tab, ready for an emergency.

### Run

1. Open `pid_ros_plot.m` and set the goal under "Target Pose" (`xd`, `yd` in metres).
2. Press Run. The console prints the position, error, `v` and `w` at 10 Hz and the live plot
   shows the robot.
3. At the goal it prints `GOAL REACHED!`, stops the robot and saves `robot_trajectory.csv` in the
   current MATLAB folder.

If `No valid MOCAP pose, stopping` appears, the markers are not visible; the robot waits and
continues by itself once they are tracked again.

### Emergency stop

Ctrl+C stops the script but not the robot. Then run `stop.m`: it keeps sending zero velocity until
you press Ctrl+C in it.

## Physical robot (Python, PhaseSpace markers)

`pid_node_mocap` runs the same controller on the lab's ROS 2 setup. It reads the PhaseSpace
markers from `/phasespace/markers` (`phasespace_msgs/Markers`): the robot's position is the
midpoint of its two markers and its heading is the direction from the back to the front marker.
It publishes to `/r1a005/cmd_vel` by default.

`phasespace_msgs` comes with the lab's PhaseSpace driver. If the node prints
`phasespace_msgs not found` (or Python reports `No module named 'phasespace_msgs'`), that
workspace has not been sourced in this terminal.

The file runs on its own, like the lab's demo code (it does not need this package to be built):

```bash
source /opt/ros/humble/setup.bash
source ~/<phasespace_ws>/install/setup.bash      # the workspace with phasespace_msgs
python3 pid_node_mocap.py                        # asks: Enter your value X: / Enter your value Y:
```

The goal can also be given directly, which skips the questions:
`python3 pid_node_mocap.py --ros-args -p xd:=1.0 -p yd:=0.5`. After `colcon build` the same node
is available as `ros2 run turtlebot_pid pid_node_mocap`.

Do not set `ROS_LOCALHOST_ONLY` here (the robot and MOCAP are on the network); set
`ROS_DOMAIN_ID` if the lab uses one. The first line it prints after the markers are seen is
`First MOCAP pose: (x, y), heading ...`: check that it matches where the robot is, in metres.

| Parameter | Default | Meaning |
|---|---|---|
| `xd`, `yd` | asked at start | goal [m], inside the MOCAP area |
| `tolerance` | 0.05 | stop this close to the goal [m] |
| `v_max`, `w_max` | 0.3, 1.0 | speed limits [m/s, rad/s], as in the lab demo; use `0.2` for both to match the assignment's limits |
| `cmd_vel_topic` | `/r1a005/cmd_vel` | the robot's velocity topic |
| `markers_topic` | `/phasespace/markers` | PhaseSpace markers topic |
| `front_marker_id`, `back_marker_id` | -1, -1 | the robot's marker IDs; -1 uses the first two markers in the message, which is only correct if no other markers are visible |
| `position_scale` | 1.0 | 0.001 if the markers are in millimetres |
| `heading_offset` | 0.0 | added to the measured heading [rad] |
| `pose_timeout` | 0.5 | stop the robot if no valid pose arrives for this long [s] |
| `max_accel` | 0.5 | limit on how fast v changes [m/s^2]; 0 = no limit |

Markers that the cameras cannot see (negative `cond`) or that are missing count as no pose: the
robot stops and continues when they are tracked again. Ctrl+C stops the robot.

## Results

Start at the origin: goal reached in 32 s, final error 0.1 cm.

![Default start](results/trajectory_default.png)

Arbitrary start (0.04, 0.59), heading 177 deg: goal reached in 41 s, final error 1.8 cm.

![Arbitrary start](results/trajectory_arbitrary_start.png)
