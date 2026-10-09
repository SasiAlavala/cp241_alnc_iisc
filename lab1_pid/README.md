# Lab 1: PID control of TurtleBot3

Go-to-goal PID controller for a TurtleBot3 Burger, in simulation (ROS 2 Humble, Gazebo) and on
the physical robot (MATLAB, PhaseSpace motion capture).

| File | Purpose |
|---|---|
| `turtlebot_pid/turtlebot_pid/pid_node.py` | controller node for simulation |
| `turtlebot_pid/turtlebot_pid/plot_trajectory.py` | records `/odom` and plots the trajectory |
| `turtlebot_pid/pid_ros_plot.m` | the same controller for the physical robot with MOCAP |
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

### Build

Ubuntu 22.04 with ROS 2 Humble in `/opt/ros`:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select turtlebot_pid
source install/setup.bash
```

macOS with ROS 2 Humble from RoboStack (pixi environment in `~/ros2_ws`, zsh):

```zsh
cd ~/ros2_ws
pixi shell
colcon build --packages-select turtlebot_pid
source install/setup.zsh
```

### Run

Every terminal must be set up first: on Ubuntu `source /opt/ros/humble/setup.bash` and
`source ~/ros2_ws/install/setup.bash`; on macOS `cd ~/ros2_ws`, `pixi shell`, then
`source install/setup.zsh`.

```bash
# terminal 1
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py

# terminal 2
python3 ~/ros2_ws/src/cp241_alnc_iisc/lab1_pid/turtlebot_pid/turtlebot_pid/plot_trajectory.py

# terminal 3
ros2 run turtlebot_pid pid_node
```

After `GOAL REACHED!`, press Ctrl+C in terminal 2. The plot is saved to `~/trajectory.png`.
Pressing Ctrl+C in terminal 3 at any time stops the robot.

To use another goal, pass it to both scripts:

```bash
python3 ~/ros2_ws/src/cp241_alnc_iisc/lab1_pid/turtlebot_pid/turtlebot_pid/plot_trajectory.py --ros-args -p xd:=-2.0 -p yd:=1.0
ros2 run turtlebot_pid pid_node --ros-args -p xd:=-2.0 -p yd:=1.0
```

To test from a different start pose, restart Gazebo and move the robot first:

```bash
timeout 6 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.5}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

macOS has no `timeout`; use this for the first line instead:

```zsh
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.5}}" & sleep 6; kill $!
```

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
`/HWTB3_10/cmd_vel` on `ROS_DOMAIN_ID` 30.

1. Open `pid_ros_plot.m` and set the goal under "Target Pose" (`xd`, `yd` in metres, inside the MOCAP area).
2. Press Run. The live plot shows the robot; at the goal it stops and saves `robot_trajectory.csv`.
3. Ctrl+C stops the script but not the robot: then run `stop.m`, which keeps sending zero velocity.

## Results

Start at the origin: goal reached in 32 s, final error 0.1 cm.

![Default start](results/trajectory_default.png)

Arbitrary start (0.04, 0.59), heading 177 deg: goal reached in 41 s, final error 1.8 cm.

![Arbitrary start](results/trajectory_arbitrary_start.png)
