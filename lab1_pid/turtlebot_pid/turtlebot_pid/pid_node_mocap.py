#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
import math
import time

try:
    from phasespace_msgs.msg import Markers
except ImportError:
    Markers = None

class PIDControllerMocap(Node):
    def __init__(self):
        super().__init__('pid_controller_mocap')

        any_number = ParameterDescriptor(dynamic_typing=True)
        def number(name, default):
            value = float(self.declare_parameter(name, default, any_number).value)
            if not math.isfinite(value):
                raise ValueError("%s must be a finite number" % name)
            return value

        # Target Pose: from -p xd:= -p yd:=, otherwise typed in like the lab demo
        self.xd = float(self.declare_parameter('xd', float('nan'), any_number).value)
        self.yd = float(self.declare_parameter('yd', float('nan'), any_number).value)
        if math.isnan(self.xd) or math.isnan(self.yd):
            self.get_input()
        if not (math.isfinite(self.xd) and math.isfinite(self.yd)):
            raise ValueError("goal must be finite numbers")
        self.tolerance = number('tolerance', 0.05)        # m, stop this close to the goal
        if not 0.005 <= self.tolerance <= 1.0:
            raise ValueError("tolerance must be between 0.005 and 1.0 m")

        # MOCAP and robot topics
        self.markers_topic = self.declare_parameter('markers_topic', '/phasespace/markers').value
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/r1a005/cmd_vel').value
        # Which markers are on this robot: their PhaseSpace IDs, or -1 to use the first two
        # markers in the message (like the lab demo; only correct if no other markers are visible)
        self.front_id = int(number('front_marker_id', -1))
        self.back_id = int(number('back_marker_id', -1))
        self.position_scale = number('position_scale', 1.0)  # 0.001 if the markers are in millimetres
        self.heading_offset = number('heading_offset', 0.0)  # rad, added to the measured heading
        self.pose_timeout = number('pose_timeout', 0.5)      # s, stop if no valid pose for this long
        self.max_accel = number('max_accel', 0.5)            # m/s^2, 0 = no limit
        # Speed limits as in the lab demo; the assignment asks for 0.2 and 0.2
        self.v_max = number('v_max', 0.3)                    # m/s, v is limited to [-v_max, v_max]
        self.w_max = number('w_max', 1.0)                    # rad/s, w is limited to [-w_max, w_max]
        if self.v_max <= 0 or self.w_max <= 0:
            raise ValueError("v_max and w_max must be > 0")
        if (self.front_id >= 0) != (self.back_id >= 0):
            raise ValueError("set both front_marker_id and back_marker_id, or neither")
        if self.position_scale <= 0 or self.pose_timeout <= 0 or self.max_accel < 0:
            raise ValueError("position_scale and pose_timeout must be > 0, max_accel >= 0")

        # Gains (same as simulation)
        self.kp_p = 0.30; self.kd_p = 0.05; self.ki_p = 0.01
        self.kp_a = 1.00; self.kd_a = 0.10; self.ki_a = 0.01

        # Error States
        self.ep_prev = 0.0; self.ei_p = 0.0
        self.ea_prev = 0.0; self.ei_a = 0.0
        self.first_loop = True
        self.v_prev = 0.0

        self.current_x = 0.0; self.current_y = 0.0; self.current_theta = 0.0
        self.state_received = False
        self.last_pose_time = 0.0
        self.pose_lost = False

        self.publisher_ = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.subscriber_ = self.create_subscription(Markers, self.markers_topic, self.markers_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop) # 10Hz
        self.last_time = time.time()
        self.get_logger().info("Controller Started! Goal: (%.2f, %.2f), markers from %s, commands to %s, "
                               "limits v %.2f m/s, w %.2f rad/s"
                               % (self.xd, self.yd, self.markers_topic, self.cmd_vel_topic, self.v_max, self.w_max))

    def get_input(self):
        def ask(prompt):
            while True:
                text = input(prompt)
                try:
                    value = float(text)
                    if math.isfinite(value):
                        return value
                except ValueError:
                    pass
                print("Please enter a number, e.g. 1.5")
        self.xd = ask("Enter your value X: ")
        self.yd = ask("Enter your value Y: ")

    def tracked(self, marker):
        # PhaseSpace marks a marker it cannot see with a negative condition value
        if getattr(marker, 'cond', 1.0) < 0:
            return False
        return math.isfinite(marker.x) and math.isfinite(marker.y)

    def markers_callback(self, msg):
        if self.front_id >= 0:
            by_id = {getattr(m, 'id', None): m for m in msg.markers}
            front, back = by_id.get(self.front_id), by_id.get(self.back_id)
        else:
            front, back = (msg.markers[0], msg.markers[1]) if len(msg.markers) >= 2 else (None, None)
        if front is None or back is None or not (self.tracked(front) and self.tracked(back)):
            return  # markers missing or not seen by the cameras; treated like no pose

        dx = front.x - back.x; dy = front.y - back.y
        if math.hypot(dx, dy) < 1e-6:
            return
        x = (front.x + back.x) / 2 * self.position_scale
        y = (front.y + back.y) / 2 * self.position_scale
        if abs(x) > 50 or abs(y) > 50:
            self.get_logger().error("Pose (%.1f, %.1f) looks like millimetres: restart with -p position_scale:=0.001"
                                    % (x, y), throttle_duration_sec=5.0)
            return
        theta = math.atan2(dy, dx) + self.heading_offset

        if not self.state_received:
            self.get_logger().info("First MOCAP pose: (%.3f, %.3f), heading %.0f deg" % (x, y, math.degrees(theta)))
        self.current_x = x; self.current_y = y
        self.current_theta = math.atan2(math.sin(theta), math.cos(theta))
        self.last_pose_time = time.time()
        self.state_received = True

    def stop_robot(self):
        self.v_prev = 0.0
        self.publisher_.publish(Twist())

    def control_loop(self):
        if not self.state_received:
            self.last_time = time.time()  # do not count the wait for the first pose in dt
            return

        dt = time.time() - self.last_time
        if dt <= 0: dt = 0.001
        dt = min(dt, 0.5)  # a long gap must not blow up the I and D terms
        self.last_time = time.time()

        # Stop while the pose is not updating, resume when it comes back
        if time.time() - self.last_pose_time > self.pose_timeout:
            if not self.pose_lost:
                self.get_logger().warn("No valid MOCAP pose for %.1f s, stopping" % self.pose_timeout)
                self.pose_lost = True
            self.stop_robot()
            return
        if self.pose_lost:
            self.get_logger().info("MOCAP pose is back, resuming")
            self.pose_lost = False
            self.first_loop = True

        # 1. Position error (ep_c) and desired heading (theta_d)
        ep_c = math.sqrt((self.xd - self.current_x)**2 + (self.yd - self.current_y)**2)
        theta_d = math.atan2(self.yd - self.current_y, self.xd - self.current_x)

        # 2. Heading error (ea_c), wrapped to [-pi, pi]
        ea_c = theta_d - self.current_theta
        ea_c = math.atan2(math.sin(ea_c), math.cos(ea_c))

        # 3. Stop condition
        if ep_c < self.tolerance:
            self.get_logger().info("GOAL REACHED! (error %.3f m)" % ep_c)
            self.stop_robot()
            self.timer.cancel()
            return

        # 4. PID control inputs
        if self.first_loop:                       # no previous error yet: avoid derivative kick
            self.ep_prev = ep_c; self.ea_prev = ea_c
            self.first_loop = False
        dep = (ep_c - self.ep_prev) / dt                                   # d(ep)/dt
        dea = math.atan2(math.sin(ea_c - self.ea_prev),
                         math.cos(ea_c - self.ea_prev)) / dt               # d(ea)/dt, wrapped
        ei_p_new = min(self.ei_p + ep_c * dt, 2.0)                         # integral, capped (0.01*2 = 0.02 m/s)
        ei_a_new = max(-5.0, min(self.ei_a + ea_c * dt, 5.0))              # integral, capped (0.01*5 = 0.05 rad/s)

        v_pid = self.kp_p * ep_c + self.ki_p * ei_p_new + self.kd_p * dep
        w_pid = self.kp_a * ea_c + self.ki_a * ei_a_new + self.kd_a * dea

        v = max(-self.v_max, min(v_pid, self.v_max)) * max(0.0, math.cos(ea_c))  # saturate, slow down if not facing goal
        omega = max(-self.w_max, min(w_pid, self.w_max))                         # saturate
        if self.max_accel > 0:                                    # ramp v, no jerks on the real robot
            dv = self.max_accel * dt
            v = max(self.v_prev - dv, min(v, self.v_prev + dv))

        # 5. Update previous and integral error states (anti-windup: integrate only when not saturated)
        self.ep_prev = ep_c
        if abs(v_pid) < self.v_max: self.ei_p = ei_p_new
        self.ea_prev = ea_c
        if abs(w_pid) < self.w_max: self.ei_a = ei_a_new
        self.v_prev = v

        self.get_logger().info("pose (%.2f, %.2f) | error %.3f m | v %.3f, w %.3f"
                               % (self.current_x, self.current_y, ep_c, v, omega), throttle_duration_sec=1.0)
        cmd_msg = Twist()
        cmd_msg.linear.x = float(v)
        cmd_msg.angular.z = float(omega)
        self.publisher_.publish(cmd_msg)

def main(args=None):
    if Markers is None:
        print("phasespace_msgs not found. Source the workspace that contains the PhaseSpace driver "
              "first, e.g.  source ~/<phasespace_ws>/install/setup.bash")
        return
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = PIDControllerMocap()
    except (KeyboardInterrupt, EOFError):  # Ctrl+C or Ctrl+D at the goal prompt
        print()
        rclpy.shutdown()
        return
    except Exception as e:  # bad --ros-args value
        print("Invalid parameter:", e)
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    for _ in range(3):  # stop, sent a few times in case one message is lost over Wi-Fi
        node.stop_robot()
        time.sleep(0.05)
    node.get_logger().info("Stopped")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
