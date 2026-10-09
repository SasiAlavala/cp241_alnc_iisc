import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import time

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        any_number = ParameterDescriptor(dynamic_typing=True)
        def number(name, default):
            value = float(self.declare_parameter(name, default, any_number).value)
            if not math.isfinite(value):
                raise ValueError("%s must be a finite number" % name)
            return value

        # Target Pose
        self.xd = number('xd', 5.02)
        self.yd = number('yd', 1.78)
        self.tolerance = number('tolerance', 0.02)        # m, stop this close to the goal
        if not 0.005 <= self.tolerance <= 1.0:
            raise ValueError("tolerance must be between 0.005 and 1.0 m")

        # Pose source: Gazebo /odom by default; for MOCAP set the topic, type, units and heading offset
        self.pose_topic = self.declare_parameter('pose_topic', '/odom').value
        self.pose_type = self.declare_parameter('pose_type', 'odom').value  # 'odom' or 'pose_stamped'
        self.position_scale = number('position_scale', 1.0)  # 0.001 if the pose is in millimetres
        self.heading_offset = number('heading_offset', 0.0)  # rad, added to the measured heading
        self.pose_timeout = number('pose_timeout', 0.5)      # s, stop if no new pose for this long
        self.max_accel = number('max_accel', 0.5)            # m/s^2, 0 = no limit
        if self.pose_type not in ('odom', 'pose_stamped'):
            raise ValueError("pose_type must be 'odom' or 'pose_stamped'")
        if self.position_scale <= 0 or self.pose_timeout <= 0 or self.max_accel < 0:
            raise ValueError("position_scale and pose_timeout must be > 0, max_accel >= 0")

        # Gains (Tune these)
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

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        msg_type = Odometry if self.pose_type == 'odom' else PoseStamped
        self.subscriber_ = self.create_subscription(msg_type, self.pose_topic, self.odom_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop) # 10Hz
        self.last_time = time.time()
        self.get_logger().info("Controller Started! Goal: (%.2f, %.2f), pose from %s"
                               % (self.xd, self.yd, self.pose_topic))

    def odom_callback(self, msg):
        pose = msg.pose.pose if self.pose_type == 'odom' else msg.pose
        x = pose.position.x * self.position_scale
        y = pose.position.y * self.position_scale
        q = pose.orientation
        theta = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2)) + self.heading_offset
        if not all(math.isfinite(v) for v in (x, y, theta)):
            return  # bad sample (e.g. MOCAP lost the markers); treated like no pose
        self.current_x = x; self.current_y = y
        self.current_theta = math.atan2(math.sin(theta), math.cos(theta))
        self.last_pose_time = time.time()
        self.state_received = True

    def stop_robot(self):
        self.v_prev = 0.0
        self.publisher_.publish(Twist())

    def control_loop(self):
        if not self.state_received: return

        dt = time.time() - self.last_time
        if dt <= 0: dt = 0.001
        dt = min(dt, 0.5)  # a long gap must not blow up the I and D terms
        self.last_time = time.time()

        # Stop while the pose is not updating, resume when it comes back
        if time.time() - self.last_pose_time > self.pose_timeout:
            if not self.pose_lost:
                self.get_logger().warn("No pose for %.1f s, stopping" % self.pose_timeout)
                self.pose_lost = True
            self.stop_robot()
            return
        if self.pose_lost:
            self.get_logger().info("Pose is back, resuming")
            self.pose_lost = False
            self.first_loop = True

        # ==========================================================
        # TODO: ADD YOUR CODE HERE
        # ==========================================================

        # 1. Calculate position error (ep_c) and desired heading (theta_d)
        ep_c = math.sqrt((self.xd - self.current_x)**2 + (self.yd - self.current_y)**2)
        theta_d = math.atan2(self.yd - self.current_y, self.xd - self.current_x)

        # 2. Calculate heading error (ea_c) and wrap to [-pi, pi]
        ea_c = theta_d - self.current_theta
        ea_c = math.atan2(math.sin(ea_c), math.cos(ea_c))

        # 3. Check stop condition (e.g., if ep_c is less than a tolerance)
        if ep_c < self.tolerance:
            self.get_logger().info("GOAL REACHED!")
            self.stop_robot() # Stop robot
            self.timer.cancel()
            return

        # 4. Compute PID control inputs
        # Hint: Remember to saturate linear velocity (e.g., max 0.2 m/s)
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

        v = max(0.0, min(v_pid, 0.2)) * max(0.0, math.cos(ea_c))  # saturate to [0, 0.2], slow down if not facing goal
        omega = max(-0.2, min(w_pid, 0.2))                        # saturate to [-0.2, 0.2]
        if self.max_accel > 0:                                    # ramp v, no jerks on the real robot
            dv = self.max_accel * dt
            v = max(self.v_prev - dv, min(v, self.v_prev + dv))

        # 5. Update previous and integral error states for the next loop
        self.ep_prev = ep_c
        if v_pid < 0.2: self.ei_p = ei_p_new          # anti-windup: integrate only when not saturated
        self.ea_prev = ea_c
        if abs(w_pid) < 0.2: self.ei_a = ei_a_new
        self.v_prev = v

        # ==========================================================
        # END OF YOUR CODE
        # ==========================================================

        cmd_msg = Twist()
        cmd_msg.linear.x = float(v)
        cmd_msg.angular.z = float(omega)
        self.publisher_.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = PIDController()
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
