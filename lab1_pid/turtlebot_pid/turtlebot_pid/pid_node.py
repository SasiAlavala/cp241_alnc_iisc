import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        
        # Target Pose
        self.xd = 5.02
        self.yd = 1.78
        
        # Gains (Tune these)
        self.kp_p = 0.30; self.kd_p = 0.05; self.ki_p = 0.01
        self.kp_a = 1.00; self.kd_a = 0.10; self.ki_a = 0.01
        
        # Error States
        self.ep_prev = 0.0; self.ei_p = 0.0
        self.ea_prev = 0.0; self.ei_a = 0.0
        
        self.current_x = 0.0; self.current_y = 0.0; self.current_theta = 0.0
        self.state_received = False
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.timer = self.create_timer(0.1, self.control_loop) # 10Hz
        self.last_time = time.time()
        self.get_logger().info("Controller Started!")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_theta = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
        self.state_received = True

    def control_loop(self):
        if not self.state_received: return
            
        dt = time.time() - self.last_time
        if dt <= 0: dt = 0.001
        self.last_time = time.time()

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
        if ep_c < 0.05:  # stop within 5 cm of the goal
            self.get_logger().info("GOAL REACHED!")
            self.publisher_.publish(Twist()) # Stop robot
            self.timer.cancel()
            return

        # 4. Compute PID control inputs
        # Hint: Remember to saturate linear velocity (e.g., max 0.2 m/s)
        dep = (ep_c - self.ep_prev) / dt                                   # d(ep)/dt
        dea = math.atan2(math.sin(ea_c - self.ea_prev),
                         math.cos(ea_c - self.ea_prev)) / dt               # d(ea)/dt, wrapped
        ei_p_new = min(self.ei_p + ep_c * dt, 2.0)                         # integral, capped (0.01*2 = 0.02 m/s)
        ei_a_new = max(-5.0, min(self.ei_a + ea_c * dt, 5.0))              # integral, capped (0.01*5 = 0.05 rad/s)

        v_pid = self.kp_p * ep_c + self.ki_p * ei_p_new + self.kd_p * dep
        w_pid = self.kp_a * ea_c + self.ki_a * ei_a_new + self.kd_a * dea

        v = max(0.0, min(v_pid, 0.2)) * max(0.0, math.cos(ea_c))  # saturate to [0, 0.2], slow down if not facing goal
        omega = max(-0.2, min(w_pid, 0.2))                        # saturate to [-0.2, 0.2]
        
        # 5. Update previous and integral error states for the next loop
        self.ep_prev = ep_c
        if v_pid < 0.2: self.ei_p = ei_p_new          # anti-windup: integrate only when not saturated
        self.ea_prev = ea_c
        if abs(w_pid) < 0.2: self.ei_a = ei_a_new
        
        # ==========================================================
        # END OF YOUR CODE
        # ==========================================================
        
        cmd_msg = Twist()
        cmd_msg.linear.x = float(v)
        cmd_msg.angular.z = float(omega)
        self.publisher_.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()