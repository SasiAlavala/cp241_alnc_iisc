#!/usr/bin/env python3
"""
Records /odom and saves the robot trajectory plot when you press Ctrl+C.
Start it in its own terminal BEFORE running pid_node:
    python3 plot_trajectory.py
For another goal pass the same values as to pid_node:
    python3 plot_trajectory.py --ros-args -p xd:=2.0 -p yd:=-1.0
After "GOAL REACHED", press Ctrl+C here -> plot saved to ~/trajectory.png
"""
import os
import signal
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from nav_msgs.msg import Odometry
import matplotlib
matplotlib.use('Agg')            # no screen needed
import matplotlib.pyplot as plt

XD, YD = 5.02, 1.78              # default goal, same as pid_node.py


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')
        any_number = ParameterDescriptor(dynamic_typing=True)
        self.xd = float(self.declare_parameter('xd', XD, any_number).value)
        self.yd = float(self.declare_parameter('yd', YD, any_number).value)
        self.xs, self.ys = [], []
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.get_logger().info("Recording /odom ... press Ctrl+C after GOAL REACHED to save the plot")

    def odom_callback(self, msg):
        self.xs.append(msg.pose.pose.position.x)
        self.ys.append(msg.pose.pose.position.y)

    def save_plot(self):
        if not self.xs:
            print("No /odom messages received - nothing to plot")
            return
        plt.figure(figsize=(7, 5))
        plt.plot(self.xs, self.ys, 'b-', lw=2, label='robot trajectory')
        plt.plot(self.xs[0], self.ys[0], 'go', ms=10, label='start')
        plt.plot(self.xd, self.yd, 'r*', ms=16, label='goal (%.2f, %.2f)' % (self.xd, self.yd))
        plt.xlabel('x [m]'); plt.ylabel('y [m]')
        plt.title('TurtleBot3 trajectory with PID control')
        plt.axis('equal'); plt.grid(True); plt.legend()
        path = os.path.join(os.path.expanduser('~'), 'trajectory.png')
        plt.savefig(path, dpi=150)
        print("Trajectory plot saved to", path)


def main():
    rclpy.init()
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    signal.signal(signal.SIGINT, signal.SIG_IGN)   # a second Ctrl+C must not stop the saving
    node.save_plot()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
