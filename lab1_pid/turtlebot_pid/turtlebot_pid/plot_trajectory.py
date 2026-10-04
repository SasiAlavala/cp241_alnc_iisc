#!/usr/bin/env python3
"""
Records /odom and saves the robot trajectory plot when you press Ctrl+C.
Start it in its own terminal BEFORE running pid_node:
    python3 plot_trajectory.py
After "GOAL REACHED", press Ctrl+C here -> plot saved to ~/trajectory.png
"""
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib
matplotlib.use('Agg')            # no screen needed
import matplotlib.pyplot as plt

XD, YD = 5.02, 1.78              # keep equal to self.xd, self.yd in pid_node.py


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')
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
        plt.plot(XD, YD, 'r*', ms=16, label='goal (%.2f, %.2f)' % (XD, YD))
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
    node.save_plot()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
