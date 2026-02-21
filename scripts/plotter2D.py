#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D


class Plotter2D(Node):

    def __init__(self):
        super().__init__('plotter_2d')

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/output/robot_pose',
            self.pose_callback,
            10)

        self.target_sub = self.create_subscription(
            Point,
            '/target',
            self.target_callback,
            10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.target_x = 0.0
        self.target_y = 0.0

        self.timer = self.create_timer(0.03, self.timer_callback)

        # Setup matplotlib
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal')
        self.ax.grid(True)

        self.robot_patch = Rectangle((-0.05, -0.05), 0.1, 0.1, color='blue')
        self.ax.add_patch(self.robot_patch)

        self.target_plot, = self.ax.plot([], [], 'ro', markersize=10)
        
        self.trail_x = []
        self.trail_y = []
        
        self.trail_plot, = self.ax.plot([], [], 'b-', linewidth=1)  

        self.get_logger().info("Matplotlib Plotter Started")

    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

        q = msg.pose.orientation
        self.current_theta = q.z

    def target_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y

    def timer_callback(self):

        # Save trail
        self.trail_x.append(self.current_x)
        self.trail_y.append(self.current_y)

        max_length = 1000
        if len(self.trail_x) > max_length:
            self.trail_x.pop(0)
            self.trail_y.pop(0)

        # Update robot transform
        transform = Affine2D().rotate(self.current_theta).translate(
            self.current_x, self.current_y)

        self.robot_patch.set_transform(transform + self.ax.transData)

        # Update trail
        self.trail_plot.set_data(self.trail_x, self.trail_y)

        # Update target
        self.target_plot.set_data(self.target_x, self.target_y)

        # Adjust view dynamically
        self.ax.set_xlim(self.current_x - 1, self.current_x + 1)
        self.ax.set_ylim(self.current_y - 1, self.current_y + 1)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()                  


def main(args=None):
    rclpy.init(args=args)
    node = Plotter2D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
