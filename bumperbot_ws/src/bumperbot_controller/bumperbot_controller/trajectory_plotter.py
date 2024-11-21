#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import os

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('TrajectoryPlotter')

        # Subscription to multiple topics
        self.sub_odom1 = self.create_subscription(
            Odometry,
            '/bumperbot_controller/odom',  # Replace with your first odometry topic
            self.callback_odom1,
            10
        )
        self.sub_odom2 = self.create_subscription(
            Odometry,
            '/bumperbot_controller/odom_noisy',  # Replace with your second odometry topic
            self.callback_odom2,
            10
        )

        self.sub_odom3 = self.create_subscription(
            Odometry,
            '/imu_ekf',  # Replace with your second odometry topic
            self.callback_odom3,
            10
        )

        # Data storage for each topic
        self.odom1_positions = {'x': [], 'y': []}
        self.odom2_positions = {'x': [], 'y': []}
        self.odom3_positions = {'x': [], 'y': []}

        # Real-time plotting setup
        plt.ion()  # Enable interactive mode
        self.figure, self.ax = plt.subplots()
        self.ax.set_title("Trajectory Comparison (Real-Time)")
        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")
        self.ax.grid()

        self.get_logger().info("MultiTopicPlotter node has started.")

    def callback_odom1(self, msg):
        # Extract x and y positions for odom1
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.odom1_positions['x'].append(x)
        self.odom1_positions['y'].append(y)
        self.update_plot()

    def callback_odom2(self, msg):
        # Extract x and y positions for odom2
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.odom2_positions['x'].append(x)
        self.odom2_positions['y'].append(y)
        self.update_plot()

    def callback_odom3(self, msg):
        # Extract x and y positions for odom2
        x = msg[0].pose.pose.position.x
        y = msg[0].pose.pose.position.y

        x_one = msg[1].pose.pose.position.x
        y_one = msg[1].pose.pose.position.y

        x_two = msg[2].pose.pose.position.x
        y_two = msg[2].pose.pose.position.y

        self.odom3_positions['x'].append(x)
        self.odom3_positions['x'].append(x_one)
        self.odom3_positions['x'].append(x_two)
        self.odom3_positions['y'].append(y)
        self.odom3_positions['y'].append(y_one)
        self.odom3_positions['y'].append(y_two)

        self.update_plot()

    def update_plot(self):
        # Update the plot in real-time
        self.ax.clear()
        self.ax.set_title("Trajectory Comparison (Real-Time)")
        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")

        # Plot odom1
        self.ax.plot(self.odom1_positions['x'], self.odom1_positions['y'], label="Odometry 1", linestyle='-', marker='o')
        # Plot odom2
        self.ax.plot(self.odom2_positions['x'], self.odom2_positions['y'], label="Odometry 2", linestyle='--', marker='x')
        self.ax.plot(self.odom3_positions['x'], self.odom3_positions['y'], label="Odometry 3", linestyle='-', marker='*')

        self.ax.legend()
        self.ax.grid()
        plt.pause(0.01)  # Refresh the plot

    def save_plot(self):
        # Save the plot as an image
        plt.ioff()  # Disable interactive mode
        output_path = os.path.join(os.getcwd(), 'comparison_plot.png')
        self.ax.clear()
        self.ax.plot(self.odom1_positions['x'], self.odom1_positions['y'], label="Odometry 1", linestyle='-', marker='o')
        self.ax.plot(self.odom2_positions['x'], self.odom2_positions['y'], label="Odometry 2", linestyle='--', marker='x')
        self.ax.plot(self.odom3_positions['x'], self.odom3_positions['y'], label="Odometry 3", linestyle='-', marker='*')

        self.ax.set_title("Trajectory Comparison")
        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")
        self.ax.legend()
        self.ax.grid()
        plt.savefig(output_path)
        self.get_logger().info(f"Comparison plot saved as '{output_path}'")
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save the plot on shutdown
        node.save_plot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()