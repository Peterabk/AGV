#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
import numpy as np
import math

class SimpleSpeedController(Node):
    def __init__(self):
        super().__init__("simple_speed_controller")

        #crete a parameter for this node
        #so we can implement it with different robots with different wheels
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_seperation", 0.17)

        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_seperation = self.get_parameter("wheel_seperation").get_parameter_value().double_value

        self.get_logger().info("Using wheel_radius %f" %self.wheel_radius)
        self.get_logger().info("Using wheel_seperation %f" %self.wheel_seperation)

        #calculating the differential kinematics for wheel odometry lesson 96
        self.left_wheel_previous_pos = 0.0
        self.right_wheel_previous_pos = 0.0

        #wheel odometry for position and orientation of the robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0


        self.previous_time = self.get_clock().now()

        self.wheel_cmd_pub = self.create_publisher(Float64MultiArray,"simple_velocity_controller/commands",10)
        self.vel_sub = self.create_subscription(TwistStamped,"bumperbot_controller/cmd_vel", self.velCallback, 10)

        #subscribe to get the state of the positoins
        self.joint_sub = self.create_subscription(JointState, "joint_states", self.jointCallBack, 10)

        #publish the odometry messages using the nav_msgs library with the Odometry type
        self.odom_pub = self.create_publisher(Odometry, "bumperbot_controller/odom", 10)

        #matrix that hold the values for the conversion of the velocity
        self.speed_conversion = np.array([[self.wheel_radius/2, self.wheel_radius/2], 
                                          [self.wheel_radius/self.wheel_seperation, -self.wheel_radius/self.wheel_seperation]])
        
        self.odom_msg = Odometry()
        #frame it uses to express it's movement, this is the fixed frame
        self.odom_msg.header.frame_id = "odom"
        # this is the moving frame to the respect of the fixed frame
        self.odom_msg.child_frame_id = "base_footprint"
        self.odom_msg.pose.pose.orientation.x = 0.0
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = 0.0
        self.odom_msg.pose.pose.orientation.w = 1.0

        #Rviz2 part
        #visualizing and drawing the transformation between the fixed and dynamic frame in rviz2
        self.broadcaster = TransformBroadcaster(self)
        self.transform_stamped = TransformStamped()
        self.transform_stamped.header.frame_id = "odom"
        self.transform_stamped.child_frame_id = "base_footprint"

        self.get_logger().info("The conversion Matrix is: %s" %self.speed_conversion)

    def velCallback(self,msg):
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion), robot_speed)
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1,0], wheel_speed[0,0]]
        self.wheel_cmd_pub.publish(wheel_speed_msg)

    def jointCallBack(self, msg):
        #position of the left wheel at the current moment in time
        dp_left_wheel = msg.position[1] - self.left_wheel_previous_pos
        #position of the right wheel at the current moment in time
        dp_right_wheel = msg.position[0] - self.right_wheel_previous_pos
        #getting the delta time
        dt = Time.from_msg(msg.header.stamp) - self.previous_time

        self.left_wheel_previous_pos = msg.position[1]
        self.right_wheel_previous_pos = msg.position[0]
        self.previous_time = Time.from_msg(msg.header.stamp)

        #get the time to seconds that's why we use the S_TO_NS
        fi_left = dp_left_wheel / (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right_wheel / (dt.nanoseconds / S_TO_NS)

        linear = (self.wheel_radius * fi_right + self.wheel_radius * fi_left)/2
        angular = (self.wheel_radius * fi_right - self.wheel_radius * fi_left)/self.wheel_seperation

        #position increment of the robot
        d_s = (self.wheel_radius * dp_right_wheel + self.wheel_radius * dp_left_wheel) / 2
        d_theta = (self.wheel_radius * dp_right_wheel - self.wheel_radius * dp_left_wheel) / self.wheel_seperation

        self.theta += d_theta

        #we neeed to use trigonometry to increase the x and y seperatly
        #that's why we use the library math
        self.x += d_s * math.cos(self.theta)
        self.y += d_s * math.sin(self.theta)

        self.get_logger().info("linear: %f, angular: %f" % (linear,angular))
        self.get_logger().info("x: %f, y: %f, theta: %f "% (self.x,self.y,self.theta))

        q = quaternion_from_euler(0, 0, self.theta)
        self.odom_msg.pose.pose.orientation.x = q[0]
        self.odom_msg.pose.pose.orientation.y = q[1]
        self.odom_msg.pose.pose.orientation.z = q[2]
        self.odom_msg.pose.pose.orientation.w = q[3]
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.twist.twist.linear.x = linear
        self.odom_msg.twist.twist.angular.z = angular

        self.odom_pub.publish(self.odom_msg)

        #Rviz2 part
        #visualizing and drawing the transformation between the fixed and dynamic frame in rviz2
        self.transform_stamped.transform.translation.x = self.x
        self.transform_stamped.transform.translation.y = self.y
        self.transform_stamped.transform.rotation.x = q[0]
        self.transform_stamped.transform.rotation.y = q[1]
        self.transform_stamped.transform.rotation.z = q[2]
        self.transform_stamped.transform.rotation.w = q[3]

        self.transform_stamped.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(self.transform_stamped)


def main():
    rclpy.init()
    simple_speed_controller = SimpleSpeedController()
    rclpy.spin(simple_speed_controller)
    simple_speed_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()