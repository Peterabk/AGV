#ifndef SIMPLE_SPEED_CONTROLLER_HPP
#define SIMPLE_SPEED_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Core>

class SimpleSpeedController : public rclcpp::Node
{

    public:
        //this will take the name of ros2 node
        SimpleSpeedController(const std::string &name);
    private:
        /* data */
        void velCallback(const geometry_msgs::msg::Twist & msg);
        void jointCallback(const sensor_msgs::msg::JointState & msg);

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;


        double wheel_radius_;
        double wheel_separation_;
        double left_wheel_prev_pos_;
        double right_wheel_prev_pos_;

        double x_ = 0.0;
        double y_ = 0.0;
        double theta_ = 0.0;

        nav_msgs::msg::Odometry odom_msg_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
        geometry_msgs::msg::TransformStamped transfrom_stamped_;

        Eigen::Matrix2d speed_conversion_;
        rclcpp::Time prev_time_;


};

#endif