#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import sys
from robot_localization.srv import FromLL
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from geographic_msgs.msg import GeoPose
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import Quaternion, PoseStamped


class YamlWaypointParser:
    """
    Parse a set of GPS waypoints from a YAML file
    """
    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        Get an array of GeoPose objects from the YAML file
        """
        geopose_wps = []
        for wp in self.wps_dict.get("waypoints", []):
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp.get("yaw", 0.0)
            geopose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
        return geopose_wps


class GpsWpCommander(Node):
    """
    Use Nav2 GPS Waypoint Follower to follow waypoints from a YAML file
    """
    def __init__(self, wps_file_path):
        super().__init__('gps_wp_commander')
        self.navigator = BasicNavigator()
        self.wp_parser = YamlWaypointParser(wps_file_path)
        self.localizer = self.create_client(FromLL, '/fromLL')

        # Wait for localization service
        while not self.localizer.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /fromLL service...')

    def start_wpf(self):
        self.navigator.lifecycleStartup()
        waypoints = self.wp_parser.get_wps()
    
        map_waypoints = []
        for i, geo_wp in enumerate(waypoints):
            self.req = FromLL.Request()
            self.req.ll_point.latitude = geo_wp.position.latitude
            self.req.ll_point.longitude = geo_wp.position.longitude
    
            self.get_logger().info(f"Transforming GPS Waypoint {i+1}...")
            future = self.localizer.call_async(self.req)
            rclpy.spin_until_future_complete(self, future)
    
            if future.result() is None:
                self.get_logger().error(f"Failed to transform GPS Waypoint {i+1}.")
                continue
            
            map_pose = PoseStamped()
            map_pose.header.frame_id = 'map'
            map_pose.pose.position = future.result().map_point
            map_pose.pose.orientation = geo_wp.orientation
            map_waypoints.append(map_pose)
    
            self.get_logger().info(
                f"Waypoint {i+1} transformed: x={map_pose.pose.position.x}, "
                f"y={map_pose.pose.position.y}"
            )
    
        if not map_waypoints:
            self.get_logger().error("No valid waypoints. Aborting...")
            return
    
        self.navigator.followWaypoints(map_waypoints)
    
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(
                    f"Approaching waypoint {feedback.current_waypoint + 1}/{len(map_waypoints)}: "
                    f"Distance Remaining={feedback.distance_remaining:.2f} meters"
                )
    
        result = self.navigator.getResult()
        if result == 0:  # Result code 0 indicates success
            self.get_logger().info("Waypoints completed successfully!")
        else:
            self.get_logger().error(f"Failed to complete waypoints. Result code: {result}")



def latLonYaw2Geopose(latitude: float, longitude: float, yaw: float = 0.0) -> GeoPose:
    """
    Creates a GeoPose object from latitude, longitude, and yaw
    """
    geopose = GeoPose()
    geopose.position.latitude = latitude
    geopose.position.longitude = longitude

    q = quaternion_from_euler(0.0, 0.0, yaw)
    geopose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    return geopose


def main():
    rclpy.init()
    default_yaml_file_path = os.path.join(
        get_package_share_directory("bumperbot_controller"), "config", "waypoints.yaml"
    )
    yaml_file_path = sys.argv[1] if len(sys.argv) > 1 else default_yaml_file_path

    commander = GpsWpCommander(yaml_file_path)
    commander.start_wpf()


if __name__ == "__main__":
    main()
