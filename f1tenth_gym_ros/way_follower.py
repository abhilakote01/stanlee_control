#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import math

class StanLee(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.waypoints = np.loadtxt('/sim_ws/src/f1tenth_gym_ros/maps/SaoPaulo_centerline.csv', delimiter=',', skiprows=1, usecols=(0,1))

        self.current_index = 0
        self.distance_threshold = 0.3
        
        # TODO: create ROS subscribers and publishers
        
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        # Current position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Quaternion to yaw
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q)

        # Get current waypoint target
        target = self.waypoints[self.current_index]
        dx = target[0] - x
        dy = target[1] - y
        distance = math.hypot(dx, dy)

        # Check if close to waypoint; if yes, switch to next
        if distance < self.distance_threshold and self.current_index < len(self.waypoints) - 1:
            self.current_index += 1
            self.get_logger().info(f"Switching to waypoint {self.current_index}")

        # Calculate heading to target waypoint
        target_yaw = math.atan2(dy, dx)

        # Calculate steering angle (difference between target yaw and current yaw)
        angle_diff = self.normalize_angle(target_yaw - yaw)

        # Limit steering angle to +/- 24 degrees (0.4189 radians)
        max_steering_angle = 0.4189
        steering_angle = max(min(angle_diff, max_steering_angle), -max_steering_angle)

        # Create and send drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 2.0  # constant speed
        drive_msg.drive.steering_angle = steering_angle
        
        self.drive_pub.publish(drive_msg)

    def quaternion_to_yaw(self, q):
        # Converts quaternion to yaw angle
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        # Normalize angle to [-pi, pi]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = StanLee()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


