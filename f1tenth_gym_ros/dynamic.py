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
        self.waypoints = np.loadtxt('/sim_ws/src/f1tenth_gym_ros/maps/SaoPaulo_raceline.csv', delimiter=',', skiprows=1, usecols=(1,2))

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        self.current_index = 0
        self.distance_threshold = 0.3

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q)

        # Find closest waypoint index
        distances = np.linalg.norm(self.waypoints - np.array([x, y]), axis=1)
        closest_index = np.argmin(distances)
        closest_distance = distances[closest_index]

        # If the closest waypoint is significantly far, jump to closest waypoint
        if closest_distance > self.distance_threshold * 3:
            self.current_index = closest_index
            self.get_logger().info(f"Recovery: Jumping to closest waypoint {closest_index}")
        else:
            # Normal increment logic with waypoint switch if close enough
            target = self.waypoints[self.current_index]
            dx = target[0] - x
            dy = target[1] - y
            distance = math.hypot(dx, dy)

            if distance < self.distance_threshold and self.current_index < len(self.waypoints):
                self.current_index += 1
                if self.current_index >= len(self.waypoints):
                    self.current_index = 0
                    self.get_logger().info("Completed a lap! Starting lap again.")
                else:
                    self.get_logger().info(f"Switching to waypoint {self.current_index}")

        target_yaw = math.atan2(dy, dx)
        heading_error = self.normalize_angle(target_yaw - yaw)
        cross_track_error = math.sin(heading_error) * distance

        k = 1
        max_speed = 9.2  # max speed
        min_speed = 6.0  # min speed
        max_steering_angle = 0.4189

        steering_angle_unclipped = heading_error + math.atan2(k * cross_track_error, max_speed)
        steering_angle = max(min(steering_angle_unclipped, max_steering_angle), -max_steering_angle)

        speed = max_speed * (1 - abs(steering_angle) / max_steering_angle)
        speed = max(speed, min_speed)  # limit to min_speed

        steering_angle = heading_error + math.atan2(k * cross_track_error, speed)
        steering_angle = max(min(steering_angle, max_steering_angle), -max_steering_angle)

        self.get_logger().info(f"Speed:{speed} Steering_angle: {steering_angle}")

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)

    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
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

