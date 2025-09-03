#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import math

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Predefined waypoints (x, y)
        self.waypoints = np.array([
            [0.10201684724816548, -0.38692357074507777],
[0.20317270938404372, -0.7740540405443925],
[0.30339912716175754, -1.1614074232566218],
[0.4026274811968195, -1.548999412463284],
[0.5008106106754115, -1.936840817518988],
[0.5982083404547124, -2.3248690242362713],
[0.6953098939176092, -2.712966410823081],
[0.7926098590896348, -3.1010143145865494],
[0.8906031442734921, -3.48889383262593],
[0.9897843374947249, -3.876486222179059],
[1.090648347056042, -4.263672500275894],
[1.1936080903036803, -4.650353461061872],
[1.2984906976331108, -5.036569139183711],
[1.4048677983244682, -5.422420582089724],
[1.5123096604799036, -5.808008917297518],
[1.6203867924094297, -6.193435432463278],
[1.7286696223537914, -6.578801335173911],
[1.8367284984844194, -6.964207913085606],
[1.9442186424237942, -7.34973683687767],
[2.0511259619762665, -7.735393230984881],
[2.1575169146553943, -8.121163964043115],
[2.2634581181133004, -8.507035504341776],
[2.3690161900021316, -8.892994640447458],
[2.474257747974022, -9.279027840649562],
[2.579249329611821, -9.665121893514678],
[2.6840575525676518, -10.051263347401504],
[2.7887489544243635, -10.43743891080733],
        ])
        self.current_idx = 0
        self.threshold = 0.5  # distance to switch to next waypoint

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        # Get current position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Get target waypoint
        target = self.waypoints[self.current_idx]

        # Compute vector to target
        dx = target[0] - x
        dy = target[1] - y
        distance = math.hypot(dx, dy)

        # Switch to next waypoint if close enough
        if distance < self.threshold and self.current_idx < len(self.waypoints) - 1:
            self.current_idx += 1
            self.get_logger().info(f"Switching to waypoint {self.current_idx}")

        # Calculate heading to target
        heading = math.atan2(dy, dx)

        # For simplicity, set steering angle proportional to heading difference
        # Assuming car heading aligns with x-axis (can be improved by reading orientation)
        steering_angle = max(min(heading, 0.4189), -0.4189)  # limit steering to +/-24 degrees

        # Publish drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 2.0         # fixed speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)

    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

