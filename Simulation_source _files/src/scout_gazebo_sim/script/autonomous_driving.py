#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import cv2
from cv_bridge import CvBridge
import math
import numpy as np
import random
import signal
import sys
import time

class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')

        # Lidar & Camera 
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.bridge = CvBridge()

        # Velocity Publisher 
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Nav2 Client 
        self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Obstacle avoidance parameters 
        self.safe_distance = 1.0
        self.max_speed = 0.4
        self.min_speed = 0.1
        self.max_turn = 0.6
        self.stuck_counter = 0
        self.stuck_threshold = 15
        self.escape_turn = 0.6

        # Lidar ranges 
        self.front = float('inf')
        self.left = float('inf')
        self.right = float('inf')

        # Camera obstacle flag 
        self.object_in_path = False

        # Timer to send Nav2 exploration goals every 10 seconds ---
        self.create_timer(10.0, self.send_random_goal)

        self.get_logger().info("Autonomous Explorer Node Started")
        
        self.shutdown_requested = False


    # Lidar callback   
    def scan_callback(self, msg):
        ranges = msg.ranges
        num_samples = len(ranges)

        front_ranges = [r for r in ranges[num_samples // 3 : 2 * num_samples // 3] if not math.isinf(r)]
        left_ranges = [r for r in ranges[num_samples // 2 : 5 * num_samples // 6] if not math.isinf(r)]
        right_ranges = [r for r in ranges[num_samples // 6 : num_samples // 2] if not math.isinf(r)]

        self.front = min(front_ranges) if front_ranges else float('inf')
        self.left = min(left_ranges) if left_ranges else float('inf')
        self.right = min(right_ranges) if right_ranges else float('inf')

        self.compute_cmd_vel()

    
    # Camera callback  
    def camera_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Detect red objects
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.object_in_path = False
        for cnt in contours:
            if cv2.contourArea(cnt) > 500:
                self.object_in_path = True
                break

    
    # Compute velocity based on obstacles
    def compute_cmd_vel(self):
        if self.shutdown_requested:
            return 
        cmd = Twist()

        # LiDAR obstacle detected
        if self.front < self.safe_distance:
            self.stuck_counter += 1
            if self.stuck_counter > self.stuck_threshold:
                cmd.linear.x = -0.15
                cmd.angular.z = self.escape_turn
            else:
                cmd.linear.x = self.min_speed
                if self.left < self.safe_distance and self.right < self.safe_distance:
                    cmd.angular.z = self.max_turn
                else:
                    turn_dir = self.left - self.right
                    cmd.angular.z = max(min(self.max_turn * turn_dir, self.max_turn), -self.max_turn)

        # Camera detected obstacle
        elif self.object_in_path:
            self.stuck_counter = 0
            cmd.linear.x = 0.2
            cmd.angular.z = self.max_turn * (self.left - self.right)
       
        else:
            self.stuck_counter = 0
            cmd.linear.x = self.max_speed
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

   
    # Send random exploration goal to Nav2  
    def send_random_goal(self):
        if self.shutdown_requested:
            return
        if not self.nav2_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 server not available")
            return

        x = random.uniform(-5.0, 5.0)
        y = random.uniform(-5.0, 5.0)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending Nav2 goal: x={x:.2f}, y={y:.2f}")
        self.nav2_client.send_goal_async(goal_msg)

   
    def stop_robot(self):
        self.get_logger().info("!!! EMERGENCY STOP INITIATED !!!")
        self.shutdown_requested = True
        self.get_logger().warn("SHUTDOWN SEQUENCE STARTING...")

        try:
            self.nav2_client.cancel_all_goals()
            self.get_logger().info("Nav2 goals cancelled.")
        except Exception as e:
            self.get_logger().error(f"Could not cancel Nav2: {e}")

        # 3. The "Hammer" approach: Send 0.0 repeatedly
        stop_msg = Twist()
        for i in range(20):
            self.cmd_pub.publish(stop_msg)
            time.sleep(0.05) 
            
        self.get_logger().info("Robot stopped. Safe to exit.")

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousExplorer()

    def signal_handler(sig, frame):
        node.get_logger().info('Signal received, cleaning up...')
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    # Register the signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Node crashed: {e}")
        node.stop_robot()
    finally:
        if rclpy.ok():
            node.stop_robot()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
