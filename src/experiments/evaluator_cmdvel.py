#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import numpy as np
import csv
import time
import argparse
from collections import deque
import threading

class HybridSLAMTester(Node):
    def __init__(self, test_pattern):
        super().__init__('hybrid_slam_tester')
        
        self.test_pattern = test_pattern
        
        # Data storage for synchronized comparison (same as working script)
        self.gazebo_data = deque(maxlen=10000)
        self.odom_data = deque(maxlen=10000)
        
        # Test control
        self.test_active = False
        self.test_completed = False
        
        # Synchronization parameters
        self.max_time_diff = 0.05  # 50ms maximum time difference for sync
        
        # Robot parameters
        self.robot_name = "gravel_detect"
        self.robot_index = None
        
        # File for data logging
        timestamp = int(time.time())
        self.csv_filename = f"test_{test_pattern}_{timestamp}.csv"
        self.init_csv_file()
        
        # Publishers and Subscribers (same as working script)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.gazebo_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.gazebo_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/lio/odometry',
            self.odom_callback,
            10
        )
        
        # Threading lock for data safety
        self.data_lock = threading.Lock()
        
        # Test patterns
        self.test_patterns = {
            'straight': self.straight_line_test,
            'rotate': self.rotation_test,
            'square': self.square_test,
            'circle': self.circle_test
        }
        
        self.get_logger().info(f"Hybrid SLAM Tester initialized for pattern: {test_pattern}")
        self.get_logger().info(f"Data will be saved to: {self.csv_filename}")
    
    def init_csv_file(self):
        """Initialize CSV file with headers"""
        with open(self.csv_filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                'timestamp', 'gt_x', 'gt_y', 'gt_z', 'gt_qx', 'gt_qy', 'gt_qz', 'gt_qw',
                'odom_x', 'odom_y', 'odom_z', 'odom_qx', 'odom_qy', 'odom_qz', 'odom_qw',
                'pos_error', 'orient_error', 'sync_time_diff'
            ])
    
    def gazebo_callback(self, msg):
        """Store Gazebo ground truth with timestamp (same as working script)"""
        try:
            if self.robot_index is None:
                self.robot_index = msg.name.index(self.robot_name)
                self.get_logger().info(f"Found robot at index {self.robot_index}")
            
            if self.test_active:  # Only collect during test
                timestamp = self.get_clock().now().nanoseconds / 1e9
                pose = msg.pose[self.robot_index]
                
                with self.data_lock:
                    self.gazebo_data.append((timestamp, pose))
                    
        except (ValueError, IndexError) as e:
            if self.robot_index is None:
                available = ", ".join(msg.name)
                self.get_logger().warn(f"Robot '{self.robot_name}' not found. Available: {available}")
    
    def odom_callback(self, msg):
        """Store odometry data with timestamp (same as working script)"""
        if self.test_active:  # Only collect during test
            timestamp = self.get_clock().now().nanoseconds / 1e9
            pose = msg.pose.pose
            
            with self.data_lock:
                self.odom_data.append((timestamp, pose))
    
    def stop_robot(self):
        """Send stop command"""
        twist = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
    
    def straight_line_test(self):
        """Execute straight line trajectory"""
        self.get_logger().info("Executing straight line test (5m at 0.2 m/s)")
        
        # Move forward
        twist = Twist()
        twist.linear.x = 0.2
        
        duration = 25.0  # 5 meters at 0.2 m/s
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)  # Keep spinning to collect data
        
        self.stop_robot()
    
    def rotation_test(self):
        """Execute rotation in place test"""
        self.get_logger().info("Executing rotation test (360° at 0.5 rad/s)")
        
        # Rotate 360 degrees
        twist = Twist()
        twist.angular.z = 0.5
        
        duration = (2 * math.pi) / 0.5  # Full rotation
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        
        self.stop_robot()
    
    def square_test(self):
        """Execute square trajectory"""
        self.get_logger().info("Executing square test (2m sides)")
        
        side_length = 2.0
        linear_speed = 0.2
        angular_speed = 0.3
        
        for side in range(4):
            self.get_logger().info(f"Square side {side + 1}/4")
            
            # Drive straight
            twist = Twist()
            twist.linear.x = linear_speed
            duration = side_length / linear_speed
            start_time = time.time()
            
            while (time.time() - start_time) < duration:
                self.cmd_vel_pub.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.05)
            
            # Stop and turn
            self.stop_robot()
            time.sleep(1.0)
            
            # Turn 90 degrees
            twist = Twist()
            twist.angular.z = angular_speed
            turn_duration = (math.pi / 2) / angular_speed
            start_time = time.time()
            
            while (time.time() - start_time) < turn_duration:
                self.cmd_vel_pub.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.05)
            
            self.stop_robot()
            time.sleep(1.0)
    
    def circle_test(self):
        """Execute circular trajectory"""
        self.get_logger().info("Executing circle test (0.5m radius)")
        
        radius = 0.5
        speed = 0.25
        revolutions = 2.0
        
        angular_velocity = speed / radius
        total_time = (2 * math.pi * revolutions) / angular_velocity
        
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = angular_velocity
        
        start_time = time.time()
        while (time.time() - start_time) < total_time:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        
        self.stop_robot()
    
    def find_synchronized_pairs(self):
        """Find temporally synchronized pose pairs (same as working script)"""
        synchronized_pairs = []
        
        with self.data_lock:
            gazebo_list = list(self.gazebo_data)
            odom_list = list(self.odom_data)
        
        for odom_time, odom_pose in odom_list:
            best_match = None
            min_time_diff = float('inf')
            
            for gazebo_time, gazebo_pose in gazebo_list:
                time_diff = abs(gazebo_time - odom_time)
                
                if time_diff < min_time_diff and time_diff < self.max_time_diff:
                    min_time_diff = time_diff
                    best_match = (gazebo_time, gazebo_pose, time_diff)
            
            if best_match:
                synchronized_pairs.append({
                    'timestamp': odom_time,
                    'gazebo_pose': best_match[1],
                    'odom_pose': odom_pose,
                    'time_diff': best_match[2]
                })
        
        return synchronized_pairs
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw"""
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    
    def calculate_errors(self, gt_pose, odom_pose):
        """Calculate position and orientation errors"""
        # Position error
        pos_error = math.sqrt(
            (gt_pose.position.x - odom_pose.position.x)**2 +
            (gt_pose.position.y - odom_pose.position.y)**2 +
            (gt_pose.position.z - odom_pose.position.z)**2
        )
        
        # Yaw error
        gt_yaw = self.quaternion_to_yaw(gt_pose.orientation)
        odom_yaw = self.quaternion_to_yaw(odom_pose.orientation)
        
        yaw_error = abs(gt_yaw - odom_yaw)
        if yaw_error > math.pi:
            yaw_error = 2 * math.pi - yaw_error
        
        return pos_error, yaw_error
    
    def save_to_csv(self, data_point):
        """Save synchronized data point to CSV"""
        with open(self.csv_filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                data_point['timestamp'],
                data_point['gazebo_pose'].position.x,
                data_point['gazebo_pose'].position.y,
                data_point['gazebo_pose'].position.z,
                data_point['gazebo_pose'].orientation.x,
                data_point['gazebo_pose'].orientation.y,
                data_point['gazebo_pose'].orientation.z,
                data_point['gazebo_pose'].orientation.w,
                data_point['odom_pose'].position.x,
                data_point['odom_pose'].position.y,
                data_point['odom_pose'].position.z,
                data_point['odom_pose'].orientation.x,
                data_point['odom_pose'].orientation.y,
                data_point['odom_pose'].orientation.z,
                data_point['odom_pose'].orientation.w,
                data_point['pos_error'],
                data_point['orient_error'],
                data_point['time_diff']
            ])
    
    def run_test(self):
        """Execute the test pattern"""
        if self.test_pattern not in self.test_patterns:
            self.get_logger().error(f"Unknown test pattern: {self.test_pattern}")
            return
        
        # Wait for robot to be found
        self.get_logger().info("Waiting for robot in Gazebo...")
        while self.robot_index is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info("Starting test in 3 seconds...")
        time.sleep(3.0)
        
        # Start data collection
        self.test_active = True
        test_start_time = time.time()
        
        # Execute test pattern
        self.test_patterns[self.test_pattern]()
        
        # Stop data collection
        self.test_active = False
        test_duration = time.time() - test_start_time
        
        self.get_logger().info(f"Test completed in {test_duration:.1f} seconds")
        
        # Wait a moment for final data
        time.sleep(2.0)
        
        # Process results
        synchronized_pairs = self.find_synchronized_pairs()
        
        if len(synchronized_pairs) < 10:
            self.get_logger().error(f"Insufficient synchronized data: {len(synchronized_pairs)} pairs")
            return
        
        self.get_logger().info(f"Processing {len(synchronized_pairs)} synchronized measurements...")
        
        # Calculate errors and save to CSV
        position_errors = []
        orientation_errors = []
        
        for pair in synchronized_pairs:
            pos_error, orient_error = self.calculate_errors(pair['gazebo_pose'], pair['odom_pose'])
            pair['pos_error'] = pos_error
            pair['orient_error'] = orient_error
            
            position_errors.append(pos_error)
            orientation_errors.append(orient_error)
            
            self.save_to_csv(pair)
        
        # Calculate final statistics
        pos_errors = np.array(position_errors)
        orient_errors = np.array(orientation_errors)
        
        # Print results
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"TEST RESULTS - {self.test_pattern.upper()}")
        self.get_logger().info(f"{'='*60}")
        self.get_logger().info(f"Sample count: {len(synchronized_pairs)}")
        self.get_logger().info(f"Duration: {test_duration:.1f}s")
        self.get_logger().info(f"Position RMSE: {np.sqrt(np.mean(pos_errors**2))*1000:.1f} mm")
        self.get_logger().info(f"Position Mean: {np.mean(pos_errors)*1000:.1f} ± {np.std(pos_errors)*1000:.1f} mm")
        self.get_logger().info(f"Orientation RMSE: {math.degrees(np.sqrt(np.mean(orient_errors**2))):.2f}°")
        self.get_logger().info(f"Orientation Mean: {math.degrees(np.mean(orient_errors)):.2f} ± {math.degrees(np.std(orient_errors)):.2f}°")
        self.get_logger().info(f"Final position error: {pos_errors[-1]*1000:.1f} mm")
        self.get_logger().info(f"Max position error: {np.max(pos_errors)*1000:.1f} mm")
        self.get_logger().info(f"Data saved to: {self.csv_filename}")
        self.get_logger().info(f"{'='*60}")
        
        self.test_completed = True

def main():
    parser = argparse.ArgumentParser(description='Hybrid SLAM Testing')
    parser.add_argument('pattern', choices=['straight', 'rotate', 'square', 'circle'],
                       help='Test pattern to execute')
    
    args = parser.parse_args()
    
    rclpy.init()
    tester = HybridSLAMTester(args.pattern)
    
    try:
        tester.run_test()
        
        # Keep spinning until test is complete
        while not tester.test_completed:
            rclpy.spin_once(tester, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        tester.get_logger().info("Test interrupted")
        tester.stop_robot()
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()