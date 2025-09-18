#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
import math
import numpy as np
import csv
import time
from collections import deque
from scipy.spatial.distance import euclidean
from scipy import interpolate
import threading

class ScientificOdometryEvaluator(Node):
    def __init__(self):
        super().__init__('scientific_odometry_evaluator')
        
        # Data storage for synchronized comparison
        self.gazebo_data = deque(maxlen=10000)  # (timestamp, pose)
        self.odom_data = deque(maxlen=10000)    # (timestamp, pose)
        
        # Synchronization parameters
        self.max_time_diff = 0.05  # 50ms maximum time difference for sync
        
        # Statistics storage
        self.position_errors = []
        self.orientation_errors = []
        self.timestamps = []
        
        # Robot parameters
        self.robot_name = "gravel_detect"
        self.robot_index = None
        
        # File for data logging
        self.csv_filename = f"odometry_evaluation_{int(time.time())}.csv"
        self.init_csv_file()
        
        # Subscribers
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
        
        # Timer for analysis and reporting
        self.analysis_timer = self.create_timer(5.0, self.perform_analysis)
        
        # Threading lock for data safety
        self.data_lock = threading.Lock()
        
        self.get_logger().info("Scientific Odometry Evaluator initialized")
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
        """Store Gazebo ground truth with timestamp"""
        try:
            if self.robot_index is None:
                self.robot_index = msg.name.index(self.robot_name)
                self.get_logger().info(f"Found robot at index {self.robot_index}")
            
            timestamp = self.get_clock().now().nanoseconds / 1e9
            pose = msg.pose[self.robot_index]
            
            with self.data_lock:
                self.gazebo_data.append((timestamp, pose))
                
        except (ValueError, IndexError) as e:
            if len(msg.name) > 0:
                available = ", ".join(msg.name)
                self.get_logger().warn(f"Robot '{self.robot_name}' not found. Available: {available}")
    
    def odom_callback(self, msg):
        """Store odometry data with timestamp"""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        pose = msg.pose.pose
        
        with self.data_lock:
            self.odom_data.append((timestamp, pose))
    
    def find_synchronized_pairs(self):
        """Find temporally synchronized pose pairs"""
        synchronized_pairs = []
        
        with self.data_lock:
            gazebo_list = list(self.gazebo_data)
            odom_list = list(self.odom_data)
        
        for odom_time, odom_pose in odom_list:
            # Find closest Gazebo measurement in time
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
    
    def quaternion_to_euler(self, q):
        """Convert quaternion to roll, pitch, yaw"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def calculate_errors(self, gt_pose, odom_pose):
        """Calculate position and orientation errors"""
        # Position error (Euclidean distance)
        pos_error = math.sqrt(
            (gt_pose.position.x - odom_pose.position.x)**2 +
            (gt_pose.position.y - odom_pose.position.y)**2 +
            (gt_pose.position.z - odom_pose.position.z)**2
        )
        
        # Orientation error (angular difference)
        gt_roll, gt_pitch, gt_yaw = self.quaternion_to_euler(gt_pose.orientation)
        odom_roll, odom_pitch, odom_yaw = self.quaternion_to_euler(odom_pose.orientation)
        
        # Calculate yaw error (most important for 2D navigation)
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
    
    def perform_analysis(self):
        """Perform statistical analysis and report results"""
        synchronized_pairs = self.find_synchronized_pairs()
        
        if len(synchronized_pairs) < 10:
            self.get_logger().info(f"Collected {len(synchronized_pairs)} synchronized measurements (need ≥10 for analysis)")
            return
        
        # Process new data points
        new_data_count = 0
        for pair in synchronized_pairs:
            if pair['timestamp'] not in self.timestamps:
                pos_error, orient_error = self.calculate_errors(pair['gazebo_pose'], pair['odom_pose'])
                
                pair['pos_error'] = pos_error
                pair['orient_error'] = orient_error
                
                self.position_errors.append(pos_error)
                self.orientation_errors.append(orient_error)
                self.timestamps.append(pair['timestamp'])
                
                self.save_to_csv(pair)
                new_data_count += 1
        
        if new_data_count == 0:
            return
        
        # Statistical analysis
        pos_errors = np.array(self.position_errors)
        orient_errors = np.array(self.orientation_errors)
        
        # Calculate statistics
        stats = {
            'sample_size': len(pos_errors),
            'pos_mean': np.mean(pos_errors),
            'pos_std': np.std(pos_errors),
            'pos_rmse': np.sqrt(np.mean(pos_errors**2)),
            'pos_max': np.max(pos_errors),
            'pos_median': np.median(pos_errors),
            'orient_mean': np.mean(orient_errors),
            'orient_std': np.std(orient_errors),
            'orient_rmse': np.sqrt(np.mean(orient_errors**2)),
            'orient_max': np.max(orient_errors),
            'orient_median': np.median(orient_errors)
        }
        
        # Report results
        self.get_logger().info(f"\n{'='*80}")
        self.get_logger().info("SCIENTIFIC ODOMETRY EVALUATION RESULTS")
        self.get_logger().info(f"{'='*80}")
        self.get_logger().info(f"Sample Size: {stats['sample_size']} synchronized measurements")
        self.get_logger().info(f"Time Period: {(max(self.timestamps) - min(self.timestamps)):.1f} seconds")
        self.get_logger().info(f"\nPOSITION ERROR STATISTICS:")
        self.get_logger().info(f"  Mean Error:    {stats['pos_mean']:.4f} m")
        self.get_logger().info(f"  Std Deviation: {stats['pos_std']:.4f} m")
        self.get_logger().info(f"  RMSE:          {stats['pos_rmse']:.4f} m")
        self.get_logger().info(f"  Max Error:     {stats['pos_max']:.4f} m")
        self.get_logger().info(f"  Median Error:  {stats['pos_median']:.4f} m")
        self.get_logger().info(f"\nORIENTATION ERROR STATISTICS:")
        self.get_logger().info(f"  Mean Error:    {math.degrees(stats['orient_mean']):.2f}°")
        self.get_logger().info(f"  Std Deviation: {math.degrees(stats['orient_std']):.2f}°")
        self.get_logger().info(f"  RMSE:          {math.degrees(stats['orient_rmse']):.2f}°")
        self.get_logger().info(f"  Max Error:     {math.degrees(stats['orient_max']):.2f}°")
        self.get_logger().info(f"  Median Error:  {math.degrees(stats['orient_median']):.2f}°")
        self.get_logger().info(f"{'='*80}")

def main(args=None):
    rclpy.init(args=args)
    evaluator = ScientificOdometryEvaluator()
    
    try:
        rclpy.spin(evaluator)
    except KeyboardInterrupt:
        pass
    
    evaluator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()