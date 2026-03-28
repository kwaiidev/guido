#!/usr/bin/env python3

import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from tf2_ros import Buffer, TransformListener

class SystemValidator(Node):
    def __init__(self):
        super().__init__('system_validator')
        
        # Track when we last saw a message
        self.last_scan = 0.0
        self.last_odom = 0.0
        self.last_map = 0.0
        
        # Use common topic names that might be mismatched
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(LaserScan, '/ldlidar_node/scan', self._scan_cb, 10)
        
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Odometry, '/guido/odom', self._odom_cb, 10)
        
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _scan_cb(self, msg: LaserScan):
        self.last_scan = time.time()

    def _odom_cb(self, msg: Odometry):
        self.last_odom = time.time()

    def _map_cb(self, msg: OccupancyGrid):
        self.last_map = time.time()

    def run_check(self):
        self.get_logger().info("Listening to sensor topics for 5 seconds...")
        
        start_time = time.time()
        while time.time() - start_time < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        now = time.time()
        
        print("\n" + "="*50)
        print("          SYSTEM VALIDATION REPORT")
        print("="*50)
        
        # Check LiDAR
        if self.last_scan > 0 and (now - self.last_scan) < 2.0:
            print("[OK] LiDAR scan data is arriving actively.")
        elif self.last_scan > 0:
            print("[FAIL] LiDAR scan data was seen, but is STALE (stopped publishing).")
        else:
            print("[FAIL] NO LiDAR scan data received on /scan or /ldlidar_node/scan.")

        # Check Odometry
        if self.last_odom > 0 and (now - self.last_odom) < 2.0:
            print("[OK] Odometry data is arriving actively.")
        else:
            print("[FAIL] NO Odometry data received on /odom.")

        # Check Map
        if self.last_map > 0:
            print("[OK] Map is being published.")
        else:
            print("[FAIL] NO Map data. (Usually fails if LiDAR or TF is missing).")

        # Check Critical TF (map -> odom -> base_link)
        try:
            self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            print("[OK] TF Transform: map -> odom exists.")
        except Exception as e:
            print(f"[FAIL] Missing TF Transform map -> odom: {e}")
            
        try:
            self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            print("[OK] TF Transform: odom -> base_link exists.")
        except Exception as e:
            print(f"[FAIL] Missing TF Transform odom -> base_link: {e}")

        print("="*50 + "\n")

def main():
    rclpy.init()
    validator = SystemValidator()
    try:
        validator.run_check()
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
