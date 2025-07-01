#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2

class EnvironmentClassifier:
    def __init__(self):
        rospy.init_node('lidar_depth_classifier_node')
        self.bridge = CvBridge()
        self.latest_scan = None

        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)

        rospy.loginfo(" Monitoring environment using LiDAR and depth camera...")

    def lidar_callback(self, msg):
        self.latest_scan = msg

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            rospy.logerr("Depth image conversion failed: %s", str(e))
            return

        if depth_image is None or self.latest_scan is None:
            rospy.logwarn_throttle(1.0, "Waiting for both depth and LiDAR data...")
            return

        h, w = depth_image.shape
        center_col = w // 2

        # --- 1. DEPTH CAMERA SLOPE FITTING ---
        col_depths = depth_image[:, center_col]
        sample_rows = np.linspace(0, h - 1, 20).astype(int)

        valid_indices = []
        valid_values = []

        for row in sample_rows:
            d = col_depths[row]
            if np.isfinite(d) and d > 0:
                valid_indices.append(row)
                valid_values.append(d)

        if len(valid_values) < 10:
            rospy.logwarn_throttle(1.0, "⚠️ Not enough valid depth data for slope estimation")
            return

        m, _ = np.polyfit(valid_indices, valid_values, 1)
        slope_deg = math.degrees(math.atan(m))

        if abs(slope_deg) > 45:
            rospy.logwarn_throttle(1.0, "⚠️ Slope %.2f° too steep — ignoring this reading" % slope_deg)
            return

        # --- 2. LIDAR DATA ANALYSIS ---
        scan = self.latest_scan
        ranges = np.array(scan.ranges)
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment

        center_angle = 0.0
        center_index = int(round((center_angle - angle_min) / angle_increment))

        # Sample ±20 degrees around center
        spread_indices = int(math.radians(20.0) / angle_increment)
        start_idx = max(0, center_index - spread_indices)
        end_idx = min(len(ranges) - 1, center_index + spread_indices)

        lidar_window = ranges[start_idx:end_idx + 1]
        lidar_window = [d for d in lidar_window if not math.isinf(d) and not math.isnan(d) and d > 0]

        if len(lidar_window) < 5:
            rospy.logwarn_throttle(1.0, " Not enough valid LiDAR readings for classification")
            return

        lidar_mean = np.mean(lidar_window)
        lidar_min = np.min(lidar_window)
        lidar_max = np.max(lidar_window)
        lidar_spread = lidar_max - lidar_min

        # --- 3. CLASSIFICATION LOGIC (Updated thresholds + fallback) ---
        result = ""
        if abs(slope_deg) < 3:
            if lidar_spread < 0.15:
                result = "OBSTACLE detected (flat depth, narrow lidar)"
            else:
                result = " WALL detected (flat depth, wide lidar)"
        else:
            if lidar_spread > 0.15:
                result = "RAMP detected (slope: %.2f°, lidar spread: %.2f)" % (slope_deg, lidar_spread)
            elif lidar_spread == 0.0 and lidar_mean > 0.3:
                result = "RAMP detected (slope: %.2f°, lidar flat but valid range)" % slope_deg
            else:
                result = "OBSTACLE detected (sloped depth, but narrow lidar)"

        # --- Log Result ---
        rospy.loginfo_throttle(1.0, "[Depth] Slope = %.2f°, [LiDAR] Spread = %.2f m, Mean = %.2f m" %
                               (slope_deg, lidar_spread, lidar_mean))
        rospy.loginfo_throttle(1.0, "[Classification] %s", result)

if __name__ == '__main__':
    try:
        EnvironmentClassifier()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

