#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String

class ObjectClassifier:
    def __init__(self):
        rospy.init_node('object_classifier_node')

        # Subscribers
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/camera/depth/points', PointCloud2, self.depth_callback)

        # Publisher
        self.pub = rospy.Publisher('/object_type', String, queue_size=10)

        # Data storage
        self.latest_lidar_ranges = None
        self.latest_depth_points = []

        rospy.spin()

    def lidar_callback(self, msg):
        self.latest_lidar_ranges = np.array(msg.ranges)
        self.analyze_scene()

    def depth_callback(self, msg):
        self.latest_depth_points = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            self.latest_depth_points.append(point)
        self.analyze_scene()

    def analyze_scene(self):
        if self.latest_lidar_ranges is None or len(self.latest_depth_points) == 0:
            return

        # Filter forward-facing depth points (±20cm left/right, 0.3–3.0m forward)
        forward_points = [p for p in self.latest_depth_points if abs(p[0]) < 0.2 and 0.3 < p[2] < 3.0]
        if len(forward_points) < 10:
            return

        # Extract height and forward distance
        heights = np.array([p[1] for p in forward_points])  # vertical
        depths = np.array([p[2] for p in forward_points])   # forward

        try:
            # Fit linear slope (height vs depth)
            slope, _ = np.polyfit(depths, heights, 1)
        except Exception as e:
            rospy.logwarn("Depth fit failed: %s", str(e))
            return

        # Extract center LiDAR range: ±5 degrees around front
        center_idx = len(self.latest_lidar_ranges) // 2
        lidar_window = self.latest_lidar_ranges[center_idx - 5:center_idx + 5]

        # Python 2 fix: isfinite() not available
        lidar_valid = [r for r in lidar_window if not math.isnan(r) and not math.isinf(r)]
        if not lidar_valid:
            return
        lidar_avg = np.mean(lidar_valid)

        # Classification logic
        if abs(slope) < 0.1 and lidar_avg < 0.5:
            label = "Wall"
        elif abs(slope) < 0.1 and 0.5 <= lidar_avg <= 2.0:
            label = "Obstacle"
        elif 0.1 <= abs(slope) <= 0.5 and lidar_avg > 0.5:
            label = "Ramp"
        else:
            label = "Unknown"

        rospy.loginfo("Object: %s | Slope: %.2f | LiDAR Avg: %.2f", label, slope, lidar_avg)
        self.pub.publish(label)

if __name__ == '__main__':
    try:
        ObjectClassifier()
    except rospy.ROSInterruptException:
        pass

