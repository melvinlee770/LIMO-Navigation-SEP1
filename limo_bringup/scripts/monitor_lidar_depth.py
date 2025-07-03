#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class LidarSideMonitor:
    def __init__(self):
        rospy.init_node('lidar_side_monitor')

        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

        # Publisher for human-readable output
        self.pub = rospy.Publisher('/lidar_side_data', String, queue_size=10)

        rospy.spin()

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        num_ranges = len(ranges)

        # Convert degrees to index
        def get_index(deg):
            rad = math.radians(deg)
            idx = int((rad - angle_min) / angle_increment)
            return max(0, min(idx, num_ranges - 1))

        # Indices for front (0°), left (90°), right (-90°)
        idx_front = get_index(0)
        idx_left = get_index(90)
        idx_right = get_index(-90)

        def safe_value(val):
            return round(val, 2) if not math.isnan(val) and not math.isinf(val) else -1

        dist_front = safe_value(ranges[idx_front])
        dist_left = safe_value(ranges[idx_left])
        dist_right = safe_value(ranges[idx_right])

        # Log the values
        rospy.loginfo("LIDAR - Front: %.2f m | Left: %.2f m | Right: %.2f m",
                      dist_front, dist_left, dist_right)

        status = "Front: %.2f m | Left: %.2f m | Right: %.2f m" % (
            dist_front, dist_left, dist_right)
        self.pub.publish(status)

if __name__ == '__main__':
    try:
        LidarSideMonitor()
    except rospy.ROSInterruptException:
        pass

