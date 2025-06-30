#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    # Display number of ranges and some example values
    rospy.loginfo("Received scan with %d ranges", len(msg.ranges))
    rospy.loginfo("Front: %.2f m | Left: %.2f m | Right: %.2f m",
                  msg.ranges[len(msg.ranges)//2],    # front
                  msg.ranges[0],                      # left-most
                  msg.ranges[-1])                     # right-most

def main():
    rospy.init_node('lidar_reader', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.loginfo("LiDAR Reader Started. Listening to /scan...")
    rospy.spin()

if __name__ == '__main__':
    main()
