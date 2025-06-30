#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist

scan_data = None

def scan_callback(msg):
    global scan_data
    scan_data = msg

def send_goal_with_recovery(x, y, yaw_rad):
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base!")

    def send_goal():
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0

        q = quaternion_from_euler(0, 0, yaw_rad)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo("Sending goal: (%.2f, %.2f, %.1f°)" % (x, y, yaw_rad * 180.0 / math.pi))
        client.send_goal(goal)

    def enough_space_for_turn():
        rospy.sleep(0.5)
        global scan_data
        if scan_data is None:
            rospy.logwarn("No scan data available!")
            return False

        min_clear_distance = 1.0  # meters
        angle_range_deg = 90

        angle_min = scan_data.angle_min
        angle_increment = scan_data.angle_increment
        ranges = scan_data.ranges

        total_angles = len(ranges)
        angle_center_index = int((0 - angle_min) / angle_increment)
        angle_half_range = int(math.radians(angle_range_deg) / angle_increment)

        start_index = max(0, angle_center_index - angle_half_range)
        end_index = min(total_angles - 1, angle_center_index + angle_half_range)

        for i in range(start_index, end_index + 1):
            dist = ranges[i]
            if not math.isinf(dist) and dist < min_clear_distance:
                rospy.logwarn("Obstacle detected at index %d: %.2f m" % (i, dist))
                return False

        return True

    def perform_recovery():
        vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        twist = Twist()
        rate = rospy.Rate(10)

        if enough_space_for_turn():
            rospy.logwarn("Performing 180° turn.")
            twist.angular.z = 0.5
            duration = 6.0
        else:
            rospy.logwarn("Not enough space. Reversing for 1 second.")
            twist.linear.x = -0.2
            duration = 1.0

        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
            vel_pub.publish(twist)
            rate.sleep()

        vel_pub.publish(Twist())
        rospy.sleep(1.0)

    # Main goal-retrying loop
    attempt = 0
    while not rospy.is_shutdown():
        attempt += 1
        rospy.loginfo("Attempt #%d: sending goal..." % attempt)
        send_goal()
        success = client.wait_for_result(rospy.Duration(30.0))
        state = client.get_state()

        if success and state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Navigation goal SUCCEEDED after %d attempt(s)." % attempt)
            break
        else:
            rospy.logwarn("Navigation FAILED (state: %d). Performing recovery and retrying..." % state)
            perform_recovery()

    return  # once succeeded

if __name__ == '__main__':
    try:
        rospy.init_node('dual_goal_navigation_node')

        # Get two goals from user
        goals = []
        for i in range(2):
            print("\n--- Enter Goal %d ---" % (i + 1))
            x = float(input("Enter goal X: "))
            y = float(input("Enter goal Y: "))
            yaw_deg = float(input("Enter yaw (in degrees): "))
            yaw_rad = yaw_deg * math.pi / 180.0
            goals.append((x, y, yaw_rad))

        # Send each goal one by one
        for idx, (gx, gy, gyaw) in enumerate(goals):
            rospy.loginfo("Navigating to Goal %d..." % (idx + 1))
            send_goal_with_recovery(gx, gy, gyaw)
            rospy.sleep(1.0)

        rospy.loginfo("All goals completed.")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")

