#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

scan_data = None
pose_history = []

def scan_callback(msg):
    global scan_data
    scan_data = msg

def odom_callback(msg):
    global pose_history
    pose = msg.pose.pose
    pose_history.append((pose.position.x, pose.position.y))
    if len(pose_history) > 100:  # Keep recent 100 poses
        pose_history.pop(0)

def get_current_pose():
    listener = tf.TransformListener()
    rospy.loginfo("Waiting for transform from map to base_link...")
    listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(5.0))
    (trans, rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))
    return trans  # [x, y, z]

def compute_yaw_to_goal(current_x, current_y, goal_x, goal_y):
    dx = goal_x - current_x
    dy = goal_y - current_y
    return math.atan2(dy, dx)

def send_goal_with_recovery(x, y):
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Wait until initial scan and odometry are ready
    rospy.loginfo("Waiting for initial scan and odom data...")
    rate = rospy.Rate(10)
    while scan_data is None or len(pose_history) < 2:
        rate.sleep()
    rospy.loginfo("Sensor data received.")

    current_x, current_y, _ = get_current_pose()
    yaw_rad = compute_yaw_to_goal(current_x, current_y, x, y)

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
        rate = rospy.Rate(10)

        if enough_space_for_turn():
            rospy.logwarn("Performing 180° turn.")
            twist = Twist()
            twist.angular.z = 0.5
            duration = 6.0
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
                vel_pub.publish(twist)
                rate.sleep()
        else:
            rospy.logwarn("Not enough space. Reversing along path for 1 second.")
            duration = 1.0
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
                if len(pose_history) < 2:
                    twist = Twist()
                    twist.linear.x = -0.2
                else:
                    current = pose_history[-1]
                    previous = pose_history[-2]
                    dx = current[0] - previous[0]
                    dy = current[1] - previous[1]
                    angle = math.atan2(dy, dx)

                    twist = Twist()
                    twist.linear.x = -0.2
                    twist.angular.z = 0.0

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

    return

if __name__ == '__main__':
    try:
        rospy.init_node('dual_goal_navigation_node')

        # Get two goals from user (yaw auto-calculated)
        goals = []
        for i in range(2):
            print("\n--- Enter Goal %d ---" % (i + 1))
            x = float(input("Enter goal X: "))
            y = float(input("Enter goal Y: "))
            goals.append((x, y))

        # Navigate to each goal
        for idx, (gx, gy) in enumerate(goals):
            rospy.loginfo("Navigating to Goal %d..." % (idx + 1))
            send_goal_with_recovery(gx, gy)
            rospy.sleep(1.0)

        rospy.loginfo("All goals completed.")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")

