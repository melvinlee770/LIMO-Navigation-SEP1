#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
import tf
import random
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler

scan_data = None
pose_history = []
listener = None
cmd_vel_pub = None

def scan_callback(msg):
    global scan_data
    scan_data = msg

def odom_callback(msg):
    global pose_history
    pose = msg.pose.pose
    pose_history.append((pose.position.x, pose.position.y))
    if len(pose_history) > 100:
        pose_history.pop(0)

def euclid(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def get_arena_centers():
    centers = {}
    x, y = map(float, raw_input("Enter x and y for home 'N' (e.g. '1.0 1.0'): ").split())
    centers['N'] = (x, y, 0.0)
    for i in range(1, 9):
        x, y = map(float, raw_input("Enter x and y for arena {} (e.g. '2.0 2.0'): ".format(i)).split())
        centers[i] = (x, y, 0.0)
    return centers

def get_chosen_arenas():
    while True:
        entries = raw_input("Enter two arena IDs to visit, separated by space (e.g. '3 7'): ").split()
        if len(entries) == 2:
            try:
                a1, a2 = int(entries[0]), int(entries[1])
                if a1 in range(1, 9) and a2 in range(1, 9) and a1 != a2:
                    return [a1, a2]
            except ValueError:
                pass
        print("Invalid input. Please enter two distinct numbers from 1 to 8.")

def pick_and_order(centers, chosen):
    P0 = centers['N']
    P1 = centers[chosen[0]]
    P2 = centers[chosen[1]]
    tour_A = euclid(P0, P1) + euclid(P1, P2)
    tour_B = euclid(P0, P2) + euclid(P2, P1)
    return chosen if tour_A <= tour_B else list(reversed(chosen))

def get_current_pose():
    if listener is None:
        rospy.logerr("TF listener not initialized in get_current_pose!")
        return None, None, None
    rospy.loginfo("Waiting for transform from map to base_link...")
    listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(5.0))
    (trans, rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))
    return trans

def send_goal_with_recovery(x, y):
    rospy.loginfo("Waiting for initial scan and odom data for this goal...")
    rate = rospy.Rate(10)
    while scan_data is None or len(pose_history) < 2:
        rate.sleep()
    rospy.loginfo("Sensor data received.")

    current_x, current_y, _ = get_current_pose()
    if current_x is None:
        rospy.logwarn("Failed to get current pose for goal calculation. Skipping this goal.")
        return

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
        q = quaternion_from_euler(0, 0, 0)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        rospy.loginfo("Sending goal: (%.2f, %.2f)" % (x, y))
        client.send_goal(goal)

    attempt = 0
    while not rospy.is_shutdown():
        attempt += 1
        rospy.loginfo("Attempt #%d: sending goal..." % attempt)
        send_goal()
        success = client.wait_for_result(rospy.Duration(30.0))
        state = client.get_state()
        if success and state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Navigation goal SUCCEEDED after %d attempt(s)." % attempt)
            break
        else:
            rospy.logwarn("Navigation FAILED (state: %d). Retrying..." % state)

if __name__ == '__main__':
    try:
        rospy.init_node('dual_goal_navigation_node')
        rospy.Subscriber('/scan', LaserScan, scan_callback)
        rospy.Subscriber('/odom', Odometry, odom_callback)
        listener = tf.TransformListener()
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.loginfo("Waiting for initial sensor data (scan and odom)...")
        initial_rate = rospy.Rate(10)
        while scan_data is None or len(pose_history) < 2:
            initial_rate.sleep()
        rospy.loginfo("Initial sensor data received.")

        centers = get_arena_centers()
        chosen = get_chosen_arenas()
        visit_order = pick_and_order(centers, chosen)

        for idx, arena_id in enumerate(visit_order):
            x, y, _ = centers[arena_id]
            rospy.loginfo("Navigating to Arena %d center (%.2f, %.2f)..." % (arena_id, x, y))
            send_goal_with_recovery(x, y)
            rospy.sleep(1.0)

        rospy.loginfo("All goals completed.")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")

