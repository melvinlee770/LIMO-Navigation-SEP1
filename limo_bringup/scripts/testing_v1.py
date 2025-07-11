#!/usr/bin/env python
# -*- coding: utf-8 -*-

from std_srvs.srv import Empty
import rospy
import actionlib
import math
import tf
import random
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
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

def get_footprint_length():
    try:
        footprint = rospy.get_param("/move_base/local_costmap/footprint")
        if isinstance(footprint, str):
            import ast
            footprint = ast.literal_eval(footprint)
        xs = [point[0] for point in footprint]
        length = max(xs) - min(xs)
        return abs(length)
    except (rospy.ROSException, ValueError, SyntaxError) as e:
        rospy.logwarn("Could not get robot footprint. Using default length 0.4m. Error: %s", str(e))
        return 0.4

def get_current_pose():
    if listener is None: # Safety check (though main should initialize it)
        rospy.logerr("TF listener not initialized in get_current_pose!")
        return None, None, None # Indicate failure
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
    if current_x is None: # ADDED: Check for TF lookup failure
        rospy.logwarn("Failed to get current pose for goal calculation. Skipping this goal.")
        return # Exit if we can't get current pose

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

        q = quaternion_from_euler(0, 0, 0)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo("Sending goal: (%.2f, %.2f)" % (x, y))
        client.send_goal(goal)

    def perform_recovery_with_strategy(attempt):
        case = ((attempt - 1) % 4) + 1

        if case == 1:
            rospy.logwarn("Recovery Step 1 (cycled): Clear costmap and retry.")
            try:
                rospy.wait_for_service('/move_base/clear_costmaps', timeout=2.0)
                clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
                clear_srv()
                rospy.loginfo("Costmap cleared.")
                rospy.sleep(3.0)
            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logerr("Costmap clear failed: %s", str(e))
            return

        elif case == 2:
            rospy.logwarn("Recovery Step 2 (cycled): Perform Random Jitter with LIDAR clearance check.")

            if scan_data is None:
                rospy.logwarn("No scan data available! Skipping jitter.")
                return

            angle_min = scan_data.angle_min
            angle_increment = scan_data.angle_increment
            ranges = scan_data.ranges

            def get_index_for_angle(deg):
                angle_rad = math.radians(deg)
                return int((angle_rad - angle_min) / angle_increment)

            front_index = get_index_for_angle(0)
            if 0 <= front_index < len(ranges):
                dist = ranges[front_index]
                if math.isinf(dist) or math.isnan(dist) or dist < 0.10:
                    rospy.logwarn("Front too close (%.2fm)! Skipping jitter." % dist)
                    return

            twist = Twist()
            twist.linear.x = random.uniform(-0.1, 0.1)
            twist.angular.z = random.uniform(-0.4, 0.4)

            rospy.loginfo("Jittering with linear.x: %.3f, angular.z: %.3f",
                          twist.linear.x, twist.angular.z)

            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < 0.6 and not rospy.is_shutdown():
                cmd_vel_pub.publish(twist) # Use global publisher
                rate.sleep() # Reusing 'rate' defined at the top of send_goal_with_recovery
            cmd_vel_pub.publish(Twist()) # Stop the robot
            rospy.sleep(1.0)

        elif case == 3:
            rospy.logwarn("Recovery Step 3 (cycled): Temporary nearby goal + return to previous pose")
            try:
                current_x, current_y, _ = get_current_pose()
  		if current_x is None:
                    rospy.logwarn("Could not get current pose for recovery step 3.")
                    return
                offset = 0.05
                temp_goal_x = current_x + offset
                temp_goal_y = current_y

                temp_goal = MoveBaseGoal()
                temp_goal.target_pose.header.frame_id = "map"
                temp_goal.target_pose.header.stamp = rospy.Time.now()
                temp_goal.target_pose.pose.position.x = temp_goal_x
                temp_goal.target_pose.pose.position.y = temp_goal_y
                q = quaternion_from_euler(0, 0, 0)
                temp_goal.target_pose.pose.orientation.x = q[0]
                temp_goal.target_pose.pose.orientation.y = q[1]
                temp_goal.target_pose.pose.orientation.z = q[2]
                temp_goal.target_pose.pose.orientation.w = q[3]

                client.send_goal(temp_goal)
                success = client.wait_for_result(rospy.Duration(10.0))
                rospy.loginfo("Temporary goal result: %s", str(success))

                if len(pose_history) >= 5:
                    prev_x, prev_y = pose_history[-5]
                    rospy.loginfo("Returning to previous pose (%.2f, %.2f)", prev_x, prev_y)

                    return_goal = MoveBaseGoal()
                    return_goal.target_pose.header.frame_id = "map"
                    return_goal.target_pose.header.stamp = rospy.Time.now()
                    return_goal.target_pose.pose.position.x = prev_x
                    return_goal.target_pose.pose.position.y = prev_y
                    q = quaternion_from_euler(0, 0, 0)
                    return_goal.target_pose.pose.orientation.x = q[0]
                    return_goal.target_pose.pose.orientation.y = q[1]
                    return_goal.target_pose.pose.orientation.z = q[2]
                    return_goal.target_pose.pose.orientation.w = q[3]

                    client.send_goal(return_goal)
                    client.wait_for_result(rospy.Duration(10.0))
                else:
                    rospy.logwarn("Not enough pose history to return to previous pose.")
            except Exception as e:
                rospy.logerr("Error during Recovery Step 3: %s", str(e))

	elif case == 4:
    		rospy.logwarn("Recovery Step 4 (cycled): Reverse for 0.3 seconds.")
    		twist = Twist()
            	twist.linear.x = -0.2  # reverse speed

            	duration = 0.3  # reverse for exactly 0.3 seconds

            	start_time = rospy.Time.now()
            	while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
                	cmd_vel_pub.publish(twist) # Use global publisher
                	rate.sleep() # Reusing 'rate' defined at the top of send_goal_with_recovery
            		cmd_vel_pub.publish(Twist())  # stop the robot
    		rospy.sleep(1.0)


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
            perform_recovery_with_strategy(attempt)

if __name__ == '__main__':
    try:
        rospy.init_node('dual_goal_navigation_node')
	rospy.Subscriber('/scan', LaserScan, scan_callback)
    	rospy.Subscriber('/odom', Odometry, odom_callback)
	listener = tf.TransformListener()
	cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) # ADDED: Initialize cmd_vel_pub
	rospy.loginfo("Waiting for initial sensor data (scan and odom) in main...")
        initial_rate = rospy.Rate(10)
        while scan_data is None or len(pose_history) < 2:
            initial_rate.sleep()
        rospy.loginfo("Initial sensor data received in main.")

        try:
            listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(5.0))
            rospy.loginfo("Initial TF transform available in main.")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Initial TF transform failed: %s. Navigation might be affected.", str(e))

        goals = []
        num_goals = int(input("How many goals do you want to set? "))
        for i in range(num_goals):
            print("\n--- Enter Goal %d ---" % (i + 1))
            x = float(input("Enter goal X: "))
            y = float(input("Enter goal Y: "))
            goals.append((x, y))

        for idx, (gx, gy) in enumerate(goals):
            rospy.loginfo("Navigating to Goal %d..." % (idx + 1))
            send_goal_with_recovery(gx, gy)
            rospy.sleep(1.0)

        rospy.loginfo("All goals completed.")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")

