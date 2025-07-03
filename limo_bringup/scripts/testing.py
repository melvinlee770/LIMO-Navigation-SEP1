#!/usr/bin/env python
# -*- coding: utf-8 -*-

from std_srvs.srv import Empty
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
    if len(pose_history) > 100:
        pose_history.pop(0)

def get_current_pose():
    listener = tf.TransformListener()
    rospy.loginfo("Waiting for transform from map to base_link...")
    listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(5.0))
    (trans, rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))
    return trans

def compute_yaw_to_goal(current_x, current_y, goal_x, goal_y):
    dx = goal_x - current_x
    dy = goal_y - current_y
    return math.atan2(dy, dx)

def send_goal_with_recovery(x, y):
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

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

    def perform_recovery_with_strategy(attempt):
        def clear_costmap():
            rospy.logwarn("Clearing costmap...")
            try:
                rospy.wait_for_service('/move_base/clear_costmaps', timeout=2.0)
                clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
                clear_srv()
                rospy.loginfo("Costmap cleared.")
            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logerr("Costmap clear failed: %s", str(e))

        clear_costmap()

        case = ((attempt - 1) % 3) + 1  # Cycles 1 → 2 → 3

        if case == 1:
            rospy.logwarn("Recovery Step 1 (cycled): Clear costmap and retry.")
            return

        elif case == 2:
            rospy.logwarn("Recovery Step 2 (cycled): Check LIDAR, rotate 180° if safe.")

            if scan_data is None:
                rospy.logwarn("No scan data available! Skipping rotation.")
                return

            angle_min = scan_data.angle_min
            angle_increment = scan_data.angle_increment
            ranges = scan_data.ranges

            def get_index_for_angle(deg):
                angle_rad = math.radians(deg)
                return int((angle_rad - angle_min) / angle_increment)

            vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            rate = rospy.Rate(10)

            def rotate_left():
                twist = Twist()
                twist.angular.z = 0.5
                duration = math.pi / 0.5
                start_time = rospy.Time.now()
                rospy.loginfo("Turning LEFT 180°...")
                while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
                    vel_pub.publish(twist)
                    rate.sleep()
                vel_pub.publish(Twist())
                rospy.sleep(1.0)

            def rotate_right():
                twist = Twist()
                twist.angular.z = -0.5
                duration = math.pi / 0.5
                start_time = rospy.Time.now()
                rospy.loginfo("Turning RIGHT 180°...")
                while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
                    vel_pub.publish(twist)
                    rate.sleep()
                vel_pub.publish(Twist())
                rospy.sleep(1.0)

            front_index = get_index_for_angle(0)
            left_index = get_index_for_angle(90)
            right_index = get_index_for_angle(-90)

            front_clear = False
            left_clear = False
            right_clear = False

            if 0 <= front_index < len(ranges):
                dist = ranges[front_index]
                if not math.isinf(dist) and not math.isnan(dist) and dist >= 0.08:
                    front_clear = True

            if 0 <= left_index < len(ranges):
                dist = ranges[left_index]
                if not math.isinf(dist) and not math.isnan(dist) and dist >= 0.08:
                    left_clear = True

            if 0 <= right_index < len(ranges):
                dist = ranges[right_index]
                if not math.isinf(dist) and not math.isnan(dist) and dist >= 0.08:
                    right_clear = True

            if front_clear and left_clear:
                rotate_left()
            elif front_clear and right_clear:
                rotate_right()
            else:
                rospy.logwarn("Not enough clearance to rotate left or right.")
                return

        else:
            rospy.logwarn("Recovery Step 3 (cycled): Clear costmap, reverse for 0.5s, then retry.")
            vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            rate = rospy.Rate(10)
            twist = Twist()
            twist.linear.x = -0.2
            duration = 0.5
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
                vel_pub.publish(twist)
                rate.sleep()
            vel_pub.publish(Twist())
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

    return

if __name__ == '__main__':
    try:
        rospy.init_node('dual_goal_navigation_node')

        goals = []
        for i in range(2):
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

