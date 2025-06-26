#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist

def send_goal_with_recovery(x, y, yaw_rad):
    rospy.init_node('send_navigation_goal_with_recovery')

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

        rospy.loginfo("Sending goal")
        client.send_goal(goal)

    def perform_recovery():
        rospy.logwarn("Goal failed. Performing recovery: rotating in place for 5 seconds.")
        vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        twist = Twist()
        twist.angular.z = 0.5  # Rotate counterclockwise

        # Rotate for 5 seconds
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        while (rospy.Time.now() - start_time).to_sec() < 5.0 and not rospy.is_shutdown():
            vel_pub.publish(twist)
            rate.sleep()

        # Stop rotation
        twist.angular.z = 0.0
        vel_pub.publish(twist)
        rospy.sleep(1.0)

    # Send initial goal
    send_goal()

    # Wait for result with timeout
    success = client.wait_for_result(rospy.Duration(30.0))
    state = client.get_state()

    if not success or state != actionlib.GoalStatus.SUCCEEDED:
        rospy.logwarn("Goal did not succeed. State: %d", state)
        perform_recovery()

        # Retry the goal after recovery
        rospy.loginfo("Retrying goal after recovery...")
        send_goal()
        client.wait_for_result()
        rospy.loginfo("Final navigation result: %s", client.get_state())
    else:
        rospy.loginfo("Navigation goal succeeded!")

if __name__ == '__main__':
    try:
        send_goal_with_recovery(1.99, 0.32, 0.0)
    except rospy.ROSInterruptException:
        rospy.loginfo("Goal interrupted.")

