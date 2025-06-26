#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import tf

class OdomRecorder:
    def __init__(self):
        self.poses = []
        self.current_pose = None
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.loginfo("Odom Pose Recorder started.")
        rospy.loginfo("Press Enter to record current odom pose.")
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def run(self):
        while not rospy.is_shutdown():
            try:
                raw_input()  # Wait for Enter key
            except EOFError:
                break

            if self.current_pose is None:
                rospy.logwarn("No odometry received yet.")
                continue

            pose = self.current_pose
            x = pose.position.x
            y = pose.position.y

            orientation = pose.orientation
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w])

            self.poses.append((x, y, yaw))
            rospy.loginfo("Recorded pose: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw)

    def save_poses(self, filename="odom_recorded_poses.txt"):
        with open(filename, "w") as f:
            for x, y, yaw in self.poses:
                f.write("%.3f %.3f %.3f\n" % (x, y, yaw))
        rospy.loginfo("Saved %d poses to %s", len(self.poses), filename)

if __name__ == "__main__":
    rospy.init_node("odom_pose_recorder")
    recorder = OdomRecorder()
    try:
        recorder.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        recorder.save_poses()

