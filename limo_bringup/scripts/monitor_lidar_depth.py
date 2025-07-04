#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import signal

class OdomRecorder:
    def __init__(self):
        self.poses = []
        self.current_pose = None
        self.shutdown_requested = False
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.loginfo("Odom Pose Recorder started.")
        rospy.loginfo("Press Enter to record current odom pose.")

        # Register Ctrl+C handler
        signal.signal(signal.SIGINT, self.signal_handler)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def signal_handler(self, sig, frame):
        rospy.loginfo("\nCtrl+C detected. Exiting and saving recorded poses...")
        self.shutdown_requested = True

    def run(self):
        while not rospy.is_shutdown() and not self.shutdown_requested:
            try:
                raw_input()  # Wait for Enter key (Python 2)
            except EOFError:
                break
            except KeyboardInterrupt:
                break

            if self.current_pose is None:
                rospy.logwarn("No odometry received yet.")
                continue

            pose = self.current_pose
            x = pose.position.x
            y = pose.position.y

            self.poses.append((x, y))
            rospy.loginfo("Recorded pose: x=%.2f, y=%.2f", x, y)

    def save_poses(self, filename="odom_recorded_poses.txt"): #change filename
        with open(filename, "w") as f:
            for x, y in self.poses:
                f.write("%.3f %.3f\n" % (x, y))
        rospy.loginfo("Saved %d poses to %s", len(self.poses), filename)

if __name__ == "__main__":
    rospy.init_node("odom_pose_recorder")
    recorder = OdomRecorder()
    try:
        recorder.run()
    finally:
        recorder.save_poses()

