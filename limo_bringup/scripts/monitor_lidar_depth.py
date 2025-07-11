#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import signal

class TFOdomRecorder:
    def __init__(self):
        self.poses = []
        self.shutdown_requested = False

        rospy.loginfo("TF Pose Recorder started.")
        rospy.loginfo("Press Enter to record current pose from TF (odom → base_link).")

        # Create TF listener
        self.tf_listener = tf.TransformListener()

        # Register Ctrl+C handler
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        rospy.loginfo("\nCtrl+C detected. Exiting and saving recorded poses...")
        self.shutdown_requested = True

    def run(self):
        while not rospy.is_shutdown() and not self.shutdown_requested:
            try:
                raw_input()  # Wait for Enter (Python 2, ROS Melodic)
            except EOFError:
                break
            except KeyboardInterrupt:
                break

            try:
                # Lookup transform between odom and base_link
                (trans, rot) = self.tf_listener.lookupTransform("odom", "base_link", rospy.Time(0))
                x, y = trans[0], trans[1]
                self.poses.append((x, y))
                rospy.loginfo("Recorded TF pose: x=%.2f, y=%.2f", x, y)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("TF lookup failed. Pose not recorded.")
                continue

    def save_poses(self, filename="tf_recorded_poses.txt"):
        with open(filename, "w") as f:
            for x, y in self.poses:
                f.write("%.3f %.3f\n" % (x, y))
        rospy.loginfo("Saved %d poses to %s", len(self.poses), filename)

if __name__ == "__main__":
    rospy.init_node("tf_pose_recorder")
    recorder = TFOdomRecorder()
    try:
        recorder.run()
    finally:
        recorder.save_poses()

