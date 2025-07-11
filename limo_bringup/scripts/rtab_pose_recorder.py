#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped

class ClickedPointRecorder:
    def __init__(self):
        self.poses = []
        rospy.Subscriber("/clicked_point", PointStamped, self.point_callback)
        rospy.loginfo("Waiting for points published from RViz (/clicked_point)...")

    def point_callback(self, msg):
        x = msg.point.x
        y = msg.point.y
        self.poses.append((x, y))
        rospy.loginfo("Received point: x=%.2f, y=%.2f", x, y)

    def save_poses(self, filename="clicked_points.txt"):
        with open(filename, "w") as f:
            for x, y in self.poses:
                f.write("%.3f %.3f\n" % (x, y))
        rospy.loginfo("Saved %d points to %s", len(self.poses), filename)

if __name__ == "__main__":
    rospy.init_node("clicked_point_recorder")
    recorder = ClickedPointRecorder()
    rospy.spin()
    recorder.save_poses()

