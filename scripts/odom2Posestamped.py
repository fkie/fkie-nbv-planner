#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg
from threading import Lock
import tf2_ros
import tf2_geometry_msgs


class OdomTfBroadcraster:
    def __init__(self):
        rospy.init_node('robot_pose')
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber('odom', nav_msgs.msg.Odometry, self.handle_odom)
        self.pose_pub = rospy.Publisher(
            'pose', geometry_msgs.msg.PoseStamped, queue_size=10)
        self.world_frame = "world"
        self._lock = Lock()

    def transform_pose(self, input_pose, from_frame, to_frame):

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time.now()

        try:
            output_pose_stamped = self.tf_buffer.transform(
                pose_stamped, to_frame, rospy.Duration(1))
            return output_pose_stamped.pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

    def handle_odom(self, msg):
        with self._lock:
            # print "Publishing Posestamped from odom"

            # Transform the odom posestamped into world
            robot_pose_world = self.transform_pose(
                msg.pose.pose, msg.header.frame_id, self.world_frame)
            robot_pose = geometry_msgs.msg.PoseStamped()
            robot_pose.header.stamp = rospy.Time.now()
            robot_pose.header.frame_id = self.world_frame
            robot_pose.pose = robot_pose_world
            self.pose_pub.publish(robot_pose)
            rospy.loginfo(robot_pose)


if __name__ == '__main__':
    otf = OdomTfBroadcraster()
    rospy.spin()
