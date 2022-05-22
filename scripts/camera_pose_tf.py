#!/usr/bin/env python3
""" Publish the current pose of the camera or any sensor in world coordinates """
import tf
import math
import rospy
import roslib
import geometry_msgs.msg


if __name__ == '__main__':
    rospy.init_node('camera_pose_tf')

    listener = tf.TransformListener()

    camera_pose_pub = rospy.Publisher(
        'camera_pose', geometry_msgs.msg.PoseStamped, queue_size=1)
    world_frame = rospy.get_param('~world_frame', None)
    sensor_frame = rospy.get_param('~sensor_frame', None)
    rate_param = rospy.get_param('~rate', 10.0)
    print("world frame->", world_frame)
    print("sensor frame->", sensor_frame)
    rospy.loginfo("Initialised camera pose tf listener")

    rate = rospy.Rate(rate_param)
    while not rospy.is_shutdown():
        try:
            translation, rotation = listener.lookupTransform(
                world_frame, sensor_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        cam_pos = geometry_msgs.msg.PoseStamped()
        cam_pos.header.stamp = rospy.Time.now()
        cam_pos.header.frame_id = world_frame
        cam_pos.pose.position.x = translation[0]
        cam_pos.pose.position.y = translation[1]
        cam_pos.pose.position.z = translation[2]
        cam_pos.pose.orientation.x = rotation[0]
        cam_pos.pose.orientation.y = rotation[1]
        cam_pos.pose.orientation.z = rotation[2]
        cam_pos.pose.orientation.w = rotation[3]
        camera_pose_pub.publish(cam_pos)
        rospy.loginfo("Publishing the camera pose!")
        rospy.loginfo(cam_pos)

        rate.sleep()
