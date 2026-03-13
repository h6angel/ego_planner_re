#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
import tf


class TfOdomPublisher(object):
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "wheel_legged_robot")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        self.br = tf.TransformBroadcaster()
        self.pub_odom = rospy.Publisher("/legOdom", Odometry, queue_size=10)

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.cb_model_states)

        rospy.loginfo("tf_odom_publisher started. model_name=%s", self.model_name)

    def cb_model_states(self, msg):
        if self.model_name not in msg.name:
            return
        idx = msg.name.index(self.model_name)

        pose = msg.pose[idx]
        twist = msg.twist[idx]
        now = rospy.Time.now()

        self.br.sendTransform(
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w),
            now,
            self.base_frame,
            self.odom_frame
        )

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose = pose
        odom.twist.twist = twist

        self.pub_odom.publish(odom)


if __name__ == "__main__":
    rospy.init_node("tf_odom_publisher")
    TfOdomPublisher()
    rospy.spin()

