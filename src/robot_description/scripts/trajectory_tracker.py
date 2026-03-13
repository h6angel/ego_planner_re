#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from ego_planner.msg import Bspline


class TrajectoryTracker:
    def __init__(self):
        rospy.init_node('trajectory_tracker', anonymous=True)

        self.bspline_topic = rospy.get_param("~bspline_topic", "/planning/bspline")
        self.odom_topic = rospy.get_param("~odom_topic", "/legOdom")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self.traj_frame = rospy.get_param("~traj_frame", "map")

        self.traj_sub = rospy.Subscriber(
            self.bspline_topic, Bspline, self.trajectory_callback, queue_size=10
        )

        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=10
        )

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        
        self.current_pose = None
        self.traj_points = []
        self.traj_index = 0
        
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.5)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 1.0)
        self.goal_threshold = rospy.get_param("~goal_threshold", 0.3)
        self.yaw_threshold = rospy.get_param("~yaw_threshold", 0.3)
        
        self.rate = rospy.Rate(20)
        rospy.loginfo("Trajectory Tracker Started!")
        rospy.loginfo("Waiting for Bspline trajectory on %s", self.bspline_topic)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def trajectory_callback(self, msg):
        if len(msg.pos_pts) == 0 or len(msg.yaw_pts) == 0:
            rospy.logwarn("Bspline message has empty pos_pts or yaw_pts")
            return

        self.traj_points = []
        min_len = min(len(msg.pos_pts), len(msg.yaw_pts))

        for i in range(min_len):
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = self.traj_frame

            pose_stamped.pose.position = msg.pos_pts[i]

            yaw = msg.yaw_pts[i]
            quat = quaternion_from_euler(0, 0, yaw)
            pose_stamped.pose.orientation.x = quat[0]
            pose_stamped.pose.orientation.y = quat[1]
            pose_stamped.pose.orientation.z = quat[2]
            pose_stamped.pose.orientation.w = quat[3]

            self.traj_points.append(pose_stamped)

        self.traj_index = 0
        rospy.loginfo("Received Bspline: %d points, yaw_dt=%.3f",
                      len(self.traj_points), msg.yaw_dt)

    def get_yaw(self, quaternion):
        """从四元数提取 yaw 角"""
        (_, _, yaw) = euler_from_quaternion([
            quaternion.x, quaternion.y, quaternion.z, quaternion.w
        ])
        return yaw

    def calculate_error(self, current, goal):
        dx = goal.position.x - current.position.x
        dy = goal.position.y - current.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        target_yaw = self.get_yaw(goal.orientation)
        current_yaw = self.get_yaw(current.orientation)
        
        angle_error = target_yaw - current_yaw
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
            
        return distance, angle_error

    def run(self):
        while not rospy.is_shutdown():
            twist = Twist()
            
            if self.current_pose and len(self.traj_points) > 0:
                
                if self.traj_index >= len(self.traj_points):
                    self.traj_index = len(self.traj_points) - 1
                
                goal_pose = self.traj_points[self.traj_index].pose
                distance, angle_error = self.calculate_error(self.current_pose, goal_pose)
                
                if distance < self.goal_threshold:
                    self.traj_index += 1
                    if self.traj_index >= len(self.traj_points):
                        rospy.loginfo("Trajectory completed.")
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                    else:
                        rospy.loginfo("Reached point %d, next %d",
                                      self.traj_index - 1, self.traj_index)
                else:
                    twist.linear.x = min(self.max_linear_speed, distance * 0.8)
                    
                    twist.angular.z = math.copysign(1, angle_error) * min(
                        self.max_angular_speed, abs(angle_error) * 2.0
                    )
                    
                    if abs(angle_error) > self.yaw_threshold:
                        twist.linear.x = 0.0
            
            # 发布速度命令
            self.cmd_pub.publish(twist)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        TrajectoryTracker().run()
    except rospy.ROSInterruptException:
        pass
