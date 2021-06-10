#!/usr/bin/env python2.7
# coding=utf-8
"""
将Lego-loam的里程计转换为前右下坐标系
"""
from __future__ import print_function, division

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def cb_handle_odom(twist_odometry):
    current_time = rospy.Time()
    # current_time = twist_odometry.header.stamp
    try:
        pose = PoseStamped()
        pose.header = twist_odometry.header
        pose.pose = twist_odometry.pose.pose
        pose_map = transform_listener.transformPose('map', pose).pose
        # 用map下的pose
        twist_odometry.pose.pose = pose_map

        # lookup transform
        # xyz, quat = transform_listener.lookupTransform(
        #     'map', 'chassis_link', current_time)
        # 将刚体变换转化为矩阵
        xyz, quat = transform_listener.lookupTransform(
            'camera', 'base_link', current_time)
        trans = np.matmul(
            tf.transformations.translation_matrix(xyz),
            tf.transformations.quaternion_matrix(quat)
        )

        # 将实时位姿转换为矩阵
        pose_lidar = np.matmul(
            tf.listener.xyz_to_mat44(twist_odometry.pose.pose.position),
            tf.listener.xyzw_to_mat44(twist_odometry.pose.pose.orientation),
        )

        # 获取新的坐标系下位姿
        # pose_chassis = pose_lidar(map->base_link) * trans(base_link->chassis_link)
        pose_chassis = np.matmul(pose_lidar, trans)
        xyz = tf.transformations.translation_from_matrix(pose_chassis)
        quat = tf.transformations.quaternion_from_matrix(pose_chassis)

    except:
        rospy.logwarn('Cannot find transform from odometry...')
        return
    # to odometry
    odom = Odometry()
    odom.header.frame_id = 'map'
    odom.header.stamp = twist_odometry.header.stamp
    odom.child_frame_id = 'base_link'

    # 融合底盘的全局位置
    odom.pose.pose = Pose(Point(*xyz), Quaternion(*quat))

    # 这里使用imu的速度测量
    odom.twist = twist_odometry.twist
    odom_publisher.publish(odom)


if __name__ == "__main__":
    rospy.init_node("localization_provider")
    rospy.loginfo('\n\n===== ====')
    rospy.loginfo('[map] to [base_link] odom started.\n\n')

    odom_publisher = rospy.Publisher('/odometry', Odometry, queue_size=1)
    # rospy.Subscriber('/odometry/imu', Odometry, cb_handle_odom, queue_size=1)
    rospy.Subscriber('/aft_mapped_to_init', Odometry, cb_handle_odom, queue_size=1)
    transform_listener = tf.TransformListener()

    rospy.spin()
