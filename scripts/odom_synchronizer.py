#!/usr/bin/env python2
# coding=utf-8

from __future__ import absolute_import, print_function, division

import itertools
import re
import time
from collections import OrderedDict

import message_filters
import numpy as np
import rospy
import tf.transformations
import yaml
from nav_msgs.msg import Odometry

ODOM_A = '/gnss_odom'
# ODOM_B = '/DEBUG/lio_sam/mapping/pose_map'
ODOM_B = '/odometry'

A = []
B = []
gnss_path = []
map_path = []

GNSS_DOWN = 5
MAP_DOWN = 1


def inverse_homo(trans):
    trans_inverse = np.eye(4)
    # R
    trans_inverse[:3, :3] = trans[:3, :3].T
    # t
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse


def cb_calibration(gnss_odom, map_localization):
    global A, B, first_gnss_inv, first_map_inv

    tic = time.time()
    # 第一帧
    if len(A) == 0:
        first_gnss_inv = inverse_homo(np.matmul(
            tf.listener.xyz_to_mat44(gnss_odom.pose.pose.position),
            tf.listener.xyzw_to_mat44(gnss_odom.pose.pose.orientation)
        ))
        first_map_inv = inverse_homo(np.matmul(
            tf.listener.xyz_to_mat44(map_localization.pose.pose.position),
            tf.listener.xyzw_to_mat44(map_localization.pose.pose.orientation)
        ))

    A.append(np.matmul(first_gnss_inv, np.matmul(
        tf.listener.xyz_to_mat44(gnss_odom.pose.pose.position),
        tf.listener.xyzw_to_mat44(gnss_odom.pose.pose.orientation)
    )))
    B.append(np.matmul(first_map_inv, np.matmul(
        tf.listener.xyz_to_mat44(map_localization.pose.pose.position),
        tf.listener.xyzw_to_mat44(map_localization.pose.pose.orientation)
    )))
    rospy.loginfo('Rcv: %d', len(A))

    # 存储为yaml
    data = OrderedDict()
    data['frameCount'] = len(A)
    for idx, sensor in itertools.product(list(range(len(A))), [1, 2]):
        data['T{}_{}'.format(sensor, idx)] = OrderedDict([
            ('rows', 4),
            ('cols', 4),
            ('dt', 'd'),
            ('data', [float(t) for t in (A if sensor == 1 else B)[idx].reshape(-1)])
        ])
    yaml.add_representer(OrderedDict,
                         lambda dumper, data: dumper.represent_mapping('tag:yaml.org,2002:map', data.items()))

    with open(OUTPUT_FILE, 'w') as yaml_file:
        content = yaml.dump(data, version=(1, 0), default_flow_style=None)
        content = re.sub(r'(T.*_.*:)', r'\1 !!opencv-matrix', content)
        yaml_file.write(content)
    print(time.time() - tic, 's')


if __name__ == '__main__':
    rospy.init_node('odom_synchronizer')

    OUTPUT_FILE = rospy.get_param('~filename')

    # 订阅
    gnss_subscriber = message_filters.Subscriber(ODOM_A, Odometry)
    map_localization_subscriber = message_filters.Subscriber(ODOM_B, Odometry)

    # 时间同步器
    ts = message_filters.ApproximateTimeSynchronizer(
        [gnss_subscriber, map_localization_subscriber],  # 多雷达
        queue_size=10000, slop=0.005
    )
    ts.registerCallback(cb_calibration)

    rospy.spin()
