#!/usr/bin/env python

import rospy
import csv
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import numpy as np
import message_filters
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

rospy.init_node('csv_writer_node')

header = ['x_gt', 'y_gt', 'z_gt', 'qx_gt', 'qy_gt', 'qz_gt', 'qw_gt',  'stamp.sec_gt', 'stamp.nsec_gt', 'x_sam', 'y_sam', 'z_sam', 'qx_sam', 'qy_sam', 'qz_sam', 'qw_sam',  'stamp.sec_sam', 'stamp.nsec_sam', 'x_rl', 'y_rl', 'z_rl', 'qx_rl', 'qy_rl', 'qz_rl', 'qw_rl',  'stamp.sec_rl', 'stamp.nsec_rl']
rows = []

def callback(gt, rl, sam):
    global rows
    x_gt = gt.poses[-1].pose.position.x 
    y_gt = gt.poses[-1].pose.position.y
    z_gt = gt.poses[-1].pose.position.z
    qx_gt = gt.poses[-1].pose.orientation.x
    qy_gt = gt.poses[-1].pose.orientation.y
    qz_gt = gt.poses[-1].pose.orientation.z
    qw_gt = gt.poses[-1].pose.orientation.w
    stamp_sec_gt = gt.header.stamp.secs
    stamp_nsec_gt = gt.header.stamp.nsecs
    #(roll_gt,pitch_gt,yaw_gt) = euler_from_quaternion([gt.poses[-1].pose.orientation.x, gt.poses[-1].pose.orientation.y, gt.poses[-1].pose.orientation.z, gt.poses[-1].pose.orientation.w])  
    x_rl = rl.poses[-1].pose.position.x 
    y_rl = rl.poses[-1].pose.position.y
    z_rl = rl.poses[-1].pose.position.z
    qx_rl = rl.poses[-1].pose.orientation.x
    qy_rl = rl.poses[-1].pose.orientation.y
    qz_rl = rl.poses[-1].pose.orientation.z
    qw_rl = rl.poses[-1].pose.orientation.w
    stamp_sec_rl = rl.header.stamp.secs
    stamp_nsec_rl = rl.header.stamp.nsecs

    x_sam = sam.poses[-1].pose.position.x 
    y_sam = sam.poses[-1].pose.position.y
    z_sam = sam.poses[-1].pose.position.z
    qx_sam = sam.poses[-1].pose.orientation.x
    qy_sam = sam.poses[-1].pose.orientation.y
    qz_sam = sam.poses[-1].pose.orientation.z
    qw_sam = sam.poses[-1].pose.orientation.w
    stamp_sec_sam = sam.header.stamp.secs
    stamp_nsec_sam = sam.header.stamp.nsecs

    row = [x_gt, y_gt, z_gt, qx_gt, qy_gt, qz_gt, qw_gt, stamp_sec_gt, stamp_nsec_gt, x_sam, y_sam, z_sam, qx_sam, qy_sam, qz_sam, qw_sam, stamp_sec_sam, stamp_nsec_sam, x_rl, y_rl, z_rl, qx_rl, qy_rl, qz_rl, qw_rl, stamp_sec_rl, stamp_nsec_rl]
    rows.append(row)



def write_data(data):
    global rows
    print("Se procede a escribir la info")
    with open('ruido_imu_gps_gt.csv', 'w') as f:
        writer = csv.writer(f)

        writer.writerow(header)
        writer.writerows(rows)
        print("Se escribio la info")

gt_sub = message_filters.Subscriber("ground_truth_path", Path)
rl_sub = message_filters.Subscriber("ruido_imu_gps", Path)
sam_sub = message_filters.Subscriber("liosam_ruido_imu_gps", Path)
rospy.Subscriber("/write_data", Empty, write_data)

ts = message_filters.ApproximateTimeSynchronizer([gt_sub, rl_sub, sam_sub],queue_size=5, slop=0.1, allow_headerless=True)

ts.registerCallback(callback)

rospy.spin()

