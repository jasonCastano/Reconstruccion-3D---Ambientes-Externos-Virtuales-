#!/usr/bin/env python

import rospy
import csv
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import numpy as np
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

rospy.init_node('csv_writer_node')

header = ['x_sam', 'y_sam', 'z_sam', 'qx_sam', 'qy_sam', 'qz_sam', 'qw_sam', 'stamp.sec_sam', 'stamp.nsec_sam']
rows = []

def callback(path_data):
    global rows
    x = path_data.poses[-1].pose.position.x 
    y = path_data.poses[-1].pose.position.y
    z = path_data.poses[-1].pose.position.z
    qx = path_data.poses[-1].pose.orientation.x
    qy = path_data.poses[-1].pose.orientation.y
    qz = path_data.poses[-1].pose.orientation.z
    qw = path_data.poses[-1].pose.orientation.w
    #(roll,pitch,yaw) = euler_from_quaternion([path_data.poses[-1].pose.orientation.x, path_data.poses[-1].pose.orientation.y, path_data.poses[-1].pose.orientation.z, path_data.poses[-1].pose.orientation.w])  
    row = [x,y,z,qx,qy,qz,qw,path_data.header.stamp.secs, path_data.header.stamp.nsecs]
    rows.append(row)



def write_data(data):
    global rows
    with open('liosam_ruido_imu_gps.csv', 'w') as f:
        writer = csv.writer(f)

        writer.writerow(header)
        writer.writerows(rows)

rospy.Subscriber("liosam_ruido_imu_gps", Path, callback)
rospy.Subscriber("/write_data", Empty, write_data)

rospy.spin()

