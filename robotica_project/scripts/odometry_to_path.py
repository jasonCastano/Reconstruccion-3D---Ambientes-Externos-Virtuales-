#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import numpy as np
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

rospy.init_node('odometry_to_path_node')
pub_path = rospy.Publisher('ruido_imu', Path, queue_size=1)

my_path = Path()
global xAnt
global yAnt
global zAnt

xAnt = 0.0
yAnt = 0.0
zAnt = 0.0

count = 0


def callback(odometry_data):

    global my_path, xAnt, yAnt, zAnt, count

    my_path.header = odometry_data.header
    pose_stamped = PoseStamped()
    pose_stamped.header.seq = 0
    pose_stamped.header.stamp = odometry_data.header.stamp
    pose_stamped.header.frame_id = "odom"
    pose_stamped.pose = odometry_data.pose.pose
    if(xAnt != pose_stamped.pose.position.x or yAnt != pose_stamped.pose.position.y or zAnt != pose_stamped.pose.position.z):
        
        #my_path.poses[i].header.seq = 0
        #my_path.poses[i].header.frame_id = "odom"
        #my_path.poses[i].pose = odometry_data.pose.pose
        if count <= 1000:
            my_path.poses.append(pose_stamped)
            count = count + 1
        else:
            my_path.poses.pop(0)
            my_path.poses.append(pose_stamped)
            # count = 0
        my_path.poses.append(pose_stamped)
        pub_path.publish(my_path)

        # xAnt = pose_stamped.pose.position.x
        # yAnt = pose_stamped.pose.position.y
        # zAnt = pose_stamped.pose.position.z
        
rospy.Subscriber("/odometry/filtered", Odometry, callback)

rospy.spin()

