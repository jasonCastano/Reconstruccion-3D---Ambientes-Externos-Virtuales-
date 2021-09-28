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

rospy.init_node('pose_to_path_node')
pub_path = rospy.Publisher('ground_truth_path', Path, queue_size=1)

my_path = Path()


def callback(pose_data):

    global my_path

    my_path.header = pose_data.header
    #pose_stamped = PoseStamped()
    #pose_stamped.header = odometry_data.header
    #pose_stamped.pose = odometry_data.pose.pose
    my_path.poses.append(pose_data)
    #my_path.poses[i].header.seq = 0
    #my_path.poses[i].header.frame_id = "odom"
    #my_path.poses[i].pose = odometry_data.pose.pose

    pub_path.publish(my_path)

rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped, callback)

rospy.spin()

