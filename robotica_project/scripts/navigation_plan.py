#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

#[[direccion, velocidad, tiempo]]
nav_plan = [["takeoff",0,1], ["cw",-0.3,0.6], ["stop",0,0.5], ["z",1,3], ["stop",0,0.5], ["x",1,46], ["stop",0,0.5], ["cw",-1,1.4 ], ["stop",0,0.5], ["x",1,30], ["stop",0,0.5], ["cw",0.3,0.6], ["stop",0,0.5], ["x",1,15], ["stop",0,0.5], ["cw",-1,1.5], ["stop",0,0.5], ["x",1,63], ["stop",0,0.5], ["cw",-1,1.4], ["stop",0,0.5], ["x",1,50], ["stop",0,0.5], ["cw",-1,1.8], ["stop",0,0.5], ["x",1,40], ["stop",0,0.5]]


rospy.init_node('drone_navigation_plan_node')
pub_nav = rospy.Publisher('cmd_vel', Twist, queue_size=1)
pub_take_off = rospy.Publisher('ardrone/takeoff', Empty, queue_size=1)
pub_land = rospy.Publisher('ardrone/land', Empty, queue_size=1)

nav_msg = Twist()
takeoff_land_msg = Empty()

rate = rospy.Rate(5)

time_complete = 0

beginTime = rospy.Time.now()
duration = rospy.Duration(5)
endTime = duration + beginTime

while rospy.Time.now() <= endTime:
    pass
def callback(event):
    #print("se activa funcion")
    global time_complete
    time_complete = 1

for i in range(len(nav_plan)):
    #print("Este es i: ", i)
    #beginTime = rospy.Time.now()
    #duration = rospy.Duration(nav_plan[i][2])
    #endTime = duration + beginTime

    rospy.Timer(rospy.Duration(nav_plan[i][2]), callback, oneshot=True)
    time_complete = 0
    direction = nav_plan[i][0]
    velocity = nav_plan[i][1]
    
    if direction == "x":
        nav_msg.linear.x = velocity
    elif direction == "y":
        nav_msg.linear.y = velocity
    elif direction == "z":
        nav_msg.linear.z = velocity
    elif direction == "cw" or direction == "ccw":
        nav_msg.angular.z = velocity
    elif direction == "stop":
        nav_msg.linear.x = velocity
        nav_msg.linear.y = velocity
        nav_msg.linear.z = velocity
        nav_msg.angular.z = velocity
    #elif direction == "takeoff":
    #    pub_take_off.publish(takeoff_land_msg)
    #    print("deberia de publicar takeoff")
    #elif direction == "land":
    #    pub_land.publish(takeoff_land_msg)
    #while rospy.get_rostime() < endTime:
    while time_complete == 0:
        if direction == "takeoff":
            pub_take_off.publish(takeoff_land_msg)
        elif direction == "land":
            pub_land.publish(takeoff_land_msg)
        else:
            pub_nav.publish(nav_msg)
        rate.sleep()
       #print("en while: ", time_complete)
       # print("rosTime: {} | endTime: {}".format(rospy.get_rostime(), endTime))

