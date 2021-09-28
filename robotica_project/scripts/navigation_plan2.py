#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import numpy as np
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates

#[[x, y, z]]
nav_plan = ["takeoff", [7,-0.001182,1,0.001], [14,-0.001182,1,0.001], [21,-0.001182,1,0.001], [28,-0.001182,1,0.001], [35,-0.001182,1,0.001],  [45.5,-0.001182,1,0.001], [45.5,-7,1,0.001], [45.5,-14,1,0.001], [45.5,-21,1,0.001], [45.5,-28,1,0.001], [45.5,-35,1,0.001], [45.5,-43,1,0.001], [38.5,-43,1,0.001], [30.5,-43.5,1,0.001], [23.5,-43.5,1,0.001], [16.5,-43.5,1,0.001], [9.5,-43,1,0.001], [0.5,-43,1,0.001], [-7.5,-43,1,0.001], [-14.5,-43,1,0.001], [-14.5,-37.5,1,0.001], [-14.5,-30.5,1,0.001], [-14.5,-23.5,1,0.001], [-14.5,-16,1,0.001], [-14.5,-9,1,0.001], [-14.5,0,1,0.001], [-7,0,1,0.001], [0.001,-0.0012,1,0.001], "land"]
# nav_plan = ["takeoff", [7,-0.001182,1,0.001], [14,-0.001182,1,0.001], [21,-0.001182,1,0.001], [28,-0.001182,1,0.001], [35,-0.001182,1,0.001], [45.5,-0.001182,1,0.001], [45.5,-0.001182,1,-1.57], [45.5,-7,1,-1.57], [45.5,-14,1,-1.57], [45.5,-21,1,-1.57], [45.5,-28,1,-1.57], [45.5,-35,1,-1.57], [45.5,-43,1,-1.57], [45.5,-43,1,-3], [38.5,-43,1,-3], [30.5,-43.5,1,-3], [23.5,-43.5,1,-3], [16.5,-43.5,1,-3], [9.5,-43,1,-3], [0.5,-43,1,-3], [-7.5,-43,1,-3], [-14.5,-43,1,-3], [-14.5,-43,1,1.57], [-14.5,-37.5,1,1.57], [-14.5,-30.5,1,1.57], [-14.5,-23.5,1,1.57], [-14.5,-16,1,1.57], [-14.5,-9,1,1.57], [-14.5,0,1,1.57], [-14.5,0,1,0.001], [-7,0,1,0.001], [0.0005,-0.0012,1,0.001], "land"]

rospy.init_node('drone_navigation_plan_node')
pub_nav = rospy.Publisher('cmd_vel', Twist, queue_size=1)
pub_take_off = rospy.Publisher('ardrone/takeoff', Empty, queue_size=1)
pub_land = rospy.Publisher('ardrone/land', Empty, queue_size=1)

nav_msg = Twist()
takeoff_land_msg = Empty()

rate = rospy.Rate(5)

beginTime = rospy.Time.now()
duration = rospy.Duration(5)
endTime = duration + beginTime

kp = 0.1
ki = 0.5
u = {"x":0,"y":0,"z":0,"yaw":0,"x_1":0,"y_1":0,"z_1":0,"yaw_1":0}
e = {"x":0,"y":0,"z":0,"yaw":0,"x_1":0,"y_1":0,"z_1":0,"yaw_1":0}
i = 0

while rospy.Time.now() <= endTime:
    pass

def control(x,y,z,yaw,data_x,data_y,data_z,data_yaw, u, e):
    e["x_1"] = e.get("x")
    e["y_1"] = e.get("y")
    e["z_1"] = e.get("z")
    e["yaw_1"] = e.get("yaw")

    u["x_1"] = u.get("x")
    u["y_1"] = u.get("y")
    u["z_1"] = u.get("z")
    u["yaw_1"] = u.get("yaw")

    e["x"] = data_x - x
    e["y"] = data_y - y
    e["z"] = data_z - z
    e["yaw"] = data_yaw - yaw

    u["x"] = kp*(e.get("x") - e.get("x_1")) + ki*e.get("x") + u.get("x_1")
    u["y"] = kp*(e.get("y") - e.get("y_1")) + ki*e.get("y") + u.get("y_1")
    u["z"] = kp*(e.get("z") - e.get("z_1")) + ki*e.get("z") + u.get("z_1")
    u["yaw"] = kp*(e.get("yaw") - e.get("yaw_1")) + ki*e.get("yaw") + u.get("yaw_1")

    return u, e

def instructions(data):
    global i, u, e
    if i < len(nav_plan):
        plan = nav_plan[i]
        if plan == "takeoff":
            beginTime = rospy.Time.now()
            duration = rospy.Duration(1)
            endTime = duration + beginTime
            while rospy.Time.now() <= endTime:
                pub_take_off.publish(takeoff_land_msg)
                #print("taking off")
                #print("rostime: {} || endTime: {}".format(rospy.Time.now(), endTime))
                rate.sleep()
            i = i + 1
        elif plan == "land":
            nav_msg.linear.x = 0
            nav_msg.linear.y = 0
            nav_msg.linear.z = 0
            nav_msg.angular.z = 0
            beginTime = rospy.Time.now()
            duration = rospy.Duration(1)
            endTime = duration + beginTime
            while rospy.Time.now() < endTime:
                pub_nav.publish(nav_msg)
                pub_land.publish(takeoff_land_msg)
                rate.sleep()
            beginTime = rospy.Time.now()
            duration = rospy.Duration(1)
            endTime = duration + beginTime
            while rospy.Time.now() < endTime:
                pub_land.publish(takeoff_land_msg)
                rate.sleep()
            i = i + 1
        else:
            x = plan[0]
            y = plan[1]
            z = plan[2]
            yaw = plan[3]
            data_x = data.pose[-1].position.x
            data_y = data.pose[-1].position.y
            data_z = data.pose[-1].position.z
            data_q = data.pose[-1].orientation
            q = [data_q.x, data_q.y, data_q.z, data_q.w]
            (_,_,data_yaw) = euler_from_quaternion(q)

            # u, e = control(x,y,z,yaw,data_x,data_y,data_z,data_yaw, u, e)

            # nav_msg.linear.x = u.get("x")
            # nav_msg.linear.x = u.get("y")
            # nav_msg.linear.x = u.get("z")
            # nav_msg.linear.x = u.get("yaw")

            # print(" dif_x: {} \n dif_y: {} \n dif_z: {} \n dif_yaw: {}".format(u.get("x"), u.get("y"), u.get("z"), u.get("yaw")))

            # pub_nav.publish(nav_msg)

            dif_x = data_x - x
            dif_y = data_y - y
            dif_z = data_z - z
            dif_yaw = data_yaw - yaw
            # print("Yaw simulacion: {}".format(data_yaw))
            if yaw == 0.001:
                if abs(dif_x) > 0.1:
                    nav_msg.linear.x = -1*dif_x*kp
                #    print("X ACTUALIZADO")
                if abs(dif_y) > 0.1:
                    nav_msg.linear.y = -1*dif_y*kp
                #    print("Y ACTUALIZADO")
                if abs(dif_z) > 0.1:
                    nav_msg.linear.z = -1*dif_z*kp
                #    print("Z ACTUALIZADO")
                if abs(dif_yaw) > 0.01745:
                    nav_msg.angular.z = -1*dif_yaw
                #    print("YAW ACTUALIZADO")
            elif yaw == -1.57:
                if abs(dif_x) > 0.1:
                    nav_msg.linear.y = -1*dif_x*kp
                #    print("X ACTUALIZADO")
                if abs(dif_y) > 0.1:
                    nav_msg.linear.x = dif_y*kp
                #    print("Y ACTUALIZADO")
                if abs(dif_z) > 0.1:
                    nav_msg.linear.z = -1*dif_z*kp
                #    print("Z ACTUALIZADO")
                if abs(dif_yaw) > 0.01745:
                    nav_msg.angular.z = -1*dif_yaw
                #    print("YAW ACTUALIZADO")
            elif yaw == -3:
                if abs(dif_x) > 0.1:
                    nav_msg.linear.x = dif_x*kp
                #    print("X ACTUALIZADO")
                if abs(dif_y) > 0.1:
                    nav_msg.linear.y = dif_y*kp
                #    print("Y ACTUALIZADO")
                if abs(dif_z) > 0.1:
                    nav_msg.linear.z = -1*dif_z*kp
                #    print("Z ACTUALIZADO")
                if abs(dif_yaw) > 0.01745:
                    nav_msg.angular.z = -1*dif_yaw
                #    print("YAW ACTUALIZADO")
            elif yaw == 1.57:
                if abs(dif_x) > 0.1:
                    nav_msg.linear.y = dif_x*kp
                #    print("X ACTUALIZADO")
                if abs(dif_y) > 0.1:
                    nav_msg.linear.x = -1*dif_y*kp
                #    print("Y ACTUALIZADO")
                if abs(dif_z) > 0.1:
                    nav_msg.linear.z = -1*dif_z*kp
                #    print("Z ACTUALIZADO")
                if abs(dif_yaw) > 0.01745:
                    nav_msg.angular.z = -1*dif_yaw
                #    print("YAW ACTUALIZADO")
            pub_nav.publish(nav_msg)
            #print("moviendose")
            #print(" dif_x: {} \n dif_y: {} \n dif_z: {} \n dif_yaw: {}".format(dif_x, dif_y, dif_z, dif_yaw))
            

            if abs(dif_x) <= 1 and abs(dif_y) <= 1 and abs(dif_z) <= 1 and abs(dif_yaw) <= 0.01745:
                i = i + 1
                #print("***********PASA SIGUIENTE i************")
            
    else:
        rospy.signal_shutdown("Plan de navegacion finalizado exitosamente!")

# rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped, instructions)

rospy.Subscriber("/gazebo/model_states", ModelStates, instructions)

rospy.spin()

