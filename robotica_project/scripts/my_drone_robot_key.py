#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from geometry_msgs.msg import Twist

from std_msgs.msg import Empty

import sys, select, termios, tty

import time

msg = """
Control Your Turtlebot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moverFrontal={
    'i':0.25,
    'k':-0.25,
}
moverLados={
    'l':-0.25,
    'j':0.25,
}
moverVertical={
    'o':0.25,
    'u':-0.25,
}
pitch={
    'w':0.25,
    's':-0.25,
}
roll={
    'd':0.25,
    'a':-0.25,
}
yaw={
    'e':-0.25,
    'q':0.25,
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 3
turn = 1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('my_drone_teleop')
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
    pub_land = rospy.Publisher('/ardrone/land', Empty, queue_size=1)

    empty_msg = Empty()

    x = 0
    y = 0
    z = 0
    r_turn = 0
    p_turn = 0
    y_turn = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed_x = 0
    target_speed_y = 0
    target_speed_z = 0
    target_turn_r = 0
    target_turn_p = 0
    target_turn_y = 0

    try:
       # print msg
       # print vels(speed,turn)
        while(1):
            key = getKey()

            if key in moverFrontal:
                x = moverFrontal[key]
                count = 0
            elif key in moverLados:
                y = moverLados[key]
                count = 0
            elif key in moverVertical:
                z = moverVertical[key]
                count = 0
            elif key in roll:
                r_turn = roll[key]
                count = 0
            elif key in pitch:
                p_turn = pitch[key]
                count = 0
            elif key in yaw:
                y_turn = yaw[key]
                count = 0
            elif key == 'b':
                pub_takeoff.publish(empty_msg)
                time.sleep(0.01)
            elif key == ' ':
                pub_land.publish(empty_msg)
                time.sleep(0.01)
            elif key == 'x' or key == 'm' :
                x = 0
                y = 0
                z = 0
                r_turn = 0
                p_turn = 0
                y_turn = 0

            else:
                count = count + 1
                if count > 4:
                    x = 0
                    y = 0
                    z = 0
                    r_turn = 0
                    p_turn = 0
                    y_turn = 0
                if (key == '\x03'):
                    break

            target_speed_x = speed * x
            target_speed_y = speed * y
            target_speed_z = speed * z
            target_turn_r = turn * r_turn
            target_turn_p = turn * p_turn
            target_turn_y = turn * y_turn

            twist = Twist()
            twist.linear.x = target_speed_x; twist.linear.y = target_speed_y; twist.linear.z = target_speed_z
            twist.angular.x = target_turn_r; twist.angular.y = target_turn_p; twist.angular.z = target_turn_y
            pub_cmd_vel.publish(twist)
            time.sleep(0.01)

            #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except:
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub_cmd_vel.publish(twist)
        time.sleep(0.01)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
