#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

joy_data = Joy()
joy_data.buttons = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
velocity = Twist()

def callback(data):
    global joy_data
    joy_data = data
    print(joy_data.buttons)


def control_y(joy_data):
    #X Button (Counter clockwise)
    if(joy_data.buttons[2] == 1):
        velocity.angular.z = 1

    #B Button (clockwise)
    elif(joy_data.buttons[1] == 1):
        velocity.angular.z = -1
    else:
        velocity.angular.z = 0

def control_x(joy_data):
    #Y Button (Forward)
    if(joy_data.buttons[3] == 1):
        velocity.linear.x = .5

    #A Button (Backward)
    elif(joy_data.buttons[0] == 1):
        velocity.linear.x = -.5
    else:
        velocity.linear.x = 0

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously. --'listener'
    rospy.init_node('tpain', anonymous=True)

    pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size = 10)
    rospy.Subscriber('joy', Joy, callback)

    global velocity
    global joy_data
    
    #print(type(joy_data.buttons[0]))
    while not rospy.is_shutdown():    
        control_x(joy_data)
        control_y(joy_data)
        pub.publish(velocity)
        

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__': 
    listener();
