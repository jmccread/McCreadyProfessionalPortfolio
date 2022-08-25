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
from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import Twist
#from picar import front_wheels
from picar import back_wheels


BW_direction = 1

#def callback(data, fw):
#	global BW_direction
#	#rospy.loginfo(rospy.get_caller_id() + 'I heard:')
#	rospy.loginfo(data)
#    	fw.turn(data.angular.z)
#    	#rospy.loginfo(rospy.get_caller_id() + 'I heard:')
#    	#print(BW_direction)

def callback_drive(data, bw):
	global BW_direction
	if (data.data < 0):
		bw.backward()
		bw.speed = int(abs(data.data))
#		print("Going foward, BW_dir: ", BW_direction)
#	elif ((data.linear.y > 0) and not BW_direction):
#		#bw.forward()
#		BW_direction = 1
#		bw.speed = int(data.linear.x)
#		print("Going forward, BW_dir: ", BW_direction)
#	elif ((data.linear.y < 0) and BW_direction):
#		#bw.backward()
#		BW_direction = 0
#		bw.speed = int(data.linear.x)
#		print("Going backwards, BW_dir: ", BW_direction)
	elif (data.data > 0):
		bw.forward()
		bw.speed = int(abs(data.data))
#		print("Going backwards, BW_dir: ", BW_direction)
	elif data.data == 0:
		bw.speed = 0


def listener_auto():

    	# In ROS, nodes are uniquely named. If two nodes with the same
    	# name are launched, the previous one is kicked off. The
    	# anonymous=True flag means that rospy will choose a unique
    	# name for our 'listener' node so that multiple listeners can
   	# run simultaneously.
	rospy.init_node('picar_drive', anonymous=True)
    	#FW = front_wheels.Front_Wheels(0)
    	BW = back_wheels.Back_Wheels(0)
    	#BW.ready();
    	#rospy.Subscriber('picar_cmd', Twist, callback, FW)
	rospy.Subscriber('control_effort', Float64, callback_drive, BW)
    	#rospy.Subscriber('picar_cmd', Twist, prop_callback, BW)
    	# spin() simply keeps python from exiting until this node is stopped
    	rospy.spin()

if __name__ == '__main__':
    	listener_auto()
