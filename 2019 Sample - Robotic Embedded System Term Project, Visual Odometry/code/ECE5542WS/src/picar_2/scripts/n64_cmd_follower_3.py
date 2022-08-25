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
from geometry_msgs.msg import Twist
import sys
import os
import time
import math
scriptpath = "../"
status = -1
last_chatter = -1
framerate = -1
enable_tilt_chatter = -1
# Add the directory containing your module to the Python path (wants absolute paths)
# sys.path.append(os.path.abspath(scriptpath))
import camera.camera as camera

def callback_cam(data, cam):
    	#rospy.loginfo(rospy.get_caller_id() + 'I heard:')
    	#rospy.loginfo(data)
    	global status
    	global last_chatter
    	global framerate
    	global enable_tilt_chatter
    	got_t = time.time()
	secs = got_t- int(math.floor(got_t))
	#if rospy.has_param('/usb_cam/framerate'):
		#framerate = rospy.get_param('/usb_cam/framerate')
		#frameperiod = 1/framerate # seconds
	if framerate != -1:
		chatter = int(round(secs*framerate*1/2))
		mystr = "chatter: " + str(chatter) 
		rospy.loginfo(mystr)
		if chatter != last_chatter:
			status = 0;
		if enable_tilt_chatter != -1:
			if enable_tilt_chatter:
				if chatter == 0:
					if not status:
						#cam.turn_up(cam.TILT_STEP)
						cam.turn_right(2*cam.PAN_STEP)
						status = 1
				elif chatter == 1:
					if not status:
						#cam.turn_down(cam.TILT_STEP)
						cam.turn_left(2*cam.PAN_STEP)
						status = 1
				#elif chatter == 2:
				#	if not status:
				#		#cam.turn_up(cam.TILT_STEP)
				#		cam.turn_down(cam.TILT_STEP*2)
				#		status = 1
		last_chatter = chatter

	if data.angular.x == 1:
		cam.turn_right()
	elif data.angular.x == -1:
		cam.turn_left()
	if data.angular.y == 1:
		cam.turn_up()
	elif data.angular.y == -1:
		cam.turn_down()

def listener_cam_mot():
	global framerate
	global enable_tilt_chatter
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    	rospy.init_node('picar_camera_motor', anonymous=True)
	CAM = camera.Camera()
	if rospy.has_param('/usb_cam/framerate'):
		framerate = rospy.get_param('/usb_cam/framerate')
	else:
		framerate = -1
	if rospy.has_param('/enable_tilt_chatter'):
		enable_tilt_chatter = rospy.get_param('/enable_tilt_chatter')
	else:
		framerate = -1
	CAM.ready()
	CAM.turn_right(CAM.PAN_STEP)
	CAM.turn_down(CAM.TILT_STEP)
    	rospy.Subscriber('picar_cmd', Twist, callback_cam, CAM)

    # spin() simply keeps python from exiting until this node is stopped
    	rospy.spin()

if __name__ == '__main__':
    	listener_cam_mot()
