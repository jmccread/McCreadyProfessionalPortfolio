#! /usr/bin/env python

import time
import rospy
import tf
import roslib

from geometry_msgs.msg import Vector3, Quaternion, WrenchStamped

def sensorsDataToVector3(data):
	return Vector3(data['x'], data['y'], data['z'])
	
def tf_callback(msg):
	
if __name__ == '__main__':
	rospy.init_node('picar_odom')
	listener = tf.TransformListener()

	#pub_accelerometer = rospy.Publisher('sensehat/accelerometer', Vector3, queue_size=10)
	#pub_gyroscope = rospy.Publisher('sensehat/gyroscope', Vector3, queue_size=10)
	#pub_imu = rospy.Publisher('sensehat/imu', Imu, queue_size=10); 
	
	try:
		rospy.loginfo(rospy.get_caller_id() + " Starting")
		while not rospy.is_shutdown():

			
			#pub_accelerometer.publish(sensorsDataToVector3(ros_sensehat.sensorhelper.get_accelerometer(sense)))
			#pub_gyroscope.publish(sensorsDataToVector3(ros_sensehat.sensorhelper.get_gyroscope(sense)))
			#i.header.stamp = rospy.Time.now()
			#i.angular_velocity = sensorsDataToVector3(ros_sensehat.sensorhelper.get_gyroscope(sense))
			#i.linear_acceleration = sensorsDataToVector3(ros_sensehat.sensorhelper.get_accelerometer(sense))
			# Could populated other fields but I do not think they are required
			
			rospy.Subscriber('/rebvo_node/tf', tf, tf_callback
			
			pub_imu.publish(i)
			
			rate.sleep()
		
		rospy.loginfo(rospy.get_caller_id() + " Quit")
	except rospy.ROSInterruptException:
		pass


