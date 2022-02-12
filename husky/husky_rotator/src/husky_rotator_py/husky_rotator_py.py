#!/usr/bin/env python

import time	
import rospy
from geometry_msgs.msg import Twist

#init ros and the required variables
rospy.init_node('husky_rotator_py')
pub = rospy.Publisher('/husky_velocity_controller/cmd_vel',Twist,queue_size=10)
vel = Twist()

#Duration to turn in a direction (10s)	
duration = 10


while not rospy.is_shutdown():

#Turn Robot Clockwise for a specific duration
	end_time = time.time() + duration
	while time.time() < end_time:
	    vel.linear.x = -0.5 #Robot run in reverse due to obstruction in the playpen map
	    vel.angular.z = 0.3
	    pub.publish(vel)

#Turn Robot Counter-Clockwise for a specific duration
	end_time = time.time() + duration
	while time.time() < end_time:
		vel.angular.z = -0.3
		pub.publish(vel)




    
