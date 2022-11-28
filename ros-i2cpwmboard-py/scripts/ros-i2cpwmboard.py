#!/usr/bin/env python3
import rospy
#import pwm_pca9685

if __name__ == 'main':
	rospy.init_node('ros-i2cpwmboard-py')
	rospy.loginfo('Python ROS PWM Controller Node Started!')
	
	#while not rospy.is_shutdown():
