#!/usr/bin/env python3
import rospy
from i2cpwm_board.msg import ServoArray
#import pwm_pca9685

def receiveMessage(message):
	print(message)

if __name__ == '__main__':
	rospy.init_node('i2cpwm_board_node_py')
	rospy.loginfo('Python ROS PWM Controller Node Started!')

	subscriber = rospy.Subscriber('/servos_absolute', ServoArray, callback=receiveMessage)
	rospy.spin()

