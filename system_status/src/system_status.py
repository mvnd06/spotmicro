#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

import subprocess

def get_service_status(service_name):
    cmd = ['systemctl', 'is-active', service_name]
    try:
        rospy.loginfo(f"Trying System: {service_name}")
        subprocess.check_output(cmd)
        return 1
    except subprocess.CalledProcessError:
        rospy.loginfo(f"Exception..")
        return 0

def main():
    # Initialize ROS node
    rospy.init_node('system_status')
    rospy.loginfo(f"Running System Status Node...")

    # Create publishers
    motion_pub = rospy.Publisher('/motion_status', Int32, queue_size=10)
    display_pub = rospy.Publisher('/display_status', Int32, queue_size=10)
    gui_pub = rospy.Publisher('/gui_status', Int32, queue_size=10)
    
    rospy.loginfo(f"Created Publishers..")

    rate = rospy.Rate(5)  # 5 Hz

    while not rospy.is_shutdown():
        rospy.loginfo(f"Started Loop...")
        # Get service status
        motion_status = get_service_status('motion')
        display_status = get_service_status('display')
        gui_status = get_service_status('gui')

        rospy.loginfo(f"Motion: {motion_status}, Display: {display_status}, GUI: {gui_status}")

        # Publish status messages
        motion_pub.publish(motion_status)
        display_pub.publish(display_status)
        gui_pub.publish(gui_status)

        rate.sleep()

if __name__ == '__main__':
    main()
