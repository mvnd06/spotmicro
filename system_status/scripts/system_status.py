#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

import subprocess

def get_service_status(service_name):
    cmd = ['systemctl', 'is-active', service_name]
    try:
        subprocess.check_output(cmd)
        return 1
    except subprocess.CalledProcessError:
        return 0

def main():
    # Initialize ROS node
    rospy.init_node('system_status')
    rospy.loginfo(f"Running System Status Node...")

    # Create publishers
    motion_pub = rospy.Publisher('/motion_status', Int32, queue_size=10)
    display_pub = rospy.Publisher('/display_status', Int32, queue_size=10)
    bridge_pub = rospy.Publisher('/rosbridge_status', Int32, queue_size=10)

    rate = rospy.Rate(5)  # 5 Hz

    while not rospy.is_shutdown():
        # Get service status
        motion_status = get_service_status('motion')
        display_status = get_service_status('display')
        rosbridge_status = get_service_status('rosbridge')

        # Publish status messages
        motion_pub.publish(motion_status)
        display_pub.publish(display_status)
        bridge_pub.publish(rosbridge_status)

        rate.sleep()

if __name__ == '__main__':
    main()
