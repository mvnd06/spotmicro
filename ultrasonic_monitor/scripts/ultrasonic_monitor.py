#!/usr/bin/env python

import rospy
from serial import Serial
from std_msgs.msg import String

class UltrasonicMonitor:
    def __init__(self):
        # Initialize ROS node.
        rospy.init_node('ultrasonic_monitor', anonymous=True)
        rospy.loginfo(f"Running Ultrasonic Monitor Node...")

        # Open serial port
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        baud_rate = rospy.get_param('~baud_rate', 9600)
        self.serial = Serial(port=port, baudrate=baud_rate)

        # Create ROS publisher
        self.pub = rospy.Publisher('ultrasonic_data', String, queue_size=10)

        # Start monitoring serial port
        self.monitor()

    def monitor(self):
        while not rospy.is_shutdown():
            # Read data from serial port
            line = self.serial.readline().decode().strip()

            # Publish data to ROS topic
            data = line.split('|')
            leftData, rightData = data[0], data[1]
            rospy.loginfo(f"Received data: {leftData, rightData}")
            self.pub.publish(line)

if __name__ == '__main__':
    try:
        UltrasonicMonitor()
    except rospy.ROSInterruptException:
        pass
