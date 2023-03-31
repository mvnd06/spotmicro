#!/usr/bin/env python

import rospy
from serial import Serial
from std_msgs.msg import Int32MultiArray

class UltrasonicMonitor:
    def __init__(self):
        # Initialize ROS node.
        rospy.init_node('ultrasonic_monitor', anonymous=True)
        rospy.loginfo(f"Running Ultrasonic Monitor Node...")

        # Open serial port.
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        baud_rate = rospy.get_param('~baud_rate', 9600)
        self.serial = Serial(port=port, baudrate=baud_rate)

        # Create ROS publisher.
        self.pub = rospy.Publisher('ultrasonic_data', Int32MultiArray)

        # Start monitoring serial port.
        self.monitor()

    def monitor(self):
        while not rospy.is_shutdown():
            try:
                # Read data from serial port.
                line = self.serial.readline().decode().strip()

                # Process data from serial port.
                data = line.split('|')
                leftData, rightData = data[0], data[1]
                leftInches = int(leftData.split(':')[1])
                rightInches = int(rightData.split(':')[1])
                rospy.loginfo(f"Received data -> L: {leftInches}, R: {rightInches}")

                # Publish data to ROS topic.
                dataArray = Int32MultiArray()
                dataArray.data = [leftInches, rightInches]
                self.pub.publish(dataArray)
            except Exception as e:
               rospy.logerr("Error occurred while processing data from serial port.")
               rospy.logerr(e)
               continue

if __name__ == '__main__':
    try:
        UltrasonicMonitor()
    except rospy.ROSInterruptException:
        pass
