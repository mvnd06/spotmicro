#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, ColorRGBA

RED = ColorRGBA(r=255.0, g=0.0, b=0.0, a=1.0)
GREEN = ColorRGBA(r=0.0, g=255.0, b=0.0, a=1.0)
BLUE = ColorRGBA(r=0.0, g=0.0, b=255.0, a=1.0)
BLACK = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)

class DisplayManager:
    def __init__(self):
        rospy.init_node('display_manager')
        rospy.loginfo(f"Running Display Manager Node...")

        self.current_color = BLACK
        self.color_pub = rospy.Publisher('oled_color', ColorRGBA, queue_size=10)
        rospy.Subscriber('ultrasonic_data', Int32MultiArray, self.ultrasonic_callback)

    def ultrasonic_callback(self, msg):
        # Parse the incoming message to extract the left and right distance values
        left_dist, right_dist = msg.data[0], msg.data[1]
        rospy.loginfo(f"Received data: {left_dist}, {right_dist}")

        # Determine if the distance is close enough to trigger an alert
        if (left_dist < 5 or right_dist < 5) and self.current_color != RED:
            # Send a message to the oled_color topic to update the color of the screen
            self.color_pub.publish(RED)
        elif self.current_color != GREEN:
            self.color_pub.publish(GREEN)

if __name__ == '__main__':
    try:
        display_manager = DisplayManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
