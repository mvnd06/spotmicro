#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, ColorRGBA, Empty
from enum import Enum

RED = ColorRGBA(r=255.0, g=0.0, b=0.0, a=1.0)
GREEN = ColorRGBA(r=0.0, g=255.0, b=0.0, a=1.0)
BLUE = ColorRGBA(r=0.0, g=0.0, b=255.0, a=1.0)
PURPLE = ColorRGBA(r=128.0, g=0.0, b=128.0, a=1.0)
BLACK = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)

class ScreenMode(Enum):
    ULTRASONIC = 1
    STATIC = 2

class DisplayManager:
    def __init__(self):
        rospy.init_node('display_manager')
        rospy.loginfo(f"Running Display Manager Node...")

        self.current_color = BLACK
        self.color_pub = rospy.Publisher('oled_color', ColorRGBA, queue_size=10)
        self.screen_mode = ScreenMode.ULTRASONIC

        rospy.Subscriber('ultrasonic_data', Int32MultiArray, self.ultrasonic_callback)
        rospy.Subscriber('button_press', Empty, self.button_callback)

    def ultrasonic_callback(self, msg):
        if self.screen_mode != ScreenMode.ULTRASONIC:
            return
        # Parse the incoming message to extract the left and right distance values
        left_dist, right_dist = msg.data[0], msg.data[1]
        rospy.logdebug(f"Received data: {left_dist}, {right_dist}")

        # Determine if the distance is close enough to trigger an alert
        if (left_dist < 5 or right_dist < 5):
            self.current_color = RED
        else:
            self.current_color = GREEN
        self.publish_color()

    def static_screen(self):
        self.current_color = PURPLE
        self.publish_color()

    def button_callback(self, msg):
        if self.screen_mode == ScreenMode.ULTRASONIC:
            self.screen_mode = ScreenMode.STATIC
            self.static_screen()
        else:
            self.screen_mode = ScreenMode.ULTRASONIC

    def publish_color(self):
        self.color_pub.publish(self.current_color)
        
if __name__ == '__main__':
    try:
        display_manager = DisplayManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
