#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, ColorRGBA, Empty, String
from enum import Enum
from system_monitor import SystemMonitor

RED = ColorRGBA(r=255.0, g=0.0, b=0.0, a=1.0)
GREEN = ColorRGBA(r=0.0, g=255.0, b=0.0, a=1.0)
BLUE = ColorRGBA(r=0.0, g=0.0, b=255.0, a=1.0)
PURPLE = ColorRGBA(r=128.0, g=0.0, b=128.0, a=1.0)
BLACK = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)

class ScreenMode(Enum):
    ULTRASONIC = 1
    STATIC = 2
    MONITOR = 3

class DisplayManager:
    def __init__(self):
        rospy.init_node('display_manager')
        rospy.loginfo(f"Running Display Manager Node...")

        self.current_color = BLACK
        self.current_text = ''

        self.screen_mode = ScreenMode.ULTRASONIC
        self.system_monitor = SystemMonitor()
        self.system_monitor.run()
        self.button_taps = 0

        self.color_pub = rospy.Publisher('oled_color', ColorRGBA, queue_size=10)
        self.text_pub = rospy.Publisher('oled_text', String, queue_size=10)
        rospy.Subscriber('ultrasonic_data', Int32MultiArray, self.ultrasonic_callback)
        rospy.Subscriber('button_press', Empty, self.button_callback)

#        rospy.Timer(rospy.Duration(1), self.publish_system_stats)
        rospy.loginfo("End of init")


    # Callback Methods

    def ultrasonic_callback(self, msg):
        rospy.loginfo('ultrasonic_callback')
        if self.screen_mode != ScreenMode.ULTRASONIC:
            return
        # Parse the incoming message to extract the left and right distance values
        left_dist, right_dist = msg.data[0], msg.data[1]
        rospy.loginfo(f"Received data: {left_dist}, {right_dist}")

        # Determine if the distance is close enough to trigger an alert
        if (left_dist < 5 or right_dist < 5):
            self.current_color = RED
        else:
            self.current_color = GREEN
        self.publish_color()

    def button_callback(self, msg):
        rospy.loginfo('button_callback')
        rospy.loginfo(self.button_taps)
        self.button_taps += 1
        self.update_mode()

    # Publish Methods

    def publish_color(self):
        rospy.loginfo('publish_color')
        self.color_pub.publish(self.current_color)

    def publish_system_stats(self):
        rospy.loginfo('publish_system_stats')
        if self.system_monitor.paused:
            rospy.loginfo('publish_system_stats early return')
            return
        stats_array = [
            self.system_monitor.ip,
            self.system_monitor.cpu, 
            self.system_monitor.mem_usage, 
            self.system_monitor.disk,
            self.system_monitor.temperature
        ]
        stats_str = '|'.join(stats_array)
        if self.current_text != stats_str:
            rospy.loginfo(f"Printing: {stats_str}")
            self.text_pub.publish(stats_str)
            self.current_text = stats_str

    # Helper Methods

    def static_screen(self):
        rospy.loginfo('static_screen')
        self.current_color = PURPLE
        self.publish_color()
        
    def update_mode(self):
        rospy.loginfo('update_mode')
        # Prepare for transition.
        if self.screen_mode == ScreenMode.MONITOR:
            self.system_monitor.pause()
        
        # Update mode.
        self.screen_mode = self.button_taps % len(ScreenMode)

        # Mode startup tasks.
        if self.screen_mode == ScreenMode.STATIC:
            self.static_screen()
        elif self.screen_mode == ScreenMode.MONITOR:
            self.system_monitor.resume()
        
if __name__ == '__main__':
    try:
        display_manager = DisplayManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
