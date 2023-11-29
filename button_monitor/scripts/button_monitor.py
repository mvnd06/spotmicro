#!/usr/bin/env python

import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Empty
import os
import sys

class ButtonNode:
    def __init__(self, cooldown):
        rospy.init_node('button_node', anonymous=True)
        rospy.loginfo(f"Running Display Manager Node...")

        self.pub = rospy.Publisher('button_press', Empty, queue_size=10)
        self.cooldown = cooldown
        self.last_time = time.monotonic()
        self.button_pin = 22
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.button_pin, GPIO.RISING, callback=self.button_callback, bouncetime=50)

    def button_callback(self, channel):
        current_time = time.monotonic()
        if current_time - self.last_time >= self.cooldown:
            rospy.logdebug('Button was pushed')
            self.pub.publish()
            self.last_time = current_time

    def run(self):
        rospy.spin()
        GPIO.cleanup()

if __name__ == '__main__':
    cooldown = rospy.get_param('~cooldown', 0.5)

    # Check if the script is running as root (sudo)
    if os.geteuid() == 0:
        # The script is running as root (sudo), no need to use sudo again.
        button_node = ButtonNode(cooldown)
    else:
        # The script is not running as root, use sudo to run the script with elevated permissions.
        os.system(f"sudo {sys.executable} {os.path.realpath(__file__)}")

