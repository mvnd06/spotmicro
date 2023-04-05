#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, ColorRGBA, Empty, String
from enum import Enum
import subprocess, time

class SystemMonitor:
    def __init__(self):
        self.ip = ""
        self.cpu = ""
        self.mem_usage = ""
        self.disk = ""
        self.temperature = ""
        self.paused = False
        self.update() # set initial values.

    def update(self):
        # Get IP address
        cmd = "hostname -I | cut -d\' \' -f1 | head --bytes -1"
        ipString = subprocess.check_output(cmd, shell=True).decode().strip()
        self.ip = f"IP: {ipString}"

        # Get CPU usage
        cmd = "top -bn1 | grep load | awk '{printf \"%.2fLA\", $(NF-2)}'"
        cpuString = subprocess.check_output(cmd, shell=True).decode().strip()
        self.cpu = f"CPU: {cpuString}"

        # Get memory usage
        cmd = "free -m | awk 'NR==2{printf \"%.2f%%\", $3*100/$2 }'"
        memString = subprocess.check_output(cmd, shell=True).decode().strip()
        self.mem_usage = f"MEM: {memString}"

        # Get disk usage
        cmd = "df -h | awk '$NF==\"/\"{printf \"%d/%dGB\", $3,$2}'"
        diskString = subprocess.check_output(cmd, shell=True).decode().strip()
        self.disk = f"DISK: {diskString}"

        # Get temperature
        cmd = "vcgencmd measure_temp | cut -d '=' -f 2 | head --bytes -1"
        tempString = subprocess.check_output(cmd, shell=True).decode().strip()
        self.temperature = f"TEMP: {tempString}"

    def pause(self):
        self.paused = True

    def resume(self):
        self.paused = False


RED = ColorRGBA(r=255.0, g=0.0, b=0.0, a=1.0)
GREEN = ColorRGBA(r=0.0, g=255.0, b=0.0, a=1.0)
BLUE = ColorRGBA(r=0.0, g=0.0, b=255.0, a=1.0)
PURPLE = ColorRGBA(r=128.0, g=0.0, b=128.0, a=1.0)
BLACK = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)

class ScreenMode(Enum):
    MONITOR = 0
    ULTRASONIC = 1
    STATIC = 2

class DisplayManager:
    def __init__(self):
        rospy.init_node('display_manager')
        rospy.loginfo(f"Running Display Manager Node...")

        self.current_color = BLACK
        self.current_text = ''

        self.screen_mode = ScreenMode.MONITOR
        self.button_taps = 0

        self.system_monitor = system_monitor.SystemMonitor()

        self.color_pub = rospy.Publisher('oled_color', ColorRGBA, queue_size=10)
        self.text_pub = rospy.Publisher('oled_text', String, queue_size=10)
        rospy.Subscriber('ultrasonic_data', Int32MultiArray, self.ultrasonic_callback)
        rospy.Subscriber('button_press', Empty, self.button_callback)
        rospy.Timer(rospy.Duration(5), self.publish_system_stats)

    # Callback Methods

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

    def button_callback(self, msg):
        self.button_taps += 1
        self.update_mode()

    # Publish Methods

    def publish_color(self):
        self.color_pub.publish(self.current_color)

    def publish_system_stats(self, event):
        if self.system_monitor.paused:
            return
        self.system_monitor.update()
        stats_array = [
            self.system_monitor.ip,
            self.system_monitor.cpu, 
            self.system_monitor.mem_usage, 
            self.system_monitor.disk,
            self.system_monitor.temperature
        ]
        stats_str = '|'.join(stats_array)
        if self.current_text != stats_str:
            rospy.logdebug(f"Printing Stats:\n {stats_str}")
            self.text_pub.publish(stats_str)
            self.current_text = stats_str

    # Helper Methods

    def static_screen(self):
        self.current_color = PURPLE
        self.publish_color()
        
    def update_mode(self):
        # Prepare for transition.
        if self.screen_mode == ScreenMode.MONITOR:
            self.system_monitor.pause()
        
        # Update mode.
        modes =  list(ScreenMode)
        index = self.button_taps % len(ScreenMode)
        self.screen_mode = modes[index]

        # Mode startup tasks.
        if self.screen_mode == ScreenMode.STATIC:
            self.static_screen()
        elif self.screen_mode == ScreenMode.MONITOR:
            self.current_color = BLACK
            self.publish_color()
            self.system_monitor.resume()
            self.publish_system_stats(None)
        
if __name__ == '__main__':
    try:
        display_manager = DisplayManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
