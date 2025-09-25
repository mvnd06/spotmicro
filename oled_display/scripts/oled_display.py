#!/usr/bin/env python

from PIL import Image, ImageDraw, ImageFont
import rospy
from std_msgs.msg import ColorRGBA, String
from luma.core.interface.serial import spi, noop
from luma.oled.device import ssd1331

DISPLAY_WIDTH = 96
DISPLAY_HEIGHT = 64
RESET_PIN = 23


class OLEDNode:
    def __init__(self):
        rospy.init_node('oled_display')
        rospy.loginfo("Running OLED Display Node...")

        serial_interface = spi(port=0, device=0, gpio=noop())
        self.display = ssd1331(serial_interface, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, rst=RESET_PIN)

        self.width = DISPLAY_WIDTH
        self.height = DISPLAY_HEIGHT
        self.font = ImageFont.load_default()
        self.background_color = (0, 0, 0)
        self.show_solid_color(self.background_color)

        self.color_sub = rospy.Subscriber('oled_color', ColorRGBA, self.color_callback)
        self.text_sub = rospy.Subscriber('oled_text', String, self.text_callback)

        while not rospy.is_shutdown():
            rospy.spin()

    def color_callback(self, msg):
        rospy.logdebug(f"Received -> Red: {msg.r}, Green: {msg.g}, Blue: {msg.b}")
        self.background_color = (int(msg.r), int(msg.g), int(msg.b))
        self.show_solid_color(self.background_color)

    def text_callback(self, msg):
        lines = msg.data.split('|')
        rospy.logdebug(f"Received text: {lines}")
        image = Image.new('RGB', (self.width, self.height), self.background_color)
        draw = ImageDraw.Draw(image)

        top_padding = 5
        line_spacing = 12
        y = top_padding

        for line in lines:
            draw.text((0, y), line, font=self.font, fill=(255, 255, 0))
            y += line_spacing

        self.display.display(image)

    def show_solid_color(self, color):
        image = Image.new('RGB', (self.width, self.height), color)
        self.display.display(image)


if __name__ == '__main__':
    OLEDNode()
