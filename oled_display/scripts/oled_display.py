#!/usr/bin/env python

from PIL import Image
import rospy, time, os
from std_msgs.msg import ColorRGBA, String
import board, busio, displayio, terminalio
from adafruit_ssd1331 import SSD1331
from adafruit_display_text import label
import pkg_resources

HOME_ANIMATION_FILE = pkg_resources.resource_filename('oled_display', 'resources/home_animation.bmp')
SPRITE_SIZE = (64, 64)
HOME_ANIMATION_FRAMES = 303

class OLEDNode():
    def __init__(self):
        rospy.init_node('oled_display')
        rospy.loginfo(f"Running OLED Display Node...")

        displayio.release_displays()
        
        # Configure display.
        mosi_pin, clk_pin, reset_pin, cs_pin, dc_pin = board.D10, board.D11, board.D25, board.D8, board.D24
        spi = busio.SPI(clock=clk_pin, MOSI=mosi_pin)
        
        display_bus = displayio.FourWire(spi, command=dc_pin, chip_select=cs_pin, reset=reset_pin)
        self.display = SSD1331(display_bus, width=96, height=64)
        
        # Create the display context.
        self.group = displayio.Group()
        self.display.show(self.group)
        self.color_palette = displayio.Palette(1)
        self.color_bitmap = displayio.Bitmap(96, 64, 1)
        self.color_palette[0] = 0x000000

        # Update display.
        self.bg_sprite = displayio.TileGrid(self.color_bitmap, pixel_shader=self.color_palette, x=0, y=0)
        self.group.append(self.bg_sprite)

        # Subscribe to `oled_display` topics.        
        self.color_sub = rospy.Subscriber('oled_color', ColorRGBA, self.color_callback)
        self.text_sub = rospy.Subscriber('oled_text', String, self.text_callback)

        while not rospy.is_shutdown():
            rospy.spin()
    
    def color_callback(self, msg):
        while len(self.group):
            self.group.pop()
        rospy.logdebug(f"Recieved -> Red: {msg.r}, Green: {msg.g}, Blue: {msg.b}")
        # Convert RGB values to a single integer.
        color_int = ((int(msg.r) << 16) | (int(msg.g) << 8) | int(msg.b))
        
        # Update the color palette with the new color.
        self.color_palette[0] = color_int
        
        # Redraw the background sprite with the new color.
        self.bg_sprite = displayio.TileGrid(self.color_bitmap, pixel_shader=self.color_palette, x=0, y=0)
        self.group.insert(0, self.bg_sprite)

    def text_callback(self, msg):
        while len(self.group):
            self.group.pop()

        topPadding = 5.0
        spacing = 12.0
        y = topPadding

        lines = msg.data.split('|')
        rospy.logdebug(f"Received text: {lines}")
        for line in lines:
            text_area = label.Label(terminalio.FONT, text=line, color=0xFFFF00, x=0, y=y)
            y += spacing
            self.group.append(text_area)

if __name__ == '__main__':
    node = OLEDNode() 