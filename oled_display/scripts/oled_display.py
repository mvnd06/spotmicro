#!/usr/bin/env python

import rospy
from std_msgs.msg import ColorRGBA
import board, busio, displayio
from adafruit_ssd1331 import SSD1331

class OLEDNode():
    def __init__(self):
        rospy.init_node('oled_display')
        self.color_sub = rospy.Subscriber('oled_color', ColorRGBA, self.color_callback)
        
        displayio.release_displays()
        rospy.loginfo(f"Running OLED Display Node...")
        
        # Configure Display
        mosi_pin, clk_pin, reset_pin, cs_pin, dc_pin = board.D10, board.D11, board.D25, board.D8, board.D24
        spi = busio.SPI(clock=clk_pin, MOSI=mosi_pin)
        
        display_bus = displayio.FourWire(spi, command=dc_pin, chip_select=cs_pin, reset=reset_pin)
        
        self.display = SSD1331(display_bus, width=96, height=64)
        
        # Make the display context
        self.splash = displayio.Group()
        self.display.show(self.splash)
        
        self.color_bitmap = displayio.Bitmap(96, 64, 1)
        self.color_palette = displayio.Palette(1)
        self.color_palette[0] = 0x000000  # Black
        
        self.bg_sprite = displayio.TileGrid(self.color_bitmap, pixel_shader=self.color_palette, x=0, y=0)
        self.splash.append(self.bg_sprite)
        
        # Draw a smaller inner rectangle
        self.inner_bitmap = displayio.Bitmap(86, 54, 1)
        self.inner_palette = displayio.Palette(1)
        self.inner_palette[0] = 0x000000  # Black
        self.inner_sprite = displayio.TileGrid(self.inner_bitmap, pixel_shader=self.inner_palette, x=5, y=5)
        self.splash.append(self.inner_sprite)
                
        while not rospy.is_shutdown():
            rospy.spin()
    
    def color_callback(self, msg):
        self.color_palette[0] = ((msg.r << 16) | (msg.g << 8) | msg.b)
        self.bg_sprite = displayio.TileGrid(self.color_bitmap, pixel_shader=self.color_palette, x=0, y=0)
        self.splash.append(self.bg_sprite)
        self.display.show(self.splash)

if __name__ == '__main__':
    node = OLEDNode()
