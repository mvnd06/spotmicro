#!/usr/bin/env python

from PIL import Image
import rospy, time, os
from std_msgs.msg import ColorRGBA
import board, busio, displayio
from adafruit_ssd1331 import SSD1331
from adafruit_imageload import load as imageLoad
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
        self.splash = displayio.Group()
        self.display.show(self.splash)
        self.color_palette = displayio.Palette(1)
        self.color_bitmap = displayio.Bitmap(96, 64, 1)
        self.color_palette[0] = 0x000000

        # Update display.
        self.bg_sprite = displayio.TileGrid(self.color_bitmap, pixel_shader=self.color_palette, x=0, y=0)
        self.splash.append(self.bg_sprite)

        # Subscribe to `oled_color` topic.        
        self.color_sub = rospy.Subscriber('oled_color', ColorRGBA, self.color_callback)

        while not rospy.is_shutdown():
            self.home_animation()
            rospy.spin()
    
    def color_callback(self, msg):
        rospy.logdebug(f"Recieved -> Red: {msg.r}, Green: {msg.g}, Blue: {msg.b}")
        # Convert RGB values to a single integer.
        color_int = ((int(msg.r) << 16) | (int(msg.g) << 8) | int(msg.b))
        
        # Update the color palette with the new color.
        self.color_palette[0] = color_int
        
        # Redraw the background sprite with the new color.
        self.bg_sprite = displayio.TileGrid(self.color_bitmap, pixel_shader=self.color_palette, x=0, y=0)
        self.splash.pop(0)
        self.splash.insert(0, self.bg_sprite)

    def home_animation(self):
        rospy.loginfo("Starting animation...")

        # Load and display all BMP files in folder
        folder_path = '/home/ubuntu/catkin_ws/src/oled_display/scripts/resources/'
        bmp_files = [f for f in os.listdir(folder_path) if f.endswith('.bmp')]

        for bmp_file in bmp_files:
            with open(os.path.join(folder_path, bmp_file), 'rb') as file:

                bmp, palette = imageLoad(file, bitmap=displayio.Bitmap, palette=displayio.Palette)
                
                sprite = displayio.TileGrid(bmp, pixel_shader=palette, x=0, y=0)
                
                group = displayio.Group(max_size=1)
                group.append(sprite)
                
                display.show(group)
                display.refresh()
                time.sleep(0.1)  # Change this delay to adjust the animation speed



if __name__ == '__main__':
    node = OLEDNode() 