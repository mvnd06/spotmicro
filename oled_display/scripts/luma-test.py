#!/usr/bin/env python3
from luma.core.interface.serial import spi
from luma.oled.device import ssd1331
from PIL import Image, ImageDraw, ImageFont
import time

# Setup SPI (default RPi pins: CE0 -> CS, SCLK -> CLK, MOSI -> DIN)
serial = spi(port=0, device=0, gpio_DC=24, gpio_RST=23, bus_speed_hz=8000000)
device = ssd1331(serial, width=96, height=64)

print("✅ SSD1331 initialized via luma.oled")

# Create a new blank image (RGB mode)
img = Image.new("RGB", (device.width, device.height), "black")
draw = ImageDraw.Draw(img)

# Draw background
draw.rectangle(device.bounding_box, outline="blue", fill="black")

# Draw some text
font = ImageFont.load_default()
draw.text((5, 25), "Hello World!", font=font, fill="yellow")

# Push image to display
device.display(img)

# Keep it up for 5 seconds
time.sleep(5)

# Clear the screen
device.clear()
