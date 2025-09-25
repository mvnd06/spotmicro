#!/usr/bin/env python3
from luma.core.interface.serial import spi
from luma.oled.device import ssd1331
from PIL import Image, ImageDraw, ImageFont
import time
import math

# --- Setup SPI connection ---
serial = spi(port=0, device=0, gpio_DC=24, gpio_RST=23)  # adjust RST if needed
device = ssd1331(serial, width=96, height=64)

print("🎉 Celebration mode engaged!")

font = ImageFont.load_default()
text = "SUCCESS"

# Use getbbox for text size
bbox = font.getbbox(text)
text_w = bbox[2] - bbox[0]
text_h = bbox[3] - bbox[1]

# Bouncing text
x, y = 0, 0
dx, dy = 2, 2

hue = 0
while True:
    # New frame
    img = Image.new("RGB", (device.width, device.height), "black")
    draw = ImageDraw.Draw(img)

    # Rainbow gradient background
    for i in range(device.height):
        r = int((math.sin(hue + i * 0.1) + 1) * 127)
        g = int((math.sin(hue + i * 0.1 + 2) + 1) * 127)
        b = int((math.sin(hue + i * 0.1 + 4) + 1) * 127)
        draw.line([(0, i), (device.width, i)], fill=(r, g, b))

    # Draw bouncing text
    draw.text((x, y), text, font=font, fill="white")

    device.display(img)

    # Bounce logic
    x += dx
    y += dy
    if x <= 0 or x + text_w >= device.width:
        dx = -dx
    if y <= 0 or y + text_h >= device.height:
        dy = -dy

    hue += 0.2
    time.sleep(0.05)
