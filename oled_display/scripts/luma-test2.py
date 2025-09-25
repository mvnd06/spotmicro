from luma.core.interface.serial import spi
from luma.oled.device import ssd1331
from PIL import Image, ImageDraw
import time

serial = spi(port=0, device=0, gpio_DC=24, gpio_RST=25)
device = ssd1331(serial, width=96, height=64)

print("✅ SSD1331 initialized")

# Fill red
img = Image.new("RGB", (device.width, device.height), "red")
device.display(img)
time.sleep(2)

# Fill green
img = Image.new("RGB", (device.width, device.height), "green")
device.display(img)
time.sleep(2)

# Fill blue
img = Image.new("RGB", (device.width, device.height), "blue")
device.display(img)
time.sleep(2)

device.clear()
