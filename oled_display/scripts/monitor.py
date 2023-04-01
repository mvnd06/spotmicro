from PIL import Image, ImageDraw, ImageFont
import time
import board, busio, digitalio, displayio, terminalio
from adafruit_ssd1331 import SSD1331
from adafruit_display_text import label

import subprocess


# Define the Reset Pin
oled_reset = digitalio.DigitalInOut(board.D25)

# Display Parameters
WIDTH = 96
HEIGHT = 64
BORDER = 5

# Display Refresh
LOOPTIME = 1.0

# Configure display.
mosi_pin, clk_pin, reset_pin, cs_pin, dc_pin = board.D10, board.D11, board.D25, board.D8, board.D24
spi = busio.SPI(clock=clk_pin, MOSI=mosi_pin)

display_bus = displayio.FourWire(spi, command=dc_pin, chip_select=cs_pin, reset=reset_pin)
display = SSD1331(display_bus, width=96, height=64)

group = displayio.Group()
display.show(group)

# Define font and size
font = terminalio.FONT

# Define text for 5 lines
line1 = "Line 1"
line2 = "Line 2"
line3 = "Line 3"
line4 = "Line 4"
line5 = "Line 5"

# Define y positions for 5 lines
y1 = 0
y2 = 12
y3 = 24
y4 = 36
y5 = 48

# Create 5 text areas with different y positions
text_area1 = label.Label(font, text=line1, color=0xFFFF00, x=0, y=y1)
group.append(text_area1)

text_area2 = label.Label(font, text=line2, color=0xFFFF00, x=0, y=y2)
group.append(text_area2)

text_area3 = label.Label(font, text=line3, color=0xFFFF00, x=0, y=y3)
group.append(text_area3)

text_area4 = label.Label(font, text=line4, color=0xFFFF00, x=0, y=y4)
group.append(text_area4)

text_area5 = label.Label(font, text=line5, color=0xFFFF00, x=0, y=y5)
group.append(text_area5)


# # Create blank image for drawing.
# # Make sure to create image with mode '1' for 1-bit color.
# width = display.width
# height = display.height
# image = Image.new('1', (width, height))

# # Get drawing object to draw on image.
# draw = ImageDraw.Draw(image)

# # Draw a black filled box to clear the image.
# draw.rectangle((0,0,width,height), outline=0, fill=0)

# # Draw some shapes.
# # First define some constants to allow easy resizing of shapes.
# padding = -2
# top = padding
# bottom = height-padding
# # Move left to right keeping track of the current x position for drawing shapes.
# x = 0


# # Load default font.
# font = ImageFont.load_default()

# # Alternatively load a TTF font.  Make sure the .ttf font file is in the same directory as the python script!
# # Some other nice fonts to try: http://www.dafont.com/bitmap.php
# # Icons website: https://icons8.com/line-awesome
# font = ImageFont.truetype('PixelOperator.ttf', 16)
# icon_font= ImageFont.truetype('lineawesome-webfont.ttf', 18)

# while True:
    # Draw a black filled box to clear the image.
    # draw.rectangle((0,0,width,height), outline=0, fill=0)

    # Shell scripts for system monitoring from here
    # https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
cmd = "hostname -I | cut -d\' \' -f1 | head --bytes -1"
IP = subprocess.check_output(cmd, shell = True )

cmd = "top -bn1 | grep load | awk '{printf \"%.2fLA\", $(NF-2)}'"
CPU = subprocess.check_output(cmd, shell = True )

cmd = "free -m | awk 'NR==2{printf \"%.2f%%\", $3*100/$2 }'"    
MemUsage = subprocess.check_output(cmd, shell = True )

cmd = "df -h | awk '$NF==\"/\"{printf \"HDD: %d/%dGB %s\", $3,$2,$5}'"
cmd = "df -h | awk '$NF==\"/\"{printf \"%d/%dGB\", $3,$2}'"
Disk = subprocess.check_output(cmd, shell = True )

cmd = "vcgencmd measure_temp | cut -d '=' -f 2 | head --bytes -1"
Temperature = subprocess.check_output(cmd, shell = True )
print(IP, CPU, MemUsage, Disk, Temperature)

#    # Icons
#     # Icon temperature
#     draw.text((x, top+5),    chr(62609),  font=icon_font, fill=255)
#     # Icon memory
#     draw.text((x+65, top+5), chr(62776),  font=icon_font, fill=255)
#     # Icon disk
#     draw.text((x, top+25), chr(63426),  font=icon_font, fill=255)
#     # Icon cpu
#     draw.text((x+65, top+25), chr(62171), font=icon_font, fill=255)
#     # Icon wifi
#     draw.text((x, top+45), chr(61931),  font=icon_font, fill=255)

#    # Text
#     # Text temperature
#     draw.text((x+19, top+5), str(Temperature,'utf-8'),  font=font, fill=255)
#     # Text memory usage
#     draw.text((x+87, top+5), str(MemUsage,'utf-8'),  font=font, fill=255)
#     # Text Disk usage
#     draw.text((x+19, top+25), str(Disk,'utf-8'),  font=font, fill=255)
#     # Text cpu usage
#     draw.text((x+87, top+25), str(CPU,'utf-8'), font=font, fill=255)
#     # Text IP address
#     draw.text((x+19, top+45), str(IP,'utf-8'),  font=font, fill=255)
    
#    # Display image.
#     display.image(image)
#     display.show()
#     time.sleep(LOOPTIME)
