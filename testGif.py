import os
import time
import board
import displayio, busio
from PIL import Image
import adafruit_imageload
from adafruit_ssd1331 import SSD1331

# Open the gif file
folder_path = '/home/ubuntu/'
gif_file = 'loader.gif'
gif_path = os.path.join(folder_path, gif_file)
gif_image = Image.open(gif_path)

# Extract individual frames from the gif
frames = []
frame_count = 0
try:
    while True:
        frames.append(gif_image.copy())
        gif_image.seek(len(frames))
        frame_count += 1
except:
    pass

print(f"Loaded {frame_count} frames")

# Create the display
spi = busio.SPI(clock=board.D11, MOSI=board.D10)
tft_cs = board.D8
tft_dc = board.D24
reset = board.D25
display_bus = displayio.FourWire(spi, command=tft_dc, chip_select=tft_cs, reset=reset)
display = SSD1331(display_bus, width=96, height=64)

# Loop through the frames and display them on the screen
for i, frame in enumerate(frames):
    print(f"Displaying frame {i+1} of {len(frames)}")
    
    # Convert the frame to a format that can be displayed on the screen
    frame = frame.convert("RGB")
    frame = frame.transpose(method=Image.TRANSPOSE)

    # Load the frame onto the screen
    bitmap, palette = adafruit_imageload.load(frame.tobytes(), bitmap=displayio.Bitmap, palette=displayio.Palette)

    print("Creating sprite")
    sprite = displayio.TileGrid(bitmap, pixel_shader=palette)
    group = displayio.Group(max_size=1)
    group.append(sprite)

    display.show(group)
    display.refresh()

    # Wait for the next frame
    time.sleep(0.1)

print("Animation complete")
