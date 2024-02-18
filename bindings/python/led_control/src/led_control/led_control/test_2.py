#!/usr/bin/env python
import time
import sys
import os
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/../../..'))
from rgbmatrix import RGBMatrix, RGBMatrixOptions
from PIL import Image

# if len(sys.argv) < 2:
#     sys.exit("Require an image argument")
# else:
#     image_file = sys.argv[1]

image_file = '/home/msr/rpi-rgb-led-matrix/bindings/python/led_control/src/led_control/led_control/images/Northwestern_Wildcats_logo.svg.png'
image = Image.open(image_file)

# Configuration for the matrix
options = RGBMatrixOptions()
options.rows = 64
options.cols = 64
options.hardware_mapping = 'adafruit-hat'
options.gpio_slowdown = 4

matrix = RGBMatrix(options = options)

# Make image fit our screen.
image.thumbnail((matrix.width, matrix.height), Image.ANTIALIAS)

matrix.SetImage(image.convert('RGB'))

try:
    print("Press CTRL-C to stop.")
    while True:
        time.sleep(100)
except KeyboardInterrupt:
    sys.exit(0)