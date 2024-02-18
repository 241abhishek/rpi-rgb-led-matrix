#!/usr/bin/env python
import sys
import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import Bool

sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/../../..'))
from rgbmatrix import RGBMatrix, RGBMatrixOptions
from PIL import Image

class ImageViewer():
    def __init__(self, *args, **kwargs):
        super(ImageViewer, self).__init__(*args, **kwargs)

    def create_frame_canvas_from_options(self):
        options = RGBMatrixOptions()
        options.rows = 64
        options.cols = 64
        options.hardware_mapping = 'adafruit-hat'
        options.gpio_slowdown = 4
        options.drop_privileges=False

        self.matrix = RGBMatrix(options = options)
        self.canvas = self.matrix.CreateFrameCanvas()

    def display_image(self):
        # clear the canvas for the next frame
        self.canvas.Clear()

        # # open the image
        image = Image.open('/home/msr/images/Northwestern-Wildcats-Logo.jpg')

        # Make image fit our screen.
        image.thumbnail((self.matrix.width, self.matrix.height), Image.ANTIALIAS)

        self.matrix.SetImage(image.convert('RGB'))

        # swap the canvas to the display the frame
        # self.canvas = self.matrix.SwapOnVSync(self.canvas)

    def clear(self):
        # clear the canvas for the next frame
        self.canvas.Clear()
        self.canvas = self.matrix.SwapOnVSync(self.canvas)

class LedControl(Node):
    def __init__(self):
        super().__init__('led_control')
        
        # declare a timer frequency parameter
        self.declare_parameter('timer_frequency', 1000.0)
        self.timer_frequency = self.get_parameter('timer_frequency').get_parameter_value().double_value

        # create a timer with a rate
        self.create_timer(1/self.timer_frequency, self.timer_callback)

        # create an image viewer object
        self.image_viewer = ImageViewer()
        self.image_viewer.create_frame_canvas_from_options()

        # set the image viewer properties
        self.flag = False

        # create a subscriber to the light_status topic
        self.light_status_sub = self.create_subscription(Bool, 'light_status', self.light_status_callback, 10)

    def timer_callback(self):
        # display the image if flag is True
        # self.get_logger().info(f"Running")
        # self.get_logger().info(f"{self.flag=}")
        if self.flag:
            self.image_viewer.display_image()
        else:
            self.image_viewer.clear()

    def light_status_callback(self, msg):
        if msg.data:
            # log the msg data
            # self.get_logger().info(f"{msg.data=}")
            self.flag = True
        else:
            # log the msg data
            # self.get_logger().info(f"{msg.data=}")
            self.flag = False
    

def main(args=None):
    rclpy.init(args=args)

    led_control = LedControl()

    rclpy.spin(led_control)

    led_control.destroy_node()
    rclpy.shutdown()