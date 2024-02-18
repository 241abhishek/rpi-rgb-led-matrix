#!/usr/bin/env python
import sys
import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/../../..'))
from rgbmatrix import RGBMatrix, RGBMatrixOptions

class Circle():
    def __init__(self, *args, **kwargs):
        super(Circle, self).__init__(*args, **kwargs)

    def create_frame_canvas_from_options(self):
        options = RGBMatrixOptions()
        options.rows = 64
        options.cols = 64
        options.hardware_mapping = 'adafruit-hat'
        options.gpio_slowdown = 4

        self.matrix = RGBMatrix(options = options)
        self.canvas = self.matrix.CreateFrameCanvas()
    
    def display(self, x, y, radius, r, g, b):
        # clear the canvas for the next frame
        self.canvas.Clear()
        # draw a circle with the center at (x, y) and radius r
        for i in range(x - radius, x + radius):
            for j in range(y - radius, y + radius):
                if (i - x)**2 + (j - y)**2 <= radius**2:
                    self.canvas.SetPixel(i, j, r, g, b)
        # swap the canvas to the display the frame
        self.canvas = self.matrix.SwapOnVSync(self.canvas)

    def clear(self):
        # clear the canvas for the next frame
        self.canvas.Clear()
        self.canvas = self.matrix.SwapOnVSync(self.canvas)

class LedControl(Node):
    def __init__(self):
        super().__init__('led_control')
        
        # declare a timer frequency parameter
        self.declare_parameter('timer_frequency', 100.0)
        self.timer_frequency = self.get_parameter('timer_frequency').get_parameter_value().double_value

        # create a timer with a rate
        self.create_timer(1/self.timer_frequency, self.timer_callback)
        
        # create a circle object
        self.circle = Circle()
        self.circle.create_frame_canvas_from_options()

        # set the circle properties
        self.x = 32
        self.y = 32
        self.radius = 10
        # initialize the color of the circle
        self.r = 255
        self.g = 0
        self.b = 0
        self.flag = False
        self.status = False

        # create a subscriber to the light_status topic
        self.light_status_sub = self.create_subscription(Bool, 'light_status', self.light_status_callback, 10)

        # create a subscriber to the circle_color topic
        self.circle_color_srv = self.create_subscription(Point, 'circle_color', self.circle_color_callback, 10)

    def timer_callback(self):
        # display the circle if flag is True
        # self.get_logger().info(f"Running")
        # self.get_logger().info(f"{self.flag=}")
        # if self.flag:
        #     self.circle.display(x=self.x, y=self.y, r=self.r)
        # else:
        #     self.circle.clear()
        pass

    def light_status_callback(self, msg):
        if msg.data:
            # log the msg data
            # self.get_logger().info(f"{msg.data=}")
            # self.flag = 
            self.circle.display(x=self.x, y=self.y, radius=self.radius, r=self.r, g=self.g, b=self.b)
        else:
            # log the msg data
            # self.get_logger().info(f"{msg.data=}")
            # self.flag = False
            self.circle.clear()

    def circle_color_callback(self, msg):
        self.r = msg.x
        self.g = msg.y
        self.b = msg.z
        self.get_logger().info(f"Color changed to ({self.r}, {self.g}, {self.b})")

def main(args=None):
    rclpy.init(args=args)

    led_control = LedControl()

    rclpy.spin(led_control)

    led_control.destroy_node()
    rclpy.shutdown()