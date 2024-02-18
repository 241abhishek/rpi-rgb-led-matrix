#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
# from std_msgs.msg import String

from led_control.samplebase import SampleBase

class Test(SampleBase):
    def __init__(self, *args, **kwargs):
        super(Test, self).__init__(*args, **kwargs)

    def create_frame_canvas(self):    
        self.offset_canvas = self.matrix.CreateFrameCanvas()

    def run(self, x, y):
        # while True:
            # for x in range(0, self.matrix.width):
        self.offset_canvas.Clear()
        for i in range(x - 5, x + 5):
            for j in range(y - 5, y + 5):
                self.offset_canvas.SetPixel(i, j, 255, 0, 0)
        self.offset_canvas = self.matrix.SwapOnVSync(self.offset_canvas)

class LedControl(Node):
    def __init__(self):
        super().__init__('led_control')
        self.create_timer(1/10000.0, self.timer_callback)
        self.test = Test()
        self.test.process()
        self.test.create_frame_canvas()
        self.x = 32
        self.y = 32

        # create a service to display pixel
        self.service = self.create_service(Empty, 'pixel_location', self.pixel_location_callback)

    def timer_callback(self):
        self.test.run(x=self.x, y=self.y)

    def pixel_location_callback(self, request, response):
        self.get_logger().info('Changing pixel location')
        self.x = 45
        self.y = 45
        return response

def main(args=None):
    rclpy.init(args=args)

    led_control = LedControl()

    rclpy.spin(led_control)

    led_control.destroy_node()
    rclpy.shutdown()

# Main function
if __name__ == "__main__":
    main()