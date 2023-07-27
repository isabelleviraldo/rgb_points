import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image

import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
bridge = CvBridge()

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'zed2i/zed_node/rgb_raw/image_raw_color',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('1. I heard: "%s"' % msg.width)
        self.get_logger().info('2. I heard: "%s"' % msg.height)
        self.get_logger().info('3. I heard: "%s"' % msg.encoding)
        self.cv_image = bridge.imgmsg_to_cv2(msg, 'bgra8')
        self.get_logger().info('4. I heard: "%s"' % self.cv_image)

        exit()
        


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()