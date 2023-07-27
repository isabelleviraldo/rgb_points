import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import MarkerArray


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            MarkerArray,
            'lidar_0/AHC/centroids',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, marker):
        #self.get_logger().info('I heard: "%s"' % (point_cloud2.read_points(cloud, field_names=('x','y','z','label'))))
        self.get_logger().info('I Heard: "%s"' % (marker))
        breakpoint()


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



