#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import Image

from multiprocessing import Process
import sys

import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
bridge = CvBridge()

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
import struct

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

class combining(Node):

    def __init__(self):
        super().__init__('cloud_subscriber')
        self.cloudsub = self.create_subscription(PointCloud2, 'lidar_0/m1600/pcl2', self.cloud_callback, 10)
        self.colorsub = self.create_subscription(Image, 'zed2i/zed_node/rgb_raw/image_raw_color', self.color_callback, 10)
        self.cloudsub #only to prevent warning for unused variable?
        self.colorsub #only to prevent warning for unused variable?

        self.pub_testing = self.create_publisher(PointCloud2, 'lidar_0/rgb_points/testing', 10)


    def cloud_callback(self, msg):
        self.xyz = point_cloud2.read_points(msg, field_names=('x','y','z'))
        #self.get_logger().info('I heard: "%s"' % (self.xyz))

        x = self.xyz['x']
        y = self.xyz['y']
        z = self.xyz['z']
        
        try:
            self.create_testing_and_pub(x, y, z, msg.width)
        except:
            print('did exception')
        #exit()

    def color_callback(self, msg):
        self.cv_image = bridge.imgmsg_to_cv2(msg, 'bgra8')
        #breakpoint()
        self.get_logger().info('hello from color_callback')


    def create_testing_and_pub(self, x, y, z, ra):
        fields = [ PointField(name = 'x'    , offset =  0, datatype = PointField.FLOAT32, count = 1), 
                   PointField(name = 'y'    , offset =  4, datatype = PointField.FLOAT32, count = 1),
                   PointField(name = 'z'    , offset =  8, datatype = PointField.FLOAT32, count = 1),
                   PointField(name = 'rgb'  , offset = 12, datatype = PointField.UINT32 , count = 1) ]
        
        #new for loop to add stuff to the list pts
        pts = []
        for i in range(ra):
            rgb = self.pixelpick(x[i], y[i], z[i])
            #breakpoint()
            pts.append([x[i], y[i], z[i], rgb])

        header = Header()
        # header.stamp = rclpy.time.Time.now()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "lidar_0"
        rgb_processed = point_cloud2.create_cloud(header, fields, pts)
        self.get_logger().info('published my new point cloud')
        self.pub_testing.publish(rgb_processed)

        return self
    
    def pixelpick(self, x, y, z):

        #setting const variables of WAM-V mount
        #the cam is using the left lens, match lengths accordingly
        zdiff = 0.04 #diff in height from center of lidar to center of cam lens (meters)
        ydiff = 0.06 #diff in left to right from center of lidar to cam lens (meters)
        xdiff = 0    #diff in forward to backward of lidar front center and cam lens front center (meters)

        img_w = 896
        img_h = 512

        cam_w_fov = 1.10
        cam_h_fov = 0.70

        lidar_w_fov = 1.20
        lidar_h_fov = 0.32

        x = x - xdiff
        x = x / 2
        y = y - ydiff
        z = z - zdiff

        if x <= 0:
            self.get_logger().warn('Object detected too close, please clean lidar and zed cam')
            rgb_values = (1 << 16) | (0 << 8) | 0
            return rgb_values
        
        w_theta = np.arctan(y/x)
        h_theta = np.arctan(z/x)

        if w_theta / cam_w_fov > 1:
            rgb_values = (1 << 16) | (0 << 8) | 0
            return rgb_values
        else:
            y_pixel = w_theta / cam_w_fov / 2 * img_w / 2

        z_pixel = h_theta / cam_h_fov / 2 * img_h / 2

        if y_pixel > 0: 
            y_pixel = y_pixel - 1
        
        if z_pixel > 0:
            z_pixel = z_pixel - 1

        init_w = (896 / 2) - 1
        init_h = (512 / 2) - 1

        new_w = init_w - y_pixel
        new_h = init_h - z_pixel

        new_w = int(new_w)
        new_h = int(new_h)

        r = self.cv_image[new_h][new_w][2]
        g = self.cv_image[new_h][new_w][1]
        b = self.cv_image[new_h][new_w][0]

        rgb_values = (r << 16) | (g << 8) | b

        return rgb_values

def main(args=None):
    rclpy.init(args=args)
    combining_sub = combining()
    rclpy.spin(combining_sub)
    combining_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()