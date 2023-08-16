#!/usr/bin/env python3
import rclpy
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
bridge = CvBridge()

from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2 #point_cloud2 reads pointcloud info


class projectColor(Node):
    def __init__(self):
        super().__init__('cloud_subscriber')

        # subscriber to the point cloud published by AHC_node
        self.cloudsub = self.create_subscription(PointCloud2, '/lidar_0/AHC/clusters', self.cloud_callback, 10)
        self.cloudsub #only to prevent warning for unused variable?

        # subscriber to zed camera's left camera image
        self.colorsub = self.create_subscription(Image, '/zed2i/zed_node/left/image_rect_color', self.color_callback, 10)
        self.colorsub #only to prevent warning for unused variable?

        # publisher for colored point cloud as '/rgb_points/testing' with (x, y, z, rgb, label) data for each point
        self.publish_colored_pc = self.create_publisher(PointCloud2, '/rgb_points/testing', 10)

        #######################################
        ####    initialising parameters    ####
        #######################################

        # parameters for pixelpick()
        self.cam_w_fov = 1.10 # zed2i camera width fov is 110 degrees
        self.cam_h_fov = 0.70 # zed2i camera height fov is 70 degrees
        self.lidar_w_fov = 1.20 #lidar width fov is 120 degrees
        self.lidar_h_fov = 0.32 # lidar width fov is 32 degrees
        # During initial testing I thought that an arctan function was returning 360 degrees 
        # as 3.60, but it was actually returning radians, this is why the math isn't accurate

        # setting const variables of WAM-V mount
        # the cam is using the left lens, match lengths accordingly
        self.zdiff = 0.04 # diff in height from center of lidar to center of left cam lens (meters)
        self.ydiff = 0.06 # diff in left to right from center of lidar to cam lens (meters)
        self.xdiff = 0    # diff in forward to backward of lidar front center and cam lens front center (meters)

        # parameter for what point clouds contain
        self.fields = [ PointField(name = 'x'    , offset =  0, datatype = PointField.FLOAT32, count = 1), 
                        PointField(name = 'y'    , offset =  4, datatype = PointField.FLOAT32, count = 1),
                        PointField(name = 'z'    , offset =  8, datatype = PointField.FLOAT32, count = 1),
                        PointField(name = 'rgb'  , offset = 12, datatype = PointField.UINT32 , count = 1),
                        PointField(name = 'label', offset = 16, datatype = 1                 , count = 1) ]

    # converts point cloud data to array
    def cloud_callback(self, msg):
        self.xyz = point_cloud2.read_points(msg, field_names=('x','y','z','label'))
        self.get_logger().info('hello from cloud_callback')

        # labeling the field types
        x = self.xyz['x']
        y = self.xyz['y']
        z = self.xyz['z']
        label = self.xyz['label']
        
        try:
            # sends point cloud info to be published
            self.publish_colored_pc(x, y, z, label, msg.width)
        except:
            print('couldnt map colors to point cloud')
        
    # converts image data to an array
    def color_callback(self, msg):
        self.get_logger().info('hello from color_callback')
        self.cv_image = bridge.imgmsg_to_cv2(msg, 'bgra8')

        # get image quality for later use
        self.img_h = self.cv_image.shape[0]
        self.img_w = self.cv_image.shape[1]
        self.init_w = (self.img_w / 2) - 1
        self.init_h = (self.img_h / 2) - 1

    #get rgb for each point, then publish the new point cloud
    def publish_colored_pc(self, x, y, z, label, pc_range):        
        
        #for each point in the point cloud
        pts = []
        for i in range(pc_range):
            #set rgb for this point
            rgb = self.pixelpick(x[i], y[i], z[i])

            #add point with color to array with (x, y, z, rgb, label)
            #adjusting for offset from center of lidar to center of zed2i's left camera
            pts.append([x[i] - self.xdiff, y[i] - self.ydiff, z[i] - self.zdiff, rgb, int(label[i])])

        ##### format copied from Tyler's code, publishing new point cloud #####
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'zed2i_left_camera_frame'
        rgb_processed = point_cloud2.create_cloud(header, self.fields, pts)
        self.get_logger().info('published my new point cloud')
        self.publish_colored_pc.publish(rgb_processed)

        return self
    
    #math finding where the 3D point exists on the 2D image
    #this math is not good, however, produces a nicer outcome than the fixed math
    def pixelpick(self, x, y, z):
        # below was done as debugging, and for whatever reason this works out nicer
        x = x / 2

        if x <= 0: # never divide by 0, but also nothing should be negative
            self.get_logger().warn('Object detected too close, please clean lidar and zed cam')
            rgb_values = (1 << 16) | (0 << 8) | 0
            return rgb_values
        
        # Finds angle between object and lans
        w_theta = np.arctan(y/x)
        h_theta = np.arctan(z/x)

        #calculate pixel dist from center
        if w_theta / self.cam_w_fov > 1:
            rgb_values = (1 << 16) | (0 << 8) | 0
            return rgb_values
        else:
            # REALLY BAD MATH, WHO DOES THIS?
            # was *trying* to calculate the pixel dist by taking the ratio diff between max
            # width angle and actual width angle, and multiplying that ratio by the num of pixels
            # instead I ended up with this garbage
            w_pixel = w_theta / self.cam_w_fov / 2 * self.img_w / 2
        if w_theta / self.cam_w_fov > 1:
            rgb_values = (1 << 16) | (0 << 8) | 0
            return rgb_values
        else:
            # REALLY BAD MATH, WHO DOES THIS?
            # was *trying* to calculate the pixel dist by taking the ratio diff between max
            # height angle and actual height angle, and multiplying that ratio by the num of pixels
            # instead I ended up with this garbage
            h_pixel = h_theta / self.cam_h_fov / 2 * self.img_h / 2

        # make sure out of bounds error doesnt occur
        if w_pixel > 0: 
            w_pixel = w_pixel - 1
        if h_pixel > 0:
            h_pixel = h_pixel - 1

        # finds actual pixel location using init_w and init_h as frame of reference
        new_w = self.init_w - w_pixel
        new_h = self.init_h - h_pixel
        new_w = int(new_w)
        new_h = int(new_h)

        # get rgb value from image
        r = self.cv_image[new_h][new_w][2]
        g = self.cv_image[new_h][new_w][1]
        b = self.cv_image[new_h][new_w][0]
        rgb_value = (r << 16) | (g << 8) | b

        return rgb_value

def main(args=None):
    rclpy.init(args=args)
    rgb_pc_pub = projectColor()
    rclpy.spin(rgb_pc_pub)
    rgb_pc_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
