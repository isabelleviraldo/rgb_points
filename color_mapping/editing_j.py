#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import Point
import numpy as np
from image_geometry import PinholeCameraModel
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs_py import point_cloud2
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header
import timeit


class colorMappingNode(Node):

    def __init__(self):

        super().__init__("color_mapping")
        self.get_logger().info("Node has been started")

        #####################################################################################
        # Declarations
        #####################################################################################
        self.camera_matrix = None
        self.latest_image = None
        self.latest_pcl = None
        # Set flags to synchronize data
        self.need_info = True
        self.need_image = True
        # Declare base image size for later calculations
        self.image_height = 512
        self.image_width = 896
        # CV Bridge to be used later for image processing
        self.bridge = CvBridge()

        # Pinhole Camera model to perform 3d -> 2d projection
        self.model = PinholeCameraModel()

        #####################################################################################
        # Subscriptions
        #####################################################################################

        # Subscriber to gather CameraInfo parameters from left camera
        self.sub_cam_info_left = self.create_subscription(CameraInfo, 'zed2i/zed_node/left/camera_info', self.callback_info_left, 10)

        # Subscriber to an RGB image
        self.sub_image_rgb = self.create_subscription(Image, '/zed2i/zed_node/rgb/image_rect_color', self.callback_image, 10)

        # Subscriber to a point cloud
        self.sub_pcl = self.create_subscription(PointCloud2, 'lidar_0/AHC/clusters', self.callback_pcl, 10)

        #####################################################################################
        # Publishers
        #####################################################################################
        self.pub_pcl = self.create_publisher(
            PointCloud2, '/lidar_0/rgb_points/testing', 10)

        self.get_logger().info("color_mapping node has initialized successfully")

    #####################################################################################
    # Callback functions
    #####################################################################################

    ##########################################################################################
    ### Supposed to time align the image and pointcloud subscription. Currently not working###
    def process_synchronized_data(self):
        # Check if both point cloud and image are available
        if self.latest_image is None or self.latest_pcl is None:
            return

        # Check timestamps to ensure synchronization
        time_diff = abs(self.latest_image_timestamp.sec - self.latest_pcl_timestamp.sec) + \
            abs(self.latest_image_timestamp.nanosec -
                self.latest_pcl_timestamp.nanosec) * 1e-9

        # Set a tolerance (adjust as needed) to consider them synchronized
        synchronization_tolerance = 0.05  # seconds

        if time_diff <= synchronization_tolerance:
            print('success')

    ##########################################################################################
    ### Camera parameter callback -- only to be done one time upon startup###
    def callback_info_left(self, info):
        if self.need_info:
            self.get_logger().info('Inside info callback 1: Gathering left cam info')
            # Get focal point and principal point coordinates from cameraInfo, then turn into matrix
            self.fx = info.k[0]
            self.fy = info.k[4]
            self.cx = info.k[2]
            self.cy = info.k[5]
            self.camera_matrix = np.array(
                [[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]])
            # Same purpose as above, but meant for the PinholeCamera() model
            self.model.fromCameraInfo(info)
            # Reset need_info, gathering camera info only needs to be done once.
            self.need_info = False

    ##########################################################################################
    ### Image Callback###
    def callback_image(self, image):
        self.get_logger().info('Entering image callback')

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(image, 'bgra8')
            self.get_logger().info('Image retrieved successfully')
            self.latest_image_timestamp = self.get_clock().now().to_msg()
        except CvBridgeError as e:
            print("Error converting between ROS Image and OpenCV image:", e)

        self.process_synchronized_data()
        self.need_image = False

    ##########################################################################################
    # Pointcloud callback + processing
    def callback_pcl(self, pcl2_msg):
        self.get_logger().info("Received pcl2")

        self.latest_pcl_timestamp = self.get_clock().now().to_msg()
        self.process_synchronized_data()

        # Extract x, y, z, and label from structured array. Optional: ring, intensity
        self.xyz = point_cloud2.read_points(pcl2_msg, field_names=('x','y','z','label'), reshape_organized_cloud=True)
        #self.get_logger().info('I heard: "%s"' % (self.xyz))

        self.xyz.sort(order='label')

        x = self.xyz['x']
        y = self.xyz['y']
        z = self.xyz['z']
        labels = self.xyz['label']

        # Stack the columns to create array of shape (n, 3)
        lidar_points = np.column_stack((x, y, z, labels))

##########################################################
# Solution 2 -- using image_geometry PinholeCameraModel
##########################################################

        # Filter points based on angle (Currently 50 degrees provides 100%)
        valid_lidar_points = self.filter_by_angle(
            lidar_points)  # Optional param: limiting_angle

        # Obtain u,v pixel coordinates from projection
        pixel_coordinates = self.project3dtoPixel(valid_lidar_points)

        # Validate pixel coordinates to ensure they are inside bounds of image
        valid_pixel_coordinates = self.is_within_bounds(pixel_coordinates)

        # Print filter success rate (filtered_size / original_size)
        self.filter_percentage(
            pixel_coordinates.shape[0], valid_pixel_coordinates.shape[0])

        if not self.need_image:

            # Grab rgb values from all pixel coordinates u,v
            rgb_values = self.grab_pixel_rgb(valid_pixel_coordinates)

            # Publish new point cloud
            self.publish_point_cloud(valid_lidar_points, rgb_values)

    #####################################################################################
    # Supplementary functions
    #####################################################################################

    # Filter out pixel values that are not in bounds of the image height and width
    def is_within_bounds(self, pixel_coords):

        # Use Boolean masking of Numpy for efficient filtering
        mask = (pixel_coords[:, 0] >= 0) & (pixel_coords[:, 0] < self.image_width - 1) & \
            (pixel_coords[:, 1] >= 0) & (pixel_coords[:, 1] < self.image_height - 1)
        valid_pixel_coords = pixel_coords[mask]
        return valid_pixel_coords

    def filter_percentage(self, original_size, filtered_size):
        print('Percentage of lidar points remaining after being filtered:',
              filtered_size / original_size*100, '%')

    # To avoid extra data processing, I noticed all of my points out of bounds were due to
    # the lidar capturing more data horizontally than the image.
    # This is one method of filtering that should be less computationally expensive.

    def filter_by_angle(self, lidar_points, limiting_angle=50):
        angle_array_w = np.arctan2(lidar_points[:, 1], lidar_points[:, 0])
        # This was calculated experimentally
        acceptable_radians = limiting_angle * (np.pi/180)
        valid_indices = np.where(np.logical_and(
            angle_array_w >= -acceptable_radians, angle_array_w <= acceptable_radians))
        valid_lidar_points = lidar_points[valid_indices]
        return valid_lidar_points

    def project3dtoPixel(self, lidar_points):
        valid_pixel_coordinates = []
        for p in lidar_points:
            # To perform the matrix rotation, the columns are input in a different order to save processing power.
            # Apparently works the same way. (Instead of passing x, y, z, I pass (-y, -z, x))
            pixel_coordinates = np.array(
                self.model.project3dToPixel((-p[1], -p[2], p[0])))
            valid_pixel_coordinates.append(pixel_coordinates)

        # Convert the list of pixel coordinates to a NumPy array as integer
        valid_pixel_coordinates = np.around(
            valid_pixel_coordinates).astype(int)

        return valid_pixel_coordinates

    def grab_pixel_rgb(self, pixel_coords):
        # cv_image comes in the shape (height, width, 4)
        # Slicing permits efficient way to grab all pixel values at once
        bgr_values = self.cv_image[pixel_coords[:, 1], pixel_coords[:, 0], 0:3]

        # Convert BGR values to UINT32 RGB values
        r = np.uint32(bgr_values[:, 2])
        g = np.uint32(bgr_values[:, 1])
        b = np.uint32(bgr_values[:, 0])

        # Bitwise operations to combine r, g, b into one value of type UINT32
        rgb_values = (r << 16) | (g << 8) | b

        return rgb_values

    def publish_point_cloud(self, xyz_values, rgb_values):

        fields = [ PointField(name = 'x'    , offset =  0, datatype = PointField.FLOAT32, count = 1), 
                   PointField(name = 'y'    , offset =  4, datatype = PointField.FLOAT32, count = 1),
                   PointField(name = 'z'    , offset =  8, datatype = PointField.FLOAT32, count = 1),
                   PointField(name = 'rgb'  , offset = 12, datatype = PointField.UINT32 , count = 1),
                   PointField(name = 'label', offset = 16, datatype = 1                 , count = 1) ]

        num_points = len(xyz_values)

        # Create structured array with datatypes that we want for rgb image and labels
        points = np.empty(num_points, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.uint32)
        ])

        #Assign values to points message with proper label
        points['x'] = xyz_values[:, 0]
        points['y'] = xyz_values[:, 1]
        points['z'] = xyz_values[:, 2]
        points['rgb'] = rgb_values

        # Create the PointCloud2 message
        msg = PointCloud2()
        msg.header.frame_id = 'lidar_0'
        msg.height = 1
        msg.width = num_points
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16  # 3 (xyz) + 1 (rgb)
        msg.row_step = msg.point_step * num_points
        msg.data = points.tobytes()
        msg.is_dense = True

        # Publish the message
        self.pub_pcl.publish(msg)
        self.get_logger().info('Published point cloud')

        return self


def main(args=None):

    rclpy.init(args=args)
    node = colorMappingNode()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
