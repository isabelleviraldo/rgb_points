#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import Point
import numpy as np
import open3d as o3d
from image_geometry import StereoCameraModel, PinholeCameraModel
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs_py import point_cloud2


class colorMappingNode(Node):

    def __init__(self):

        super().__init__("color_mapping")  # MODIFY NAME
        # self.model = StereoCameraModel()
        self.model = PinholeCameraModel()
        self.get_logger().info("Node has been started")
        self.need_info = True
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, 'test_publisher', 10)

        self.sub_cam_info1 = self.create_subscription(CameraInfo,
                                                      'zed2i/zed_node/left/camera_info',
                                                      self.callback_info_left,
                                                      10)

        self.sub_image_rgb = self.create_subscription ( msg_type=Image,
                                                          topic = '/zed2i/zed_node/rgb/image_rect_color',
                                                          callback = self.callback_image,
                                                          qos_profile = 10)
        
        self.sub_pcl = self.create_subscription(
            # message_type
            PointCloud2,
            # 'topic'
            'lidar_0/m1600/pcl2',
            self.callback_pcl, 10
            )

        self.get_logger().info("initialized lidar_pcl2_Subscriber on lidar_0/m1600/pcl2")
        

# For two cameras
#########################################################################################################################################################
        # self.sub_image_depth = self.create_subscription(Image,
        #                                           'zed2i/zed_node/depth/depth_registered',
        #                                           self.callback_image,
        #                                           10)




        # self.sub_cam_info2 = self.create_subscription(CameraInfo,
        #                                               'zed2i/zed_node/right/camera_info',
        #                                               self.callback_info_right,
        #                                               10)        
#########################################################################################################################################################



    def callback_info_left(self, info):
        if self.need_info:
            self.get_logger().info('Inside info callback1: Gathering left cam info')
            self.model.fromCameraInfo(info)
            print(self.model.P)
            self.need_info = False
            

# #Only used for two cameras
#     def callback_info_right(self, info):
#         if self.need_info:
#             self.get_logger().info('Inside info callback2: Gathering right cam info')
#             self.cam2 = info
#             self.model.fromCameraInfo(self.cam1, info)
#             self.model.tfFrame()
#             self.need_info = False
            

    def callback_image(self, image):
        self.get_logger().info('Entering image callback')
    
    def callback_pcl(self, pcl2_msg):
        self.get_logger().info("Received pcl2")

        assert isinstance(pcl2_msg, PointCloud2), \
            'pc2_msg is not a sensor_msgs.msg.PointCloud2'
        
        self.xyz = point_cloud2.read_points(cloud = pcl2_msg,
                                            field_names = ('x', 'y', 'z'),
                                            reshape_organized_cloud=True)
        

        # self.P = Point()
        # self.P.x = self.xyz[0][0].item()
        # self.P.y = self.xyz[0][1].item()
        # self.P.z = self.xyz[0][2].item()
        x = self.xyz['x']
        y = self.xyz['y']
        z = self.xyz['z']

        breakpoint()

        points_3d = np.column_stack((x, y, z))
        breakpoint()

        pixel_coordinates = []
        for i in range(len(self.xyz)):
            u, v = self.model.project3dToPixel((points_3d[i][0], points_3d[i][1], points_3d[i][2]))
            pixel_coordinates.append((u, v))
            breakpoint()
        breakpoint()


def main(args=None):

    rclpy.init(args=args)
    node = colorMappingNode()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
