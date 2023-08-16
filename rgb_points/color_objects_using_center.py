#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
bridge = CvBridge()

from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2 #point_cloud2 reads pointcloud info

class projectColor(Node):

    def __init__(self):
        super().__init__('cloud_subscriber')

        #######################################
        ####    initialising parameters    ####
        #######################################

        # parameters for pixelpick()
        self.cam_w_fov = 110 # zed2i camera width fov is 110 degrees
        self.cam_h_fov = 70 # zed2i camera height fov is 70 degrees
        self.lidar_w_fov = 120 # lidar width fov is 120 degrees       # unused variable
        self.lidar_h_fov = 32 # lidar width fov is 32 degrees         # unused variable
        
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
        

        # subscriber to the point cloud published by AHC_node
        self.cloudsub = self.create_subscription(PointCloud2, '/lidar_0/AHC/clusters', self.cloud_callback, 10)
        self.cloudsub #only to prevent warning for unused variable?

        # subscriber to zed camera's left camera image
        self.colorsub = self.create_subscription(Image, '/zed2i/zed_node/left/image_rect_color', self.color_callback, 10)
        self.colorsub #only to prevent warning for unused variable?

        # publisher for colored point cloud as '/rgb_points/testing' with (x, y, z, rgb, label) data for each point
        self.publish_colored_pc = self.create_publisher(PointCloud2, '/rgb_points/testing', 10)

    # converts point cloud data to array
    def cloud_callback(self, msg):
        self.xyz = point_cloud2.read_points(msg, field_names=('x','y','z','label'))
        self.get_logger().info('hello from cloud_callback')
        
        # to make it easier to color points later
        self.xyz.sort(order = 'label')

        # labeling the field types
        x = self.xyz['x']
        y = self.xyz['y']
        z = self.xyz['z']
        label = self.xyz['label']
        
        # sends point cloud info to be published
        try:
            self.build_colored_pc(x, y, z, label, msg.width)
        except:
            print('couldnt map colors to point cloud')

    # converts image data to an array
    def color_callback(self, msg):
        self.get_logger().info('hello from color_callback')
        self.cv_image = bridge.imgmsg_to_cv2(msg, 'bgra8')

        # get image quality for later use + setup reference point
        self.img_w = msg.width
        self.img_h = msg.height
        self.init_w = (self.img_w / 2) - 1
        self.init_h = (self.img_h / 2) - 1

    # get rgb for each object from its centerpoint, then publish the new point cloud
    def build_colored_pc(self, x, y, z, label, pc_range):
        
        pts = []
        nowlabel = int(label[0])
        ctr = []

        # get the center points for all the objects
        try: 
            ctr = self.get_center(label)
        except:
            print('error with getting the center')

        # get first object's color
        rgb = self.pixelpick((ctr[0][0] - self.xdiff), (ctr[0][1] - self.ydiff), (ctr[0][2] - self.zdiff))

        # for each point in the point cloud
        for i in range(pc_range):
            
            # grab color once for unique labels, reuse color until new unique label
            if int(label[i]) != nowlabel:
                nowlabel = int(label[i])
                for ele in ctr:
                    if nowlabel == ele[3]:
                        rgb = self.pixelpick((ctr[nowlabel][0] - self.xdiff), (ctr[nowlabel][1] - self.ydiff), (ctr[nowlabel][2] - self.zdiff))

            # add point with color to array with (x, y, z, rgb, label)
            pts.append([x[i], y[i], z[i], rgb, int(label[i])])

        ##### format copied from Tyler's code, publishing new point cloud #####
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'zed2i_left_camera_frame'
        rgb_processed = point_cloud2.create_cloud(header, self.fields, pts)
        self.get_logger().info('published my new point cloud')
        self.publish_colored_pc.publish(rgb_processed)

        return self
    
    # taken from tyler's code
    # goes through all the points in point cloud, and averages the x, y, and z to find centerpoint
    # returns an array with the centerpoints of each object and the label (x, y, z, label)
    def get_center(self, AHC_model_labels):
        label_list = np.unique(AHC_model_labels)
        centerpoints = []  

        # get center point of each cluster
        for label in label_list:
            filtered = [i for i in self.xyz if i[-1] == label ]
            fx = [i[0] for i in filtered]
            fy = [i[1] for i in filtered]
            fz = [i[2] for i in filtered]    
            c = ((min(fx) + max(fx)) / 2, (min(fy) + max(fy)) / 2, (min(fz) + max(fz)) / 2, label)    
            centerpoints.append(c)
        centerpoints = np.array(centerpoints)
        return centerpoints

    # math finding where the 3D point exists on the 2D image
    def pixelpick(self, x, y, z):

        if x <= 0: # never divide by 0, but also nothing should be negative
            print('object too close to lidar, please clean the lens')
            rgb_value = (1 << 16) | (0 << 8) | 0
            return rgb_value

        # Finds angle between object and lens (returns degree)
        h_theta = np.arctan2(z,x)*(180/np.pi)
        w_theta = np.arctan2(y,x)*(180/np.pi)

        # check that point is within bounds of the camera
        if (h_theta/int(self.cam_h_fov/2)) > 1:
            rgb_value = (1 << 16) | (0 << 8) | 0
            return rgb_value
        if (w_theta/int(self.cam_w_fov/2)) > 1:
            rgb_value = (1 << 16) | (0 << 8) | 0
            return rgb_value
        if (h_theta/int(self.cam_h_fov/2)) < -1:
            rgb_value = (1 << 16) | (0 << 8) | 0
            return rgb_value
        if (w_theta/int(self.cam_w_fov/2)) < -1:
            rgb_value = (1 << 16) | (0 << 8) | 0
            return rgb_value

        # Calculates the pixel dist by taking the ratio diff between max angle
        # and actual angle, and multiplying that ratio by the num of pixels
        h_pixel = (h_theta/int(self.cam_h_fov/2))*(self.img_h/2)
        w_pixel = (w_theta/int(self.cam_w_fov/2))*(self.img_w/2)

        # make sure out of bounds error doesnt occur
        if h_pixel > 0:
            h_pixel = h_pixel - 1
        h_pixel = int(h_pixel)
        if w_pixel > 0:
            w_pixel = w_pixel - 1
        w_pixel = int(w_pixel)

        # finds actual pixel location using init_w and init_h as frame of reference
        location_h = int(self.init_h - h_pixel)
        location_w = int(self.init_w - w_pixel)

        # make sure out of bounds error doesnt occur
        # it shouldnt ever get to this point before already detecting it would be 
        # out of bounds, this is just precautionary
        if location_h < 0:
            rgb_value = (0 << 16) | (1 << 8) | 0
            return rgb_value
        if location_w < 0:
            rgb_value = (0 << 16) | (1 << 8) | 0
            return rgb_value
        if location_h >= self.img_h:
            rgb_value = (0 << 16) | (1 << 8) | 0
            return rgb_value
        if location_w >= self.img_w:
            rgb_value = (0 << 16) | (1 << 8) | 0
            return rgb_value
        
        # get rgb value from image
        r = self.cv_image[location_h][location_w][2]
        g = self.cv_image[location_h][location_w][1]
        b = self.cv_image[location_h][location_w][0]

        # ros2 can take a single float type for rgb with this format
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
