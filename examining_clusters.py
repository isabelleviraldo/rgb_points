import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from sensor_msgs_py import point_cloud2


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(PointCloud2, 'lidar_0/AHC/clusters', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, cloud):
        self.xyz = point_cloud2.read_points(cloud, field_names=('x','y','z','label'))

        x = self.xyz['x']
        y = self.xyz['y']
        z = self.xyz['z']
        labels = self.xyz['label']

        try:
            print(self.get_center(list(labels)))
        except:
            print('did exception')
        
        #self.get_logger().info('I Heard: "%s"' % (marker))
        #exit()

    def get_center(self, AHC_model_labels):
        label_list = np.unique(AHC_model_labels)
        centerpoints = []  

        # get centroid of each cluster
        for label in label_list:
            filtered = [i for i in self.xyz if i[-1] == label ]
            fx = [i[0] for i in filtered]
            fy = [i[1] for i in filtered]
            fz = [i[2] for i in filtered]    
            c = ((min(fx) + max(fx)) / 2, (min(fy) + max(fy))/2, (min(fz) + max(fz))/2, label)    
            centerpoints.append(c)
            ## get centroids with mean of points
            # c4 = (sum(fx)/ len(fx), sum(fy)/len(fy), sum(fz)/len(fz), label)
            # c4s.append(c4)
        centerpoints = np.array(centerpoints)
        print(centerpoints)
        breakpoint()
        return self, centerpoints
    


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



