#!/usr/bin/env python

import rclpy
import cv2
import message_filters 

from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2



class MinimalSubscriber(Node):
  def __init__(self):
    super().__init__('minimal_subscriber')
    self.publisher_ = self.create_publisher(JointState, 'box_width', 10)
    self.image_sub = message_filters.Subscriber(self,Image,'/camera/color/image_raw', qos_profile=qos_profile_sensor_data)
    self.pc_sub = message_filters.Subscriber(self,PointCloud2,'/camera/depth/color/points')
    self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub,self.pc_sub], 10, 1) 
    self.ts.registerCallback(self.callback)

  def callback(self, img, pc_in):
    print('hello')
    #pc = ros_numpy.numpify(pc_in)
    #points=np.zeros((pc.shape[0],3))
    #points[:,0]=pc['x']
    #points[:,1]=pc['y']
    #points[:,2]=pc['z']
    
    #pcd = o3d.geometry.PointCloud()
    #pcd.pc = o3d.utility.Vector3dVector(pc)
    #o3d.visualization.draw_geometries([pcd])




def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

