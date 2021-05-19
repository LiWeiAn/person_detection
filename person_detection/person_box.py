#!/usr/bin/env python

import rclpy
import cv2
import message_filters
import numpy as np
from rclpy.node import Node
from .detection_node import load_model
from .detection_node import detection
from .detection_node import show_inference
from .detection_node import get_boxes

from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from box_msgs.msg import Box


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.publisher_ = self.create_publisher(Box, 'box_coordinates', 10)
        self.subscription = self.create_subscription(Image,'serena/camera/right/color/image_raw',  self.detect_callback, qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        self.detection_model = load_model()
        self.bridge = CvBridge()
        

    def detect_callback(self, msg):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        img = show_inference(self.detection_model, cv_image)        
        person_box = get_boxes(self.detection_model, cv_image)
        #print (person_box[1])

	
        box = Box()
        
        t = self.get_clock().now()
        #box.header.stamp = t.to_msg()
        box.header.stamp = msg.header.stamp
        box.l = float(person_box[0])
        box.r = float(person_box[1])
        box.t = float(person_box[2])
        box.b = float(person_box[3])
        

        self.publisher_.publish(box)


        #Display the frames
        cv2.imshow('frame',img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            



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
