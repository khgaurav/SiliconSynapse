#!/usr/bin/env python3
'''
Terrain classification node using standard TensorFlow model
'''

import rclpy
from rclpy.node import Node
import numpy as np
import os
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import tensorflow as tf

# Use standard non-quantized model
ml_model = 'train2_resnet18_terraset6_float.h5'

class MLPublisher(Node):
    def __init__(self):
        super().__init__('ml_publisher')
        self.subscriber_ = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 10)
        self.get_logger().info('[INFO] __init__, Create Subscription to rgb image...')
        self.subscriber_  # prevent unused variable warning
        self.publisher_ = self.create_publisher(String, 'terrain_class', 10)
        # self.publisher_dist = self.create_publisher(PointStamped, 'terrain_dist', 10)
        
        # Load model using standard TensorFlow
        self.model_path = os.path.join(get_package_share_directory('terra_sense'), 'config', ml_model)
        self.get_logger().info("MODEL="+self.model_path)
        self.model = tf.keras.models.load_model(self.model_path)
        self.get_logger().info('[INFO] Model loaded successfully')
        
        # Print model summary for debugging
        self.model.summary()
        self.get_logger().info('[INFO] __init__ exiting...')

    def calculate_softmax(self, data):
        result = np.exp(data)
        return result

    def listener_callback(self, msg):
        self.get_logger().info("Starting of listener callback...")
        bridge = CvBridge()
        cv2_image_org = bridge.imgmsg_to_cv2(msg,desired_encoding="rgb8")
        y1 = (128)
        y2 = (128+280)
        x1 = (208)
        x2 = (208+280)
        roi_img = cv2_image_org[ y1:y2, x1:x2, : ]
        resized_image = cv2.resize(roi_img, (224, 224), interpolation=cv2.INTER_LINEAR)
        # roi_img_gray=cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
        cv2_image_normal = np.asarray(resized_image/255, dtype=np.float32)
        # cv2_image = np.expand_dims(cv2_image_normal, axis=2)
        
        # Add batch dimension
        input_tensor = np.expand_dims(cv2_image_normal, axis=0)
        
        # Run inference with TensorFlow model
        predictions = self.model.predict(input_tensor)
        
        # Get class prediction
        prediction = np.argmax(predictions[0])
        
        # Publish terrain class
        self.get_logger().info("prediction=" + str(prediction))
        terrain_class_msg = String()
        terrain_class_msg.data = str(prediction)
        self.publisher_.publish(terrain_class_msg)
        
        # # Calculate ROI center
        # roi_center_x = (x1 + x2) / 2
        # roi_center_y = (y1 + y2) / 2
        
        # # Assuming you have the camera intrinsic parameters
        # fx = 500  # Focal length in x direction
        # fy = 500  # Focal length in y direction
        # cx = 320  # Principal point x-coordinate
        # cy = 240  # Principal point y-coordinate
        
        # # Depth value at the center of the ROI
        # depth = 1.0  # This should be obtained from a depth sensor or estimation
        
        # # Calculate the real-world coordinates
        # real_x = (roi_center_x - cx) * depth / fx
        # real_y = (roi_center_y - cy) * depth / fy
        # real_z = depth
        
        # # Create and publish the terrain location message
        # terrain_location_msg = PointStamped()
        # terrain_location_msg.header.frame_id = str(prediction)
        # terrain_location_msg.point.x = real_x
        # terrain_location_msg.point.y = real_y
        # terrain_location_msg.point.z = real_z
        
        # self.publisher_dist.publish(terrain_location_msg)

        
        # DISPLAY
        # cv2_bgr_image = cv2.cvtColor(cv2_image_org, cv2.COLOR_RGB2BGR)
        # cv2.imshow('rosai_demo',cv2_bgr_image)
        # cv2.waitKey(1)
        
        # CONVERT BACK TO ROS & PUBLISH
        # image_ros = bridge.cv2_to_imgmsg(cv2_image)
        # self.publisher_.publish(image_ros)
        # self.get_logger().info("published prediction="+str(prediction))

def main(args=None):
    rclpy.init(args=args)
    node = MLPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
