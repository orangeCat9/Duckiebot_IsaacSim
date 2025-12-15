#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageCompressor(Node):

    def __init__(self):
        super().__init__('image_compressor_node')
        
        # 설정: 입력 토픽(원본) / 출력 토픽(압축)
        self.input_topic = '/duckie/camera/image_raw'
        self.output_topic = '/duckie/camera/image_raw/compressed'
        
        self.subscription = self.create_subscription(
            Image, self.input_topic, self.listener_callback, 10)
        self.publisher_ = self.create_publisher(
            CompressedImage, self.output_topic, 10)
            
        self.bridge = CvBridge()
        self.get_logger().info(f'압축 노드 시작! {self.input_topic} -> {self.output_topic}')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # JPG 압축 (화질 50%)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
            result, encimg = cv2.imencode('.jpg', cv_image, encode_param)
            
            if result:
                msg_compressed = CompressedImage()
                msg_compressed.header = msg.header
                msg_compressed.format = "jpeg"
                msg_compressed.data = np.array(encimg).tobytes()
                self.publisher_.publish(msg_compressed)
                
        except Exception as e:
            self.get_logger().error(f'에러: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageCompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
