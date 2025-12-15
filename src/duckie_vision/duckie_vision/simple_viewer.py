#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import qos_profile_sensor_data # QoS 문제 해결
import cv2
import numpy as np

class SimpleViewer(Node):

    def __init__(self):
        super().__init__('simple_viewer_node')
        
        # 👇 여기가 핵심입니다. 우리가 보고 싶은 '빨간 박스' 채널입니다.
        self.target_topic = '/duckie/detection/debug/compressed'
        
        self.subscription = self.create_subscription(
            CompressedImage,
            self.target_topic,
            self.listener_callback,
            qos_profile_sensor_data) # 연결 끊김 방지
            
        self.get_logger().info(f'[{self.target_topic}] 뷰어 시작! 화면을 기다리는 중...')

    def listener_callback(self, msg):
        try:
            # RQT 따위 필요 없이 OpenCV로 직접 풉니다
            np_arr = np.frombuffer(msg.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # 화면 띄우기
            cv2.imshow("Red Cube Detection Viewer", image_np)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'화면 오류: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
