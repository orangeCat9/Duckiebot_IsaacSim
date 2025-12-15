#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np

class SmartDriver(Node):

    def __init__(self):
        super().__init__('smart_driver')

        # 1. 구독 및 발행
        self.sub = self.create_subscription(
            CompressedImage,
            '/duckie/camera/image_raw/compressed',
            self.image_callback,
            qos_profile_sensor_data
        )
        self.pub_cmd = self.create_publisher(Twist, '/duckie/cmd_vel', 10)
        self.pub_debug = self.create_publisher(CompressedImage, '/duckie/smart/debug/compressed', 10)

        # 2. 파라미터 설정
        self.max_speed = 0.2     # 최고 속도 (안전을 위해 낮춤)
        self.turn_gain = 0.01    # 회전 민감도
        self.stop_area = 50000   # 목표물 도착 기준 면적
        
        self.get_logger().info('🚦 스마트 드라이버 시작! (초록=전진, 빨강=정지)')

    def image_callback(self, msg):
        try:
            # 이미지 변환
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            h, w = frame.shape[:2]

            # BGR -> HSV 변환
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # --- 1. 빨간색(정지 신호) 찾기 ---
            # 빨간색은 스펙트럼 양끝에 걸쳐 있어서 두 범위를 합쳐야 함
            lower_red1 = np.array([0, 100, 100]); upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100]); upper_red2 = np.array([180, 255, 255])
            mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
            
            # --- 2. 초록색(출발 신호) 찾기 ---
            lower_green = np.array([35, 100, 100]); upper_green = np.array([85, 255, 255])
            mask_green = cv2.inRange(hsv, lower_green, upper_green)

            # --- 3. 상황 판단 및 제어 ---
            twist = Twist()
            debug_frame = frame.copy()

            # (1) 빨간색이 감지되면 무조건 정지! 🛑
            contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            red_detected = False
            
            if contours_red:
                largest_red = max(contours_red, key=cv2.contourArea)
                if cv2.contourArea(largest_red) > 500: # 노이즈 무시
                    red_detected = True
                    # 빨간 박스 그리기
                    x, y, bw, bh = cv2.boundingRect(largest_red)
                    cv2.rectangle(debug_frame, (x, y), (x+bw, y+bh), (0, 0, 255), 3)
                    cv2.putText(debug_frame, "STOP SIGNAL!", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            if red_detected:
                # 빨간색이 보이면 무조건 멈춤
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info('🛑 빨간불 발견! 정지합니다.', once=True)

            else:
                # (2) 빨간색이 없으면 초록색 찾기 🟢
                contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                if contours_green:
                    largest_green = max(contours_green, key=cv2.contourArea)
                    area = cv2.contourArea(largest_green)

                    if area > 300: # 초록색이 유의미하게 클 때
                        x, y, bw, bh = cv2.boundingRect(largest_green)
                        cx = x + bw // 2
                        
                        # 초록 박스 그리기
                        cv2.rectangle(debug_frame, (x, y), (x+bw, y+bh), (0, 255, 0), 3)
                        cv2.putText(debug_frame, "GO!", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                        # --- 주행 로직 (따라가기) ---
                        err = cx - (w / 2)
                        
                        # 회전 제어 (부드럽게)
                        twist.angular.z = float(self.turn_gain * err)

                        # 전진 제어 (너무 가까우면 멈춤)
                        if area < self.stop_area:
                            twist.linear.x = self.max_speed
                        else:
                            twist.linear.x = 0.0 # 도착
                            
                        self.get_logger().info('🟢 초록불! 주행 중...', once=True)
                else:
                    # 아무 색도 안 보이면 정지
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

            # 명령 전송
            self.pub_cmd.publish(twist)

            # 디버그 화면 송출
            msg_debug = CompressedImage()
            msg_debug.header = msg.header
            msg_debug.format = "jpeg"
            msg_debug.data = np.array(cv2.imencode('.jpg', debug_frame)[1]).tobytes()
            self.pub_debug.publish(msg_debug)

        except Exception as e:
            self.get_logger().error(f'에러 발생: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SmartDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
