#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
import math

class CircleDriver(Node): # 클래스 이름도 CircleDriver로 변경했습니다.

    def __init__(self):
        super().__init__('circle_driver')

        # 1) 카메라 구독
        self.sub = self.create_subscription(
            CompressedImage,
            '/duckie/camera/image_raw/compressed',
            self.image_callback,
            qos_profile_sensor_data
        )

        # 2) 디버그 영상 publish
        self.pub_debug = self.create_publisher(
            CompressedImage,
            '/duckie/detection/debug/compressed',
            10
        )

        # 3) cmd_vel publish (행동 제어)
        self.pub_cmd = self.create_publisher(
            Twist,
            '/duckie/cmd_vel',
            10
        )

        # ---- 제어 파라미터 ----
        self.kp_ang = 0.02         # 회전 계수
        self.max_ang = 1.0         # 회전 속도 제한
        self.max_lin = 0.3         # 전진 속도
        
        self.min_area = 100        # 노이즈 제거용 최소 크기
        self.stop_area = 45000     # 정지 기준 크기 (가까움)

        self.get_logger().info('🟢⚪🔴 원형(Circle) 감지 드라이버 시작!')
        self.get_logger().info('🔴빨강 원: 정지 | 🟢초록 원: 따라가기')

    def is_circle(self, contour):
        """ 윤곽선이 원에 가까운지 판별 (1.0 = 완벽한 원) """
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0: return False
        area = cv2.contourArea(contour)
        
        # 원형도 공식: (4 * pi * Area) / (Perimeter^2)
        circularity = 4 * math.pi * area / (perimeter * perimeter)
        
        # 0.75 이상이면 둥근 물체로 인정 (정사각형은 약 0.785이나 노이즈 고려)
        return circularity > 0.75

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame_bgr is None: return

            h, w = frame_bgr.shape[:2]
            hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
            
            twist = Twist()
            debug_frame = frame_bgr.copy()

            # 노이즈 제거 커널
            kernel = np.ones((5, 5), np.uint8)

            # ==========================================
            # 1. 🔴 빨간색 원 감지 (최우선 순위 - 정지)
            # ==========================================
            lower_red1 = np.array([0, 100, 100]);  upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100]); upper_red2 = np.array([180, 255, 255])
            mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
            
            contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            red_detected = False
            for c in contours_red:
                area = cv2.contourArea(c)
                if area > self.min_area and self.is_circle(c): 
                    red_detected = True
                    # 디버그 표시
                    x, y, bw, bh = cv2.boundingRect(c)
                    cv2.rectangle(debug_frame, (x, y), (x+bw, y+bh), (0, 0, 255), 3)
                    cv2.putText(debug_frame, "RED CIRCLE!", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    break 

            if red_detected:
                # 빨간 원 발견 -> 정지
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.pub_cmd.publish(twist)
                self.publish_debug(msg, debug_frame)
                self.get_logger().info("🔴 빨간 원 발견! 정지!", once=True)
                return 

            # ==========================================
            # 2. 🟢 초록색 원 감지 (추적)
            # ==========================================
            lower_green = np.array([35, 100, 100]); upper_green = np.array([85, 255, 255])
            mask_green = cv2.inRange(hsv, lower_green, upper_green)
            mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
            
            contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            target_green = None
            max_area = 0
            
            for c in contours_green:
                area = cv2.contourArea(c)
                if area > self.min_area and area > max_area and self.is_circle(c):
                    max_area = area
                    target_green = c

            if target_green is not None:
                # 초록 원 발견 -> 추적
                x, y, bw, bh = cv2.boundingRect(target_green)
                cx = x + bw // 2
                
                # 디버그 표시
                cv2.rectangle(debug_frame, (x, y), (x+bw, y+bh), (0, 255, 0), 3)
                cv2.putText(debug_frame, "GREEN CIRCLE!", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # --- 주행 로직 ---
                err = cx - (w / 2.0)
                
                # 회전 제어 (Action Graph가 정상이면 마이너스(-) 사용)
                ang = -self.kp_ang * err 
                twist.angular.z = float(np.clip(ang, -self.max_ang, self.max_ang))

                # 전진 제어
                if max_area > self.stop_area:
                    twist.linear.x = 0.0 # 도착
                    twist.angular.z = 0.0
                else:
                    twist.linear.x = self.max_lin
                    if abs(err) > 100: # 너무 옆에 있으면 일단 회전부터
                        twist.linear.x = 0.0

            else:
                # ==========================================
                # 3. 아무것도 없음 -> 탐색 모드
                # ==========================================
                twist.linear.x = 0.0
                twist.angular.z = 0.5  # 제자리 회전 탐색
                self.get_logger().info("탐색 중... 👀", once=True)

            self.pub_cmd.publish(twist)
            self.publish_debug(msg, debug_frame)

        except Exception as e:
            self.get_logger().error(f'에러 발생: {e}')

    def publish_debug(self, src_msg, debug_frame):
        msg_debug = CompressedImage()
        msg_debug.header = src_msg.header
        msg_debug.format = "jpeg"
        msg_debug.data = np.array(cv2.imencode('.jpg', debug_frame)[1]).tobytes()
        self.pub_debug.publish(msg_debug)

def main(args=None):
    rclpy.init(args=args)
    node = CircleDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
