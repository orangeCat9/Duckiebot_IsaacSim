#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np

class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection_node')

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

        # ---- 제어 파라미터(필요하면 조절) ----
        self.kp_ang = 0.008          # 화면 픽셀 오차 -> 회전
        self.max_ang = 1.5           # 회전 속도 제한
        self.max_lin = 0.5          # 전진 속도 상한
	#전진속도가 회전속도보다 빠르면 자율주행 할때 회전을 잘 못함 
        self.min_area = 20          # 검출 인정 최소 면적
        self.slow_area = 40000       # 감속 시작 면적
        self.stop_area = 45000       # 정지 면적(가까움)

        self.get_logger().info('5번 완성: 🟥 빨간 큐브를 따라갑니다!')

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame_bgr is None:
                return

            h, w = frame_bgr.shape[:2]

            # BGR -> HSV
            hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

            # ✅ 빨간색 HSV thresholding (2구간)
            lower1 = np.array([0,   120, 70])
            upper1 = np.array([10,  255, 255])
            lower2 = np.array([160, 120, 70])
            upper2 = np.array([179, 255, 255])

            mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)

            # 노이즈 제거
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            debug_frame = frame_bgr.copy()
            twist = Twist()

            if not contours:
                # [수정 전] 못 찾으면 정지
                # twist.linear.x = 0.0
                # twist.angular.z = 0.0

                # [수정 후] 제자리에서 천천히 돌면서 찾기 (Search Mode)
                twist.linear.x = 0.0      # 전진은 멈추고
                twist.angular.z = 0.5     # 천천히 좌회전 (속도는 0.3 ~ 0.5 추천)
                
                # (선택사항) 터미널에 로그 띄우기 (도배 방지 위해 once=True 사용 추천)
                self.get_logger().info("빨간 큐브가 없는디용 ㅠ", once=True)

                self.pub_cmd.publish(twist)
                self.publish_debug(msg, debug_frame)
                return

            # 너무 큰 덩어리(바닥 등) 제거
            max_area = 0.6 * (w * h)

            candidates = []
            for c in contours:
                area = cv2.contourArea(c)
                if self.min_area < area < max_area:
                    candidates.append(c)

            if not candidates:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.pub_cmd.publish(twist)

                self.publish_debug(msg, debug_frame)
                return

            largest = max(candidates, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            x, y, bw, bh = cv2.boundingRect(largest)
            cx = x + bw // 2
            cy = y + bh // 2

            # --- 디버그 표시 ---
            cv2.rectangle(debug_frame, (x, y), (x + bw, y + bh), (0, 255, 0), 3)
            cv2.circle(debug_frame, (cx, cy), 7, (0, 255, 255), -1)
            
            # ✅ [추가된 부분] 중심 좌표 텍스트 출력 (ex: "(320, 240)")
            coord_text = f"({cx}, {cy})"
            cv2.putText(debug_frame, coord_text, (cx + 10, cy), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            cv2.putText(debug_frame, f"Red Cube: {int(area)}", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.line(debug_frame, (w // 2, 0), (w // 2, h), (255, 0, 0), 2)  # 화면 중앙선

            # --- 행동 제어 ---
            # 1) 좌/우 회전: 큐브 중심(cx)과 화면 중앙(w/2) 오차로 P제어
            # --- 행동 제어 (수정 버전) ---
            
            # 1) 좌/우 회전 (ang 계산은 그대로 유지)
            err = cx - (w / 2.0)
            ang = -self.kp_ang * err  # (마이너스 뺀 버전)
            ang = float(np.clip(ang, -self.max_ang, self.max_ang))
            twist.angular.z = ang

            # 2) 전진/정지 (스마트한 버전)
            # 👇 [추가된 로직] "중심이 많이 틀어져 있으면(err > 20) 일단 멈춰서 회전만 해라"
            if abs(err) > 1000: #원래 20이었음
                lin = 0.0
            else:
                # 중심이 맞았을 때만 거리 계산해서 전진
                if area >= self.stop_area:
                    lin = 0.0
                    twist.angular.z = 0.0 # 회전도 하지 마!
                    twist.linear.x = 0.0
                    self.pub_cmd.publish(twist)
                    self.publish_debug(msg, debug_frame)
                    return # 여기서 함수 끝내버림 (더 이상 계산 안 함)
                    
                    
                elif area >= self.slow_area:
                    t = (area - self.slow_area) / float(self.stop_area - self.slow_area)
                    lin = self.max_lin * (1.0 - t)
                else:
                    lin = self.max_lin

            twist.linear.x = float(np.clip(lin, 0.0, self.max_lin))
            
            self.pub_cmd.publish(twist)
            self.publish_debug(msg, debug_frame)

        except Exception as e:
            self.get_logger().error(f'영상처리 중 에러: {e}')

    def publish_debug(self, src_msg, debug_frame):
        msg_debug = CompressedImage()
        msg_debug.header = src_msg.header
        msg_debug.format = "jpeg"
        msg_debug.data = np.array(cv2.imencode('.jpg', debug_frame)[1]).tobytes()
        self.pub_debug.publish(msg_debug)

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

