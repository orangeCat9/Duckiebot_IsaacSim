#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')

        self.sub = self.create_subscription(
            CompressedImage,
            '/duckie/camera/image_raw/compressed',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.pub_debug = self.create_publisher(
            CompressedImage,
            '/duckie/detection/debug/compressed',
            10
        )

        self.get_logger().info('객체 검출 노드 시작! 🟥 빨간 큐브를 찾습니다.')

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame_bgr is None:
                return

            h, w = frame_bgr.shape[:2]

            # ✅ BGR -> HSV (굳이 RGB로 바꿨다가 HSV 갈 필요 없음)
            hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

            # ✅ 빨강은 Hue가 양 끝(0 근처, 179 근처)로 갈라져서 2구간 필요
            #    + 바닥(회색) 제거하려면 S 하한을 올리는 게 핵심!
            lower1 = np.array([0,   120, 70])
            upper1 = np.array([10,  255, 255])
            lower2 = np.array([160, 120, 70])
            upper2 = np.array([179, 255, 255])  # OpenCV Hue 최대는 179

            mask1 = cv2.inRange(hsv, lower1, upper1)
            mask2 = cv2.inRange(hsv, lower2, upper2)
            mask = cv2.bitwise_or(mask1, mask2)

            # ✅ 노이즈 제거 (권장)
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

            # 윤곽선 찾기
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            debug_frame = frame_bgr.copy()

            if contours:
                # ✅ 바닥 같은 "너무 큰 덩어리" 제거용 상한(화면 60% 초과는 버림)
                max_area = 0.6 * (w * h)

                candidates = []
                for c in contours:
                    area = cv2.contourArea(c)
                    if 200 < area < max_area:
                        candidates.append(c)

                if candidates:
                    largest = max(candidates, key=cv2.contourArea)
                    area = cv2.contourArea(largest)

                    x, y, bw, bh = cv2.boundingRect(largest)
                    cx = x + bw // 2
                    cy = y + bh // 2

                    cv2.rectangle(debug_frame, (x, y), (x + bw, y + bh), (0, 255, 0), 3)
                    cv2.circle(debug_frame, (cx, cy), 7, (0, 255, 255), -1)
                    cv2.putText(debug_frame, f"Red Cube: {int(area)}", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 결과 publish
            msg_debug = CompressedImage()
            msg_debug.header = msg.header
            msg_debug.format = "jpeg"
            msg_debug.data = np.array(cv2.imencode('.jpg', debug_frame)[1]).tobytes()
            self.pub_debug.publish(msg_debug)

        except Exception as e:
            self.get_logger().error(f'영상처리 중 에러: {e}')

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

