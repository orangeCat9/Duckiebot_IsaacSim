#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from duckie_led.srv import SetColor  # 방금 만든 커스텀 서비스

class LEDServiceServer(Node):
    def __init__(self):
        super().__init__('led_server_node')
        
        # Isaac Sim과 통신하는 토픽 (Twist)
        self.publisher_ = self.create_publisher(Twist, '/new_led_color', 10)
        
        # 사용자의 명령을 받는 서비스 (/duckie_led_control)
        self.srv = self.create_service(SetColor, '/duckie_led_control', self.handle_request)
        
        self.get_logger().info("Ready! Use command: ros2 service call /duckie_led_control ...")

    def handle_request(self, request, response):
        # 들어온 문자열: "red", "green" 등
        cmd = request.color.lower()
        msg = Twist()
        
        # 색상 매핑 (Isaac Sim은 RGB 0~1.0을 받음)
        if cmd == 'red':
            msg.linear.x = 1.0
        elif cmd == 'green':
            msg.linear.y = 1.0
        elif cmd == 'blue':
            msg.linear.z = 1.0
        elif cmd == 'white':
            msg.linear.x = 1.0; msg.linear.y = 1.0; msg.linear.z = 1.0
        elif cmd == 'off':
            pass # 0,0,0
        else:
            self.get_logger().warn(f"Unknown color: {cmd}")
            response.success = False
            return response

        # Isaac Sim으로 전송
        self.publisher_.publish(msg)
        self.get_logger().info(f"LED Changed to: {cmd}")
        
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LEDServiceServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
