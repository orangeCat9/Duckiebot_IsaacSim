#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        
        # [설정] 로봇 제원
        self.wheel_radius = 0.03   
        self.wheel_base = 1.16894 * 2.0 

        # 1. High-level 입력
        self.sub_cmd = self.create_subscription(Twist, '/duckie/cmd_vel', self.cmd_callback, 10)

        # 2. Low-level 출력
        self.pub_left = self.create_publisher(Float64, '/duckie/wheel_left_cmd', 10)
        self.pub_right = self.create_publisher(Float64, '/duckie/wheel_right_cmd', 10)
        
        self.get_logger().info("Duckiebot Motor Controller Started! 🔧")

    def cmd_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        
        # 왼쪽 바퀴 속도 (부호를 -에서 +로 변경)
        vel_left = (linear_vel + (angular_vel * self.wheel_base / 2.0)) / self.wheel_radius
        
        # 오른쪽 바퀴 속도 (부호를 +에서 -로 변경)
        vel_right = (linear_vel - (angular_vel * self.wheel_base / 2.0)) / self.wheel_radius

        # 메시지 발행
        msg_left = Float64(); msg_left.data = vel_left
        msg_right = Float64(); msg_right.data = vel_right

        self.pub_left.publish(msg_left)
        self.pub_right.publish(msg_right)

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
