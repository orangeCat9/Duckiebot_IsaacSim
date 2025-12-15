#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import threading

class LowLevelController(Node):
    def __init__(self):
        super().__init__('low_level_controller')
        
        # [중요] Isaac Sim 액션 그래프와 연결된 토픽 이름
        # ROS 1 코드처럼 하나의 메시지가 아니라, 왼쪽/오른쪽 각각 보냅니다.
        self.pub_left = self.create_publisher(Float64, '/duckie/wheel_left_cmd', 10)
        self.pub_right = self.create_publisher(Float64, '/duckie/wheel_right_cmd', 10)
        
        self.get_logger().info("Low-Level Controller Initialized (Direct Wheel Control)")
        self.print_manual()

    def send_motor_command(self, left_vel, right_vel):
        # 왼쪽 바퀴 메시지 생성 및 발행
        msg_l = Float64()
        msg_l.data = float(left_vel)
        self.pub_left.publish(msg_l)

        # 오른쪽 바퀴 메시지 생성 및 발행
        msg_r = Float64()
        msg_r.data = float(right_vel)
        self.pub_right.publish(msg_r)

    def stop_motors(self):
        self.send_motor_command(0.0, 0.0)

    # ==========================================
    # [저수준 제어 로직] 
    # 사용자의 의도를 바퀴 회전수(RPM/Rad)로 직접 변환
    # ==========================================
    def move_forward(self, speed=10.0):
        # 직진: 양쪽 다 정회전 (+)
        self.send_motor_command(speed, speed)

    def move_backward(self, speed=10.0):
        # 후진: 양쪽 다 역회전 (-)
        self.send_motor_command(-speed, -speed)

    def turn_left(self, speed=10.0):
        # 좌회전(제자리): 왼쪽(-), 오른쪽(+)
        self.send_motor_command(-speed, speed)

    def turn_right(self, speed=10.0):
        # 우회전(제자리): 왼쪽(+), 오른쪽(-)
        self.send_motor_command(speed, -speed)

    def print_manual(self):
        print("\n" + "="*30)
        print("   Low-Level Wheel Control")
        print("="*30)
        print("   W: Forward")
        print("   S: Backward")
        print("   A: Spin Left")
        print("   D: Spin Right")
        print("   X: Stop")
        print("   Q: Quit")
        print("="*30)

    def run_interactive_mode(self):
        # ROS 2 노드는 백그라운드에서 계속 돌게 둡니다 (Publish를 위해)
        spinner_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        spinner_thread.start()

        try:
            while rclpy.ok():
                # 파이썬 input()은 블로킹 함수라 입력을 기다립니다.
                char = input("Command: ").strip().upper()

                if char == 'Q':
                    self.stop_motors()
                    break
                elif char == 'W':
                    self.move_forward()
                    self.get_logger().info("Low-level: Left=10, Right=10")
                elif char == 'S':
                    self.move_backward()
                    self.get_logger().info("Low-level: Left=-10, Right=-10")
                elif char == 'A':
                    self.turn_left()
                    self.get_logger().info("Low-level: Left=-10, Right=10")
                elif char == 'D':
                    self.turn_right()
                    self.get_logger().info("Low-level: Left=10, Right=-10")
                elif char == 'X':
                    self.stop_motors()
                    self.get_logger().info("Stop")
                else:
                    print("Invalid Key!")

        except KeyboardInterrupt:
            pass
        finally:
            self.stop_motors()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    controller = LowLevelController()
    controller.run_interactive_mode()

if __name__ == '__main__':
    main()
