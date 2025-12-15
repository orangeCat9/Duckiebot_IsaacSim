#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
import sys, select, termios, tty

# 터미널 설정을 저장 (나중에 복구하기 위함)
settings = termios.tcgetattr(sys.stdin)

def getKey():
    tty.setraw(sys.stdin.fileno())
    # 0.1초 동안 키 입력을 기다림 (Non-blocking)
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class RawWheelOperator(Node):
    def __init__(self):
        super().__init__('raw_wheel_operator')
        
        # [Twist 꼼수 유지] 왼쪽/오른쪽 바퀴 토픽 발행
        self.pub_left = self.create_publisher(Twist, '/duckie/wheel_left_cmd', 10)
        self.pub_right = self.create_publisher(Twist, '/duckie/wheel_right_cmd', 10)
        
        # ⭐ [핵심] 현재 속도 상태를 저장할 변수 추가
        self.current_left_speed = 0.0
        self.current_right_speed = 0.0
        
        print("===== 저수준 모터 제어 (Twist Hack) =====")
        print(" w: 전진 | x: 후진 | a: 좌회전 | d: 우회전")
        print(" s: 정지 | q: 종료")
        print("=========================================")

    # 속도 설정만 하고, 발행은 하지 않음
    def set_speed(self, l_speed, r_speed):
        self.current_left_speed = float(l_speed)
        self.current_right_speed = float(r_speed)

    # 저장된 속도를 실제로 발행 (계속 호출될 예정)
    def publish_cmd(self):
        msg_l = Twist()
        msg_l.linear.x = self.current_left_speed
        
        msg_r = Twist()
        msg_r.linear.x = self.current_right_speed
        
        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)

def main(args=None):
    rclpy.init(args=args)
    node = RawWheelOperator()
    
    try:
        while True:
            key = getKey()
            
            # 1. 키 입력에 따라 '목표 속도 변수'만 변경
            if key == 'w':
                node.set_speed(5.0, 5.0)  # 전진
                print("Forward")
            elif key == 'x':
                node.set_speed(-5.0, -5.0) # 후진
                print("Backward")
            elif key == 'a':
                node.set_speed(-5.0, 5.0)  # 좌회전 (제자리)
                print("Left")
            elif key == 'd':
                node.set_speed(5.0, -5.0)  # 우회전 (제자리)
                print("Right")
            elif key == 's':
                node.set_speed(0.0, 0.0)     # 정지
                print("Stop")
            elif key == 'q':
                break

            # 2. ⭐ [핵심] 키를 안 눌러도 명령은 계속 전송함!
            node.publish_cmd()

            # ROS2 콜백 처리 (통신 유지용)
            rclpy.spin_once(node, timeout_sec=0.01)

    except Exception as e:
        print(e)

    finally:
        # 종료 시 0으로 정지 신호 보냄
        node.set_speed(0.0, 0.0)
        node.publish_cmd()
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
