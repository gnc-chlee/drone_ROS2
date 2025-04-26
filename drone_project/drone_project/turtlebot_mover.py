import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class TurtleBotMover(Node):
    def __init__(self):
        # 노드 초기화
        super().__init__('turtlebot_mover_node')
        
        # TurtleBot3의 속도 명령을 보내기 위한 퍼블리셔
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # TurtleBot3의 위치와 방향을 받기 위한 서브스크라이버
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # TurtleBot3의 현재 위치와 방향을 저장할 변수
        self.current_x = 0.0  # 현재 x 좌표
        self.current_y = 0.0  # 현재 y 좌표
        self.current_yaw = 0.0  # 현재 방향 (yaw, 라디안)
        
        # 초기 위치 설정 여부
        self.initial_position_set = False

        # 목표 위치 리스트 (좌표)
        self.target_positions = [
            (0.0, 0.0),    # 첫 번째 목표: 원점
            (-2.0, -2.0),  # 두 번째 목표
            (-2.0, 2.0),   # 세 번째 목표
            (2.0, 2.0),    # 네 번째 목표
            (2.0, -2.0),   # 다섯 번째 목표
            (0.0, 0.0)     # 마지막 목표: 원점으로 복귀
        ]
        
        # 현재 목표 위치의 인덱스 (0부터 시작)
        self.current_target_idx = 0
        
        # 타이머 설정 (0.1초마다 제어 실행)
        self.timer = self.create_timer(0.1, self.move_turtlebot)
        
        # 노드 시작 메시지
        self.get_logger().info('TurtleBot Mover Node Started')

    def odom_callback(self, msg):
        # TurtleBot3의 현재 위치를 업데이트
        self.current_x = msg.pose.pose.position.x  # x 좌표
        self.current_y = msg.pose.pose.position.y  # y 좌표
        
        # 쿼터니언에서 yaw 각도 계산 (라디안 단위)
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # 초기 위치 로그 (한 번만 출력)
        if not self.initial_position_set:
            self.get_logger().info(f"Initial position: ({self.current_x:.2f}, {self.current_y:.2f})")
            self.initial_position_set = True

    def move_turtlebot(self):
        # 속도 명령 메시지 생성
        cmd_vel = Twist()
        
        # 목표 위치가 남아 있는지 확인
        if self.current_target_idx < len(self.target_positions):
            # 현재 목표 위치 가져오기
            target_x, target_y = self.target_positions[self.current_target_idx]
            
            # 목표 위치 로그 출력
            self.get_logger().info(f"Moving to target {self.current_target_idx}: ({target_x:.2f}, {target_y:.2f})")
            
            # 현재 위치와 목표 위치 간 오차 계산
            error_x = target_x - self.current_x  # x 방향 오차
            error_y = target_y - self.current_y  # y 방향 오차
            
            # 목표까지의 거리 계산
            distance = math.sqrt(error_x**2 + error_y**2)

            # 오차가 0.05보다 작으면 목표에 도달한 것으로 간주
            if distance < 0.1:
                self.get_logger().info(f"Reached target {self.current_target_idx}: ({target_x:.2f}, {target_y:.2f})")
                self.current_target_idx += 1  # 다음 목표로 이동
                return
            
            # 목표 방향 계산 (arctan2로 목표 방향 계산)
            target_angle = math.atan2(error_y, error_x)
            
            # 현재 방향과 목표 방향 간 각도 오차 계산
            angle_error = target_angle - self.current_yaw
            # 각도 정규화 (-π ~ π)
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            # 거리와 각도 오차 로그 출력
            self.get_logger().info(f"Distance to target: {distance:.2f}, Angle error: {angle_error:.2f} radians")

            # 비례 제어 (P 제어)로 속도 계산
            k_p_linear = 0.5  # 선속도 비례 상수
            k_p_angular = 0.5  # 각속도 비례 상수

            # 직진 속도 (거리 비례)
            cmd_vel.linear.x = k_p_linear * distance
            if cmd_vel.linear.x > 0.4:
                cmd_vel.linear.x = 0.4
            # 방향 조정 (각도 오차 비례)
            cmd_vel.angular.z = k_p_angular * angle_error
            if cmd_vel.angular.z > 1.0:
                cmd_vel.angular.z = 1.0
            # 속도 명령 로그 출력
            self.get_logger().info(f"Sending speed: linear.x = {cmd_vel.linear.x:.2f}, angular.z = {cmd_vel.angular.z:.2f}")
        
        else:
            # 모든 목표에 도달하면 정지
            self.get_logger().info("All targets reached. Stopping TurtleBot3.")
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        
        # TurtleBot3 속도 명령 퍼블리시
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()