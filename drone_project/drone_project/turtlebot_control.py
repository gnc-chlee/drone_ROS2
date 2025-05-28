# turtlebot_control.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        
        # Parameter 선언 (RViz에서 조정 가능)
        self.declare_parameter('target_x', 0.0)  # 목표 x 위치
        self.declare_parameter('target_y', 0.0)  # 목표 y 위치
        self.declare_parameter('k_p_linear', 0.3)  # 직진 속도 gain
        self.declare_parameter('k_p_angular', 0.5)  # 방향 조정 gain

        # 구독: 터틀봇의 현재 위치와 방향 (/odom)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # 퍼블리셔: 터틀봇 제어 명령 (/cmd_vel)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 현재 위치와 방향 저장
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # 타이머: 주기적으로 제어 로직 실행 (0.1초마다)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('------ TurtleBot Controller Node Started -----')

    def odom_callback(self, odom_msg):
        # 터틀봇의 현재 위치와 방향 업데이트
        self.current_x = odom_msg.pose.pose.position.x
        self.current_y = odom_msg.pose.pose.position.y
        
        # 쿼터니언에서 yaw 각도 계산
        orientation_q = odom_msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        # Parameter 값 가져오기
        target_x = self.get_parameter('target_x').get_parameter_value().double_value
        target_y = self.get_parameter('target_y').get_parameter_value().double_value
        k_p_linear = self.get_parameter('k_p_linear').get_parameter_value().double_value
        k_p_angular = self.get_parameter('k_p_angular').get_parameter_value().double_value
        
        # 현재 위치와 목표 위치 간 오차 계산
        error_x = target_x - self.current_x
        error_y = target_y - self.current_y
        
        # 목표까지의 거리 계산
        distance = math.sqrt(error_x**2 + error_y**2)
        
        # 오차가 0.05보다 작으면 목표에 도달한 것으로 간주
        if distance < 0.05:
            self.get_logger().info(f"Reached target: ({target_x:.2f}, {target_y:.2f})")
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            return
        
        # 목표 방향 계산
        target_angle = math.atan2(error_y, error_x)
        
        # 현재 방향과 목표 방향 간 각도 오차 계산
        angle_error = target_angle - self.current_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        # P 제어로 속도 계산
        cmd_vel = Twist()
        cmd_vel.linear.x = k_p_linear * distance
        cmd_vel.angular.z = k_p_angular * angle_error
        
        # 속도 명령 퍼블리시
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info(f"Moving to target: ({target_x:.2f}, {target_y:.2f}), Distance: {distance:.2f}, Angle error: {angle_error:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()