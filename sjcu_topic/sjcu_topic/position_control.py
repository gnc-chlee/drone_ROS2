import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math


class PositionControl(Node):

    def __init__(self):
        super().__init__('position_control')

        # Odometry 메시지를 구독하는 subscriber 생성
        # 토픽 이름은 '/simple_drone/odom', 큐 크기는 10으로 설정
        self.subscription = self.create_subscription(
            Odometry,               # 메시지 타입
            '/simple_drone/odom',   # 토픽 이름
            self.odom_callback,     # 콜백 함수
            10)                     # 큐 크기
        self.subscription  # prevent unused variable warning

        # 속도 명령 퍼블리셔 생성
        self.cmd_pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)

        self.get_logger().info('Position Control has been started')

        # 목표 위치 설정 (여기서는 예를 들어 (2, 2)로 설정)
        self.target_position = (2.0, 2.0)



    def odom_callback(self, msg):
        # 로봇의 현재 위치 정보 출력
        self.get_logger().info(f'Position: x = {msg.pose.pose.position.x:.2f}, y = {msg.pose.pose.position.y:.2f}, z = {msg.pose.pose.position.z:.2f}')

        # 로봇의 현재 자세 정보 출력
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

        # 쿼터니언을 오일러 각도로 변환
        roll, pitch, yaw = self.quaternion_to_euler(quaternion)

        # 오일러 각도를 도(degree) 단위로 변환 및 출력
        self.get_logger().info(f'Orientation: roll = {math.degrees(roll):.2f}, pitch = {math.degrees(pitch):.2f}, yaw = {math.degrees(yaw):.2f}')

        # position control
        # 현재 위치 정보
        current_position_x = msg.pose.pose.position.x
        current_position_y = msg.pose.pose.position.y

        # 속도 명령 퍼블리시 함수 호출
        self.publish_cmd_vel(current_position_x, current_position_y)

    def publish_cmd_vel(self, current_x, current_y):
        cmd = Twist()

        # 현재 위치와 목표 위치의 차이 계산
        delta_x = self.target_position[0] - current_x
        delta_y = self.target_position[1] - current_y
        
        p_gain = 0.5  # P gain
        cmd.linear.x = p_gain * delta_x
        cmd.linear.y = p_gain * delta_y
        cmd.linear.z = 0.0

        self.get_logger().info(f'Control: x = {cmd.linear.x:.2f}, y = {cmd.linear.y:.2f}')

        # cmd 메시지를 퍼블리시
        self.cmd_pub.publish(cmd)

    def quaternion_to_euler(self, quaternion):
        x, y, z, w = quaternion
        # Roll
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))

        # Pitch
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))

        # Yaw
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

        return roll, pitch, yaw  # Roll, Pitch, Yaw


def main(args=None):
    rclpy.init(args=args)
    position_control = PositionControl()
    rclpy.spin(position_control)
    position_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        