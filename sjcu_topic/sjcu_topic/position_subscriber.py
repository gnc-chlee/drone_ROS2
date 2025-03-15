import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math


class PositionListener(Node):

    def __init__(self):
        super().__init__('position_subscriber')

        # Odometry 메시지를 구독하는 subscriber 생성
        # 토픽 이름은 '/simple_drone/odom', 큐 크기는 10으로 설정
        self.subscription = self.create_subscription(
            Odometry,               # 메시지 타입
            '/simple_drone/odom',   # 토픽 이름
            self.odom_callback,     # 콜백 함수
            10)                     # 큐 크기
        self.subscription  # prevent unused variable warning

        self.get_logger().info('OdomListener has been started')

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
    Position_listener = PositionListener()
    rclpy.spin(Position_listener)
    Position_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        