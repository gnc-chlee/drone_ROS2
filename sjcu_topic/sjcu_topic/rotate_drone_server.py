import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from sjcu_custom.action import RotateDrone
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

class RotateDroneServer(Node):
    def __init__(self):
        super().__init__('rotate_drone_server')

        self.action_server = ActionServer(
            self,
            RotateDrone,
            'rotate_drone',
            self.execute_callback)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 100)
        self.odom_sub = self.create_subscription(
            Odometry, '/simple_drone/odom', self.odom_callback, 100)
        
        self.current_yaw = 0.0  # 현재 yaw (radians)
        self.has_odom_data = False

        self.get_logger().info('RotateDrone Action Server has been started.')

    def odom_callback(self, msg):
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, self.current_yaw = self.quaternion_to_euler(quaternion)
        self.has_odom_data = True
        self.get_logger().info(f'Current yaw: {math.degrees(self.current_yaw):.2f} degrees')

    def quaternion_to_euler(self, quaternion):
        x, y, z, w = quaternion
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        return roll, pitch, yaw

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Goal received: angle={math.degrees(goal_handle.request.angle):.2f} degrees')

        target_yaw = goal_handle.request.angle
        twist_msg = Twist()
        feedback_msg = RotateDrone.Feedback()

        # odom 데이터 대기 (최대 5초)
        start_time = time.time()
        while not self.has_odom_data and (time.time() - start_time) < 5.0:
            self.get_logger().warn('Waiting for odometry data...')
            time.sleep(0.01)

        if not self.has_odom_data:
            self.get_logger().error('No odometry data received')
            goal_handle.abort()
            result = RotateDrone.Result()
            result.delta = 0.0
            result.success = False
            result.message = 'No odometry data received'
            return result

        # 회전 방향 및 속도 설정
        angle_diff = self.normalize_angle(target_yaw - self.current_yaw)

        # 디버그 로그 추가
        self.get_logger().info(f'Calculated angle difference: {math.degrees(angle_diff):.2f} degrees')

        twist_msg.angular.z = 0.5 if angle_diff > 0 else -0.5  # 회전 속도 고정
        tolerance = 0.001  # 허용 오차 (radians)

        # 회전 루프
        while abs(angle_diff) > tolerance and rclpy.ok():
            self.cmd_vel_pub.publish(twist_msg)

            # 피드백 업데이트
            feedback_msg.current_angle = self.current_yaw  # 현재 각도 추가
            feedback_msg.remaining_angle = abs(angle_diff)  # 남은 각도
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Remaining angle: {math.degrees(angle_diff):.2f} degrees')

            # odom 데이터 갱신 대기
            rclpy.spin_once(self, timeout_sec=0.01)
            # 각도 차 갱신
            angle_diff = self.normalize_angle(target_yaw - self.current_yaw)

            time.sleep(0.01)

        # 목표에 도달하면 정지
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

        # 결과 반환
        goal_handle.succeed()
        result = RotateDrone.Result()
        result.success = True
        self.get_logger().info(f'Goal achieved with delta: {math.degrees(angle_diff):.2f} degrees')
        return result

    def normalize_angle(self, angle):
        # 각도를 -pi ~ pi로 정규화
        return math.atan2(math.sin(angle), math.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    node = RotateDroneServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()