import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from sjcu_custom.action import MoveDrone
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class MoveDroneServer(Node):
    def __init__(self):
        super().__init__('move_drone_server')
        self.action_server = ActionServer(
            self,
            MoveDrone,
            'move_drone',
            self.execute_callback)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/simple_drone/odom', self.odom_callback, 10)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.p_gain = 0.3  # P 제어 게인
        self.tolerance = 0.2  # 허용 오차 (m)
        self.has_odom_data = False
        
        self.get_logger().info('MoveDrone Action Server has been started.')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        self.has_odom_data = True
        self.get_logger().info(f'Current position: x={self.current_x:.2f}, y={self.current_y:.2f}, z={self.current_z:.2f}')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Goal received: x={goal_handle.request.pos_x:.2f}, y={goal_handle.request.pos_y:.2f}, z={goal_handle.request.pos_z:.2f}')
        
        self.target_x = goal_handle.request.pos_x
        self.target_y = goal_handle.request.pos_y
        self.target_z = goal_handle.request.pos_z
        
        twist_msg = Twist()
        feedback_msg = MoveDrone.Feedback()
        
        # odom 데이터 대기 (최대 5초)
        start_time = time.time()
        while not self.has_odom_data and (time.time() - start_time) < 5.0:
            self.get_logger().warn('Waiting for odometry data...')
            time.sleep(0.1)

        if not self.has_odom_data:
            self.get_logger().error('No odometry data received')
            goal_handle.abort()
            result = MoveDrone.Result()
            result.success = False
            result.message = 'No odometry data received'
            return result

        # 거리 계산 및 제어 루프
        distance = self.calculate_distance()
        self.get_logger().info(f'Initial distance to goal: {distance:.2f} meters')
        
        while distance > self.tolerance and rclpy.ok():
            # P 제어로 속도 계산
            delta_x = self.target_x - self.current_x
            delta_y = self.target_y - self.current_y
            delta_z = self.target_z - self.current_z
            
            twist_msg.linear.x = max(min(self.p_gain * delta_x, 0.3), -0.3)
            twist_msg.linear.y = max(min(self.p_gain * delta_y, 0.3), -0.3)
            twist_msg.linear.z = max(min(self.p_gain * delta_z, 0.3), -0.3)
            
            # 속도 감쇠 (목표 근처에서 속도 줄임)
            if distance < 0.5:
                scale = distance / 0.5
                twist_msg.linear.x *= scale
                twist_msg.linear.y *= scale
                twist_msg.linear.z *= scale
            
            self.cmd_vel_pub.publish(twist_msg)
            self.get_logger().info(f'Control: x={twist_msg.linear.x:.2f}, y={twist_msg.linear.y:.2f}, z={twist_msg.linear.z:.2f}')
            
            # 피드백 전송
            feedback_msg.current_x = self.current_x
            feedback_msg.current_y = self.current_y
            feedback_msg.current_z = self.current_z
            goal_handle.publish_feedback(feedback_msg)
            
            # 실시간 위치 갱신
            rclpy.spin_once(self, timeout_sec=0.01)
            distance = self.calculate_distance()
            self.get_logger().info(f'Distance to goal: {distance:.2f} meters')
            
            time.sleep(0.01)

        # 목표 도달 후 정지
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        for _ in range(5):  # 여러 번 퍼블리시로 정지 보장
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.05)
        self.get_logger().info('Drone stopped: velocity set to zero')

        # 결과 반환
        goal_handle.succeed()
        result = MoveDrone.Result()
        result.success = True
        result.message = f'Reached target: x={self.current_x:.2f}, y={self.current_y:.2f}, z={self.current_z:.2f}'
        self.get_logger().info('Goal succeeded!')
        return result

    def calculate_distance(self):
        delta_x = self.target_x - self.current_x
        delta_y = self.target_y - self.current_y
        delta_z = self.target_z - self.current_z
        return math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

def main(args=None):
    rclpy.init(args=args)
    node = MoveDroneServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()