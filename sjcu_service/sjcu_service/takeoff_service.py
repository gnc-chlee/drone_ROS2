import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool

class TakeoffService(Node):
    def __init__(self):
        super().__init__('takeoff_service')
        # /simple_drone/takeoff 토픽 퍼블리셔
        self.takeoff_pub = self.create_publisher(Empty, '/simple_drone/takeoff', 10)
        # 현재 고도
        self.current_altitude = 0.0
        # /simple_drone/odom 구독
        self.odom_sub = self.create_subscription(
            Odometry, '/simple_drone/odom', self.odom_callback, 10)
        # /takeoff 서비스 서버
        self.srv = self.create_service(SetBool, 'takeoff', self.takeoff_callback)
        self.get_logger().info('Takeoff Service has been started.')
    
    def odom_callback(self, msg):
        # 현재 고도 업데이트
        self.current_altitude = msg.pose.pose.position.z
        self.get_logger().info(f'Current altitude: {self.current_altitude}m')

    def takeoff_callback(self, request, response):
        self.get_logger().info('Takeoff callback called!')
        self.get_logger().info(f'Request data: {request.data}')
        self.get_logger().info(f'Current altitude before check: {self.current_altitude}m')
        if self.current_altitude < 0.1:
            self.takeoff_pub.publish(Empty())
            response.success = True
            response.message = 'Takeoff triggered successfully'
            self.get_logger().info('Takeoff triggered and published')
        else:
            response.success = False
            response.message = f'Drone already flying at {self.current_altitude}m'
            self.get_logger().info(f'Drone already flying at {self.current_altitude}m')
        self.get_logger().info('Returning response...')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TakeoffService()
    rclpy.spin(node)  # 노드 실행
    node.destroy_node()  # 종료 시 정리
    rclpy.shutdown()  # ROS2 종료

if __name__ == '__main__':
    main()