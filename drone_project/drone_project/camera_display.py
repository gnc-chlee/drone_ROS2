import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber_node')
        self.declare_parameter('z_setpoint', 2.0)    # 목표 높이
        self.declare_parameter('z_p_gain', 0.25) # 비례 제어 게인

        self.camera_sub = self.create_subscription(Image,'/simple_drone/bottom/image_raw',  # 드론 하단 카메라 토픽
            self.camera_callback,10)
        # 드론 위치 정보 서브스크라이버
        self.odom_sub = self.create_subscription(Odometry,'/simple_drone/odom',  # 드론 위치 정보 토픽
            self.odom_callback,10)
        # 드론 제어 명령 퍼블리셔
        self.cmd_vel_pub = self.create_publisher(Twist,'/simple_drone/cmd_vel',10)  # 드론 제어 명령 퍼블리시
        self.bridge = CvBridge()
        self.get_logger().info('------ Camera Node Started -----')
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.p_gain_alt = 0.0  # P 제어 게인
        self.has_odom_data = False

    def odom_callback(self, msg):
        # 드론의 현재 위치를 업데이트
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z


    def camera_callback(self, camera_msg):
        cv_image = self.bridge.imgmsg_to_cv2(camera_msg, "bgr8")
        # 이미지 해상도 확인
        h, w = cv_image.shape[:2]
        self.get_logger().info(f"Image resolution: {w}x{h}")
        # 1. Binary Image 생성
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, binary_image = cv2.threshold(gray_image, 200, 255, cv2.THRESH_BINARY)

        # 2. Contour 찾기 및 그리기
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 중심점 변수
        image_center_x = 320
        image_center_y = 180
        # 드론 카메라 중심점
        center = {'cx': image_center_x, 'cy': image_center_y}
        # 드론 제어 명령
        cmd_vel = Twist()

        if contours:
            contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(contour)
            if M["m00"] != 0:
                center['cx'] = int(M["m10"] / M["m00"])
                center['cy'] = int(M["m01"] / M["m00"])

                # Contour 및 중심점 그리기
                cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 2)
                cv2.circle(cv_image, (center['cx'], center['cy']), 5, (0, 0, 255), -1)


                # 오차 계산
                error_x = center['cx'] - image_center_x
                error_y = center['cy'] - image_center_y
                # 중심점 출력
                self.get_logger().info(f"Center: ({center['cx']}, {center['cy']}, Error: ({error_x}, {error_y})")


                # P 제어 및 좌표계 변환 (부호 수정)
                k_p = 0.003
                cmd_vel.linear.x = - k_p * error_y  # 카메라 y → 드론 x (부호 변경)
                cmd_vel.linear.y = - k_p * error_x  # 카메라 x → 드론 y (부호 변경)
                cmd_vel.linear.z = 0.0
                cmd_vel.angular.z = 0.0

        # P 고도 제어
        # self.target_z = 2.0  # 목표 높이
        # self.p_gain_alt = 0.25  # 비례 제어 게인

        # Parameter 값 가져오기
        self.target_z = self.get_parameter('z_setpoint').get_parameter_value().double_value
        self.p_gain_alt = self.get_parameter('z_p_gain').get_parameter_value().double_value
        # 드론의 현재 높이와 목표 높이 간 오차 계산
        error_z = self.target_z - self.current_z
        cmd_vel.linear.z = self.p_gain_alt * error_z

        self.get_logger().info(f"altitude: {self.current_z:.2f}, error: {error_z:.2f}")
        # 드론 제어 명령 퍼블리시
        self.cmd_vel_pub.publish(cmd_vel)
        cv2.imshow("Drone Camera", cv_image)
        cv2.imshow("Binary Image", binary_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()