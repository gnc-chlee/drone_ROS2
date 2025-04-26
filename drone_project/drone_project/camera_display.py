import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber_node')
        self.subscription = self.create_subscription(Image,'/simple_drone/bottom/image_raw',  # 드론 하단 카메라 토픽
            self.camera_callback,10)
        self.cmd_vel_pub = self.create_publisher(Twist,'/simple_drone/cmd_vel',10)  # 드론 제어 명령 퍼블리시
        self.bridge = CvBridge()
        self.get_logger().info('------ Camera Node Started -----')

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