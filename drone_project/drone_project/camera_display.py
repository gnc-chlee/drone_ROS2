import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber_node')
        self.subscription = self.create_subscription(
            Image,
            '/simple_drone/bottom/image_raw',  # 드론 하단 카메라 토픽
            self.camera_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info('------ Camera Node Started -----')

    def camera_callback(self, camera_msg):
        # ROS 이미지 -> OpenCV 이미지
        cv_image = self.bridge.imgmsg_to_cv2(camera_msg, "bgr8")
        # 이미지 표시
        cv2.imshow("Drone Camera", cv_image)
        # 키 입력 대기
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