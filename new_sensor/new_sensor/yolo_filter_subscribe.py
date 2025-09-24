import cv2
import message_filters
import rclpy
from interfaces.msg import ObjectInfo, ObjectsInfo
from cv_bridge import CvBridge
from rclpy.node import Node

#  ROS Yolov5에서 작성하는 subscriber node가 /yolov5_ros2/object_detect node로 부터 inference 정보를 받아 subscribe해야함
#  정보를 받도록 구성합니다. 

class YoloFilterSubscribe(Node):
    def __init__(self):
        super().__init__('yolo_filter_subscribe')
        self.subscription_object = self.create_subscription(
            ObjectsInfo,
            '/yolov5_ros2/object_detect',
            self._callback,
            qos_profile=10)
        self.subscription_object  # prevent unused variable warning

    def _callback(self, msg):
        # 감지된 물체의 class와 score를 로그로 출력
        # string class_name, int32[]box, float32 score, int32 width, int32 height
        msg_class = msg.class_name
        msg_score = msg.score
        self.get_logger().info(f'Detected class: {msg_class}, Score: {msg_score}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloFilterSubscribe()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()