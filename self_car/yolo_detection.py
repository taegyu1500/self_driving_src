# TODO: yolo 탐지 결과를 통해 현재 탐지된 객체가 무엇인지 판단하여 self_drive에 다음 명령 전달
import rclpy
from rclpy.node import Node
from interfaces.msg import ObjectsInfo

class YoloDetection(Node):
    def __init__(self):
        super().__init__('yolo_detection')
        self.detection_subscription = self.create_subscription(
            ObjectsInfo, '/yolov5_ros2/object_detect', self.detection_callback, 10
        )
    
    
    # TODO: yolo 탐지 결과를 판단하고 명령 전달, 현재는 모의로 직진 명령 전달
    def detection_callback(self, msg):
        pass
    
def main():
    rclpy.init()
    yolo_detection_node = YoloDetection()
    rclpy.spin(yolo_detection_node)
    yolo_detection_node.destroy_node()
    rclpy.shutdown()
    