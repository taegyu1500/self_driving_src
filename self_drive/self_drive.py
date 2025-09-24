import rclpy
from rclpy.node import Node

#TODO: 주행 관련해서 이미지 처리해서 현 차량의 상태 파악하고 토픽 줘서 주행 제어
class SelfDrive(Node):
    def __init__(self):
        super().__init__('self_drive')
        self.get_logger().info('Self Drive Node has been started.')
        
        
        
def main():
    rclpy.init()
    self_drive_node = SelfDrive()
    rclpy.spin(self_drive_node)
    self_drive_node.destroy_node()
    rclpy.shutdown()