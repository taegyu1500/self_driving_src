import rclpy
from rclpy.node import Node
import enum
from interfaces.msg import ObjectsInfo
class State(enum.Enum):
    IDLE = 0
    GO_FORWARD = 1
    TURN_RIGHT = 2
    PARK = 3

#TODO: 주행 관련해서 이미지 처리해서 현 차량의 상태 파악하고 토픽 줘서 주행 제어
class SelfDrive(Node):
    def __init__(self):
        super().__init__('self_drive')
        self.get_logger().info('Self Drive Node has been started.')
        self.order_publisher = self.create_publisher(State, 'order', 10)
        self.detection_subscription = self.create_subscription(
            ObjectsInfo, '/yolov5_ros2/object_detect', self.detection_callback, 10
        )
        self.timer = self.create_timer(1.0, self.publish_order)
        self.current_order = State.IDLE
        #TODO: 우선순위 높은 명령을 실행해야함(예시: 직진 가능 but 신호등이 빨강이면 정지)
        self.order_queue = []
        
    #TODO: yolo 탐지 결과를 통해 다음 명령 판단 후 publish    
    def detection_callback(self, msg):
        match msg.names:
            case 'right':
                self.order_queue.append((State.TURN_RIGHT, "sign"))
            case 'straight':
                self.order_queue.append((State.GO_FORWARD, "sign"))
            case 'parking':
                self.order_queue.append((State.PARK, "sign"))
        pass
    
    # TODO: 횡단보도 관련 코드
    def crossing_callback(self, msg):
        # 횡단보도가 3개 이상 감지되면 정지, 그 외에는 현재 명령 유지
        if msg.data >= 3:
            self.order_queue.append((State.IDLE, "crossing"))
        else:
            self.order_queue.append((self.current_order, "crossing"))
        pass
    
    # TODO: 신호등 관련 코드
    def traffic_light_callback(self, msg):
        match msg.names:
            case 'red':
                self.order_queue.append((State.IDLE, "traffic_light"))
            case 'green':
                self.order_queue.append((self.current_order, "traffic_light"))

    # TODO: publish order
    def publish_order(self):
        if self.order_queue:
            priority_order = {"traffic_light": 1, "crossing": 2, "sign": 3}
            self.order_queue.sort(key=lambda x: priority_order[x[1]])
            self.current_order = self.order_queue[0][0]
            self.order_queue.clear()
        else:
            self.current_order = State.GO_FORWARD  # 기본 명령은 전진
        self.order_publisher.publish(self.current_order)
        
    
def main():
    rclpy.init()
    self_drive_node = SelfDrive()
    rclpy.spin(self_drive_node)
    self_drive_node.destroy_node()
    rclpy.shutdown()