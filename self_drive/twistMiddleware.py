import rclpy
from rclpy.node import Node
import enum
from geometry_msgs.msg import Twist

'''
State가 변경되는 경우 발생하는 이벤트를 구독 후 Twist 메시지를 발행해 주행을 통제하는 노드
'''

class State(enum.Enum):
    IDLE = 0
    GO_FORWARD = 1
    TURN_RIGHT = 2
    PARK = 3

class TwistMiddleware(Node):
    def __init__(self):
        super().__init__('twist_middleware')
        
        self.state_subscription = self.create_subscription(
            State, 'state', self.state_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
    def state_callback(self, msg):
        twist = Twist()
        # 정지, 전진, 우회전, 주차 상태에 따라 Twist 메시지 설정
        # TODO: 별도 함수나 상수로 분리
        if msg == State.IDLE:
            
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif msg == State.GO_FORWARD:
            twist.linear.x = 0.5  # 전진 속도
            twist.angular.z = 0.0
        elif msg == State.TURN_RIGHT:
            twist.linear.x = 0.0
            twist.angular.z = -0.5  # 우회전 속도
        elif msg == State.PARK:
            twist.linear.x = 0.0
            twist.angular.z = 0.0  # 주차 시 정지
        else:
            self.get_logger().error('Unknown state received')
            return
        self.cmd_vel_publisher.publish(twist)
    

def main(args=None):
    rclpy.init(args=args)
    twist_middleware = TwistMiddleware()
    rclpy.spin(twist_middleware)
    twist_middleware.destroy_node()
    rclpy.shutdown()