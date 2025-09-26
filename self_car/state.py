import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

# State: String으로 들어오는 IDLE, GO_FORWARD, TURN_RIGHT, PARK
_STATE_INT_MAP = {
    0: "IDLE",
    1: "GO_FORWARD",
    2: "TURN_RIGHT",
    3: "PARK",
}

class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')
        self.current_state = "IDLE"
        self.previous_state = None
        self.publisher = self.create_publisher(String, 'state', 10)
        # order 토픽은 외부에서 String 또는 Int32로 보낼 수 있으니 둘 중 하나를 사용 중이면 해당 타입으로 동작하게.
        # 일반적으로 발행자가 String이면 아래 콜백으로 들어온다.
        self.subscriber = self.create_subscription(String, 'order', self.publish_state, 10)
        
    def _normalize_state(self, msg):
        # msg는 std_msgs/String 또는 Int32 예상
        if msg is None:
            return None
        if hasattr(msg, 'data'):
            val = msg.data
            # Int 타입이면 맵으로 변환
            if isinstance(val, int):
                return _STATE_INT_MAP.get(val, None)
            # 문자열 형태이면 그대로 스트립
            try:
                return str(val).strip()
            except Exception:
                return None
        return None

    def publish_state(self, msg):
        try:
            state = self._normalize_state(msg)
            if state is None:
                self.get_logger().warning('Received unsupported order message type or empty data')
                return
            if state != self.previous_state:
                # publish String message with normalized state name
                self.publisher.publish(String(data=state))
                self.get_logger().info(f'Publishing state: {state}')
                self.previous_state = state
                self.current_state = state
        except Exception:
            self.get_logger().error('Exception in publish_state callback', once=False)

    def set_state(self, new_state):
        # new_state는 문자열을 기대. 필요 시 검증 추가
        if new_state in ["IDLE", "GO_FORWARD", "TURN_RIGHT", "PARK"]:
            self.current_state = new_state
        else:
            self.get_logger().error('Invalid state type: expected string')

def main():
    rclpy.init()
    node = StatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()