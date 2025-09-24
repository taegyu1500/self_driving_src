import rclpy
from rclpy.node import Node
import enum

class State(enum.Enum):
    IDLE = 0
    GO_FORWARD = 1
    TURN_RIGHT = 2
    PARK = 3

class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')
        self.current_state = State.IDLE
        self.previous_state = None
        self.publisher = self.create_publisher(State, 'state', 10)
        self.timer = self.create_timer(1.0, self.publish_state)
        
    def publish_state(self):
        if self.current_state != self.previous_state:
            self.publisher.publish(self.current_state)
            self.get_logger().info(f'Publishing state: {self.current_state.name}')
            self.previous_state = self.current_state
    
    def set_state(self, new_state):
        if isinstance(new_state, State):
            self.current_state = new_state
        else:
            self.get_logger().error('Invalid state type')
    
def main():
    rclpy.init()
    rclpy.spin(StatePublisher())
    rclpy.shutdown()