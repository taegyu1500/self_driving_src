import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import os
'''
State 변경 이벤트를 구독하여 Twist를 발행하는 노드
'''
 
class TwistMiddleware(Node):
    # 지속 시간(초) — 필요에 맞게 조정
    TURN_RIGHT_DURATION = 3.0
    PARK_DURATION = 3.0
 
    def __init__(self):
        super().__init__('twist_middleware')
        
        self.state_subscription = self.create_subscription(
            String, 'state', self.state_callback, 10)
        # mecanum_pub을 cmd_vel 토픽으로 사용 (기존 코드의 self.mecanum_pub 대체)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.mecanum_pub = self.cmd_vel_publisher
        # 머신 타입은 환경변수 또는 기본값으로 설정
        self.machine_type = os.environ.get('MACHINE_TYPE', '')
        self.linear_speed = 1.0 # 기본 전진 속도
        self.angular_speed = 1.0 # 기본 회전 속도
 
        # 현재 유지할 Twist와 상태
        self.current_twist = Twist()
        self.current_state = "IDLE"
 
        # 주기적으로 cmd_vel을 발행하여 제어가 유지되도록 함 (10Hz)
        self.publish_timer = self.create_timer(0.1, self._publish_current_twist)
 
        # 타이머(일회성) 참조 — 기존 타이머가 있으면 취소 가능
        self._one_shot_timer = None
        # 주차 시퀀스 관련
        self._park_sequence = []
        self._park_step_timer = None
        self._park_step_index = 0
         
    def _publish_current_twist(self):
        # 현재 twist를 계속 발행
        self.cmd_vel_publisher.publish(self.current_twist)
 
    def _cancel_one_shot_timer(self):
        try:
            if self._one_shot_timer is not None:
                self._one_shot_timer.cancel()
        except Exception:
            pass
        self._one_shot_timer = None
        # 주차 시퀀스 타이머도 취소
        try:
            if self._park_step_timer is not None:
                self._park_step_timer.cancel()
        except Exception:
            pass
        self._park_step_timer = None
        self._park_sequence = []
        self._park_step_index = 0
 
    def _make_one_shot_timer(self, duration, callback):
        # create_timer는 반복형이므로, 콜백에서 자신을 취소하여 일회성으로 사용
        def _wrapper():
            try:
                callback()
            finally:
                # 자신의 타이머를 취소
                try:
                    if self._one_shot_timer is not None:
                        self._one_shot_timer.cancel()
                except Exception:
                    pass
                self._one_shot_timer = None
        # 기존 타이머 취소 후 새로 생성
        self._cancel_one_shot_timer()
        self._one_shot_timer = self.create_timer(duration, _wrapper)
 
    def _end_turn_right(self):
        self.get_logger().info('TURN_RIGHT duration ended -> switching to GO_FORWARD')
        self.current_state = "GO_FORWARD"
        self.current_twist.linear.x = self.linear_speed
        self.current_twist.angular.z = 0.0
 
    def _end_park(self):
        # (구식) 호출되지 않음 — 주차는 시퀀스 방식으로 처리
        self.get_logger().info('_end_park called -> ensure stopped')
        self.current_state = "IDLE"
        self.current_twist = Twist()
        # cancel any running park timers
        self._cancel_one_shot_timer()
 
    def _start_park_sequence(self):
        # build sequence based on referenced park_action
        seq = []
        if self.machine_type == 'MentorPi_Mecanum':
            t = Twist()
            t.linear.y = -0.2
            seq.append((t, 0.38 / 0.2))
        elif self.machine_type == 'MentorPi_Acker':
            # step 1
            t1 = Twist()
            t1.linear.x = 0.15
            t1.angular.z = t1.linear.x * math.tan(-0.5061) / 0.145
            seq.append((t1, 3.0))
            # step 2
            t2 = Twist()
            t2.linear.x = 0.15
            t2.angular.z = -t2.linear.x * math.tan(-0.5061) / 0.145
            seq.append((t2, 2.0))
            # step 3
            t3 = Twist()
            t3.linear.x = -0.15
            t3.angular.z = t3.linear.x * math.tan(-0.5061) / 0.145
            seq.append((t3, 1.5))
        else:
            # default sequence (non-blocking equivalent)
            t1 = Twist()
            t1.angular.z = -1.0
            seq.append((t1, 1.5))
            # brief stop
            seq.append((Twist(), 0.01))
            t3 = Twist()
            t3.linear.x = 0.2
            seq.append((t3, 0.65 / 0.2))
            seq.append((Twist(), 0.01))
            t5 = Twist()
            t5.angular.z = 1.0
            seq.append((t5, 1.5))
        # ensure stop at end
        seq.append((Twist(), 0.01))
        self._park_sequence = seq
        self._park_step_index = 0
        # start first step immediately
        self._run_park_step()
 
    def _run_park_step(self):
        # cancel existing park timer
        try:
            if self._park_step_timer is not None:
                self._park_step_timer.cancel()
        except Exception:
            pass
        self._park_step_timer = None
 
        if self._park_step_index >= len(self._park_sequence):
            # finished
            self.get_logger().info('Park sequence finished -> switching to IDLE')
            self.current_state = "IDLE"
            self.current_twist = Twist()
            return
 
        twist_cmd, duration = self._park_sequence[self._park_step_index]
        # publish this step immediately (also publish via periodic publisher)
        try:
            self.mecanum_pub.publish(twist_cmd)
        except Exception:
            pass
        # also update current_twist so periodic publisher keeps publishing
        self.current_twist = twist_cmd
        self._park_step_index += 1
        # schedule next step
        def _next():
            try:
                self._run_park_step()
            finally:
                # cancel this timer after it fires (one-shot)
                try:
                    if self._park_step_timer is not None:
                        self._park_step_timer.cancel()
                except Exception:
                    pass
                self._park_step_timer = None
        # create a one-shot-style timer
        self._park_step_timer = self.create_timer(duration, _next)
 
    def state_callback(self, msg):
        # msg는 std_msgs/String 이므로 msg.data로 값 비교
        state_val = msg.data
 
        # 새로운 상태가 들어오면 기존 일회성 타이머 취소(새 상태가 우선)
        self._cancel_one_shot_timer()
 
        # 정지, 전진, 우회전, 주차 상태에 따라 current_twist 설정
        if state_val == "IDLE":
            self.current_state = "IDLE"
            self.current_twist.linear.x = 0.0
            self.current_twist.angular.z = 0.0
 
        elif state_val == "GO_FORWARD":
            self.current_state = "GO_FORWARD"
            self.current_twist.linear.x = self.linear_speed
            if self.current_twist.angular.z != 0.0:
                self.current_twist.angular.z = 0.0  # 직진 시 각속도 0으로 설정
 
        elif state_val == "TURN_RIGHT":
            # 회전을 일정 시간 유지하고 이후 자동으로 전진으로 전환
            self.current_state = "TURN_RIGHT"
            self.current_twist.linear.x = 0.0
            self.current_twist.angular.z = -self.angular_speed
            # 일회성 타이머 생성
            self._make_one_shot_timer(self.TURN_RIGHT_DURATION, self._end_turn_right)
 
        elif state_val == "PARK":
            # 주차 시퀀스 시작 (참조된 park_action을 시퀀스로 비차단 실행)
            self.get_logger().info('Starting PARK sequence')
            self.current_state = "PARK"
            self._start_park_sequence()
 
        else:
            self.get_logger().error(f'Unknown state received: {state_val}')
            return
 
        self.get_logger().info(f'State set to {self.current_state}')
        # 현재 twist는 publish_timer가 계속 발행하므로 여기서는 별도 publish 불필요
 
def main(args=None):
    rclpy.init(args=args)
    twist_middleware = TwistMiddleware()
    rclpy.spin(twist_middleware)
    twist_middleware.destroy_node()
    rclpy.shutdown()