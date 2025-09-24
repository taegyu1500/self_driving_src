import cv2
import message_filters
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np


class DepthRgbFilter(Node):
    def __init__(self):
        super().__init__('depth_rgb_filter')
        self.bridge = CvBridge()

        # Subscribe to RGB and depth topics with synchronization
        rgb_sub = message_filters.Subscriber(self, Image, '/ascamera/camera_publisher/rgb0/image', 
                                   qos_profile=10)  # QoS 설정 확인
        depth_sub = message_filters.Subscriber(self, Image, '/ascamera/camera_publisher/depth0/image_raw')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.frame_center_x = 320
        
        # 상태 추적을 위한 변수들
        self.previous_action = None
        self.no_ball_logged = False
        # 이전 공 위치 저장 (안정화용)
        self.prev_position = None

        # 디버깅 모드 플래그 추가
        self.debug_mode = False  # 필요할 때만 True로 설정
        
        # 메모리 관리를 위한 변수들
        self.display = None
        
        # 토픽 모니터링 변수
        self.last_callback_time = self.get_clock().now()
        self.callback_count = 0
        
        # 발행 억제를 위한 상태 변수
        self.last_published_action = None
        self.last_linear = None
        self.publish_tolerance = 1e-3  # 선형 속도 비교 허용 오차

        # 히스테리시스 임계값 (진동 방지)
        self.back_thresh = 330    # 뒤로 갈 기준(안쪽)
        self.forward_thresh = 370 # 앞으로 갈 기준(바깥쪽)
        self.stop_dist = 2000     # 정지 기준 (기존)
        self.motion_state = None  # 현재 모션 상태('forward'/'backwards'/'stop')

        ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.05)
        ts.registerCallback(self.callback)  # 콜백에서 직접 처리

        # 1초마다 시스템 상태 체크 (메모리 누수 방지)
        self.create_timer(5.0, self.check_system_health)

    def check_system_health(self):
        """시스템 상태를 주기적으로 확인하는 함수"""
        now = self.get_clock().now()
        duration = now - self.last_callback_time
        
        # 5초 이상 콜백이 없으면 경고
        if duration.nanoseconds / 1e9 > 5.0 and self.callback_count > 0:
            self.get_logger().warn(f'No callbacks for {duration.nanoseconds/1e9:.1f} seconds')
        
        # 메모리 정리
        self.display = None
        cv2.destroyAllWindows()  # 사용하지 않는 창 정리
        
        # 콜백 카운터 리셋
        self.callback_count = 0
        self.last_callback_time = now

    def callback(self, rgb_msg, depth_msg):
        """메모리 관리와 성능 최적화"""
        self.callback_count += 1
        self.last_callback_time = self.get_clock().now()
        
        # 이미지 처리 - RGB는 필요하면 다운샘플링하여 처리 부하 줄이기
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
        
        # depth를 RGB 해상도로 리사이즈하여 매핑 정확도 향상
        if depth.shape != rgb.shape[:2]:
            depth = cv2.resize(depth, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
        
        position, depth_value = self.find_red_ball_depth(rgb, depth)
        
        # 통합된 제어 로직
        twist = Twist()
        current_action = None
        
        if position is None or depth_value is None:
            # 공을 찾지 못한 경우
            self.stop_movement()
            current_action = "no_ball"
            if not self.no_ball_logged:
                self.get_logger().info('No ball detected - stopping')
                self.no_ball_logged = True
            return
        
        # 공을 찾았으므로 no_ball 로그 플래그 리셋
        self.no_ball_logged = False
        
        # 각도 조정
        error_x = position[0] - self.frame_center_x
        if abs(error_x) > 20:
            twist.angular.z = -0.001 * error_x  # 더 부드러운 회전
        
        # 히스테리시스 기반 모션 결정 (진동 방지)
        dv = depth_value
        prev = self.motion_state
        if prev == "backwards":
            if dv > self.forward_thresh and dv < self.stop_dist:
                current_action = "forward"
            elif dv >= self.stop_dist:
                current_action = "stop"
            else:
                current_action = "backwards"
        elif prev == "forward":
            if dv < self.back_thresh:
                current_action = "backwards"
            elif dv >= self.stop_dist:
                current_action = "stop"
            else:
                current_action = "forward"
        else:
            # 초기 또는 unknown 상태: 기본 임계값 적용
            if dv < self.back_thresh:
                current_action = "backwards"
            elif self.back_thresh <= dv < self.stop_dist:
                current_action = "forward"
            else:
                current_action = "stop"
        
        # 모션에 따라 선형 속도 설정
        if current_action == "backwards":
            twist.linear.x = -0.2
        elif current_action == "forward":
            twist.linear.x = 0.2
        else:
            twist.linear.x = 0.0
        
        # 상태가 변경된 경우에만 로그 출력
        if current_action != self.previous_action:
            if current_action == "backwards":
                self.get_logger().info(f'Move backwards (depth: {depth_value})')
            elif current_action == "forward":
                self.get_logger().info(f'Move toward (depth: {depth_value})')
            elif current_action == "stop":
                self.get_logger().info(f'Stop - target reached (depth: {depth_value})')
            self.previous_action = current_action
        
        # 모션 상태 업데이트
        self.motion_state = current_action
        
        # 변경된 경우에만 발행 (angular.z 변화는 비교에서 제외)
        self.publish_if_changed(twist, current_action)
        
        # 디버깅 창 (속도 최적화: 창을 별도 스레드나 간단히)
        if self.debug_mode and position is not None:
            # 매번 새 이미지를 만들지 않고 필요할 때만 할당
            if self.display is None or self.display.shape != rgb.shape:
                self.display = np.zeros_like(rgb)
            
            # 원본 복사 대신 필요한 부분만 그리기
            self.display[:] = rgb  # 기존 배열 재사용
            cx, cy = position
            cv2.circle(self.display, (cx, cy), 6, (0, 255, 0), -1)
            cv2.rectangle(self.display, (cx - 10, cy - 10), (cx + 10, cy + 10), (0, 255, 0), 2)
            cv2.line(self.display, (self.frame_center_x, 0), (self.frame_center_x, self.display.shape[0]), (255, 0, 0), 1)
            cv2.putText(self.display, f'error_x: {error_x}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(self.display, f'depth: {depth_value}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            try:
                cv2.imshow('debug_ball', self.display)
                cv2.waitKey(1)
            except Exception as e:
                # 에러 한 번만 로깅
                if not hasattr(self, 'gui_error_logged'):
                    self.get_logger().error(f'GUI error: {e}')
                    self.gui_error_logged = True

    def find_red_ball(self, rgb_image):
        # BGR → HSV 변환 후 빨간색 마스크 생성 (HoughCircles는 그레이스케일 필요)
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        lower_red1 = (0, 100, 100)
        upper_red1 = (10, 255, 255)
        lower_red2 = (160, 100, 100)
        upper_red2 = (180, 255, 255)
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 모폴로지 연산 추가: 노이즈 제거 (침식 후 팽창)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # 노이즈 제거
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 구멍 메우기

        # 마스크 영역만 그레이스케일로 변환
        masked = cv2.bitwise_and(rgb_image, rgb_image, mask=mask)
        gray = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)

        # HoughCircles로 원 검출 (파라미터 튜닝: 더 엄격하게)
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=50,  # 가까운 원 제외 (기존 30 → 50)
            param1=50,
            param2=20,  # 더 엄격하게 (기존 15 → 20)
            minRadius=10,  # 최소 반지름 늘림 (기존 8 → 10)
            maxRadius=60
        )

        if circles is not None:
            circles = np.uint16(np.around(circles))
            # 디버깅: 검출된 원 수 로그 출력
            self.get_logger().debug(f'Detected {len(circles[0])} circles')

            # 이전 위치와 가까운 원 선택 (안정화)
            if self.prev_position is not None:
                prev_cx, prev_cy = self.prev_position
                # 거리 계산하여 가장 가까운 원 선택
                closest = min(circles[0, :], key=lambda c: (c[0] - prev_cx)**2 + (c[1] - prev_cy)**2)
                cx, cy, radius = closest
            else:
                # 첫 검출 시 가장 큰 원 선택
                largest = max(circles[0, :], key=lambda c: c[2])
                cx, cy, radius = largest
            
            # 이전 위치 업데이트
            self.prev_position = (cx, cy)
            return (cx, cy), radius
        return None  # 공을 찾지 못하면 None 반환

    def find_red_ball_depth(self, rgb_image, depth_image):
        result = self.find_red_ball(rgb_image)
        if result is not None:
            (cx, cy), radius = result
            # 인덱스 범위 확인 (이미지 크기 초과 방지)
            if 0 <= cy < depth_image.shape[0] and 0 <= cx < depth_image.shape[1]:
                depth_value = depth_image[cy, cx]
                if depth_value > 0:
                    return (cx, cy), depth_value
        return None, None

    def publish_if_changed(self, twist: Twist, action: str):
        """angular.z 변화는 무시하고 선형 속도/액션이 동일하면 발행하지 않음"""
        linear = float(twist.linear.x)
        should_publish = False

        if self.last_linear is None or self.last_published_action is None:
            should_publish = True
        else:
            # 선형 속도가 충분히 차이나면 발행
            if abs(linear - self.last_linear) > self.publish_tolerance:
                should_publish = True
            # 액션 문자열이 바뀌면 발행 (예: stop -> forward)
            elif action != self.last_published_action:
                should_publish = True

        if should_publish:
            self.publisher.publish(twist)
            self.last_linear = linear
            self.last_published_action = action
        else:
            # 디버그 목적으로만 로깅 (기본은 debug)
            self.get_logger().debug('Skipping publish (same linear command; angular.z changes ignored)')

    def stop_movement(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # 모션 상태 및 발행 억제 로직 사용
        self.motion_state = 'stop'
        self.publish_if_changed(twist, 'stop')


def main(args=None):
    rclpy.init(args=args)
    node = DepthRgbFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()