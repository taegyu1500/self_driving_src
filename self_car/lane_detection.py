import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
from geometry_msgs.msg import Twist
import numpy as np
import math
import time
from std_msgs.msg import String

class LaneDetection(Node):
    def __init__(self):
        super().__init__('lane_detection')
        self.input_image_subscription = self.create_subscription(
            Image, '/ascamera/camera_publisher/rgb0/image', self.image_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # 상태 구독: GO_FORWARD일 때만 차선 중앙 정렬 적용
        self.state_subscription = self.create_subscription(String, 'state', self.state_callback, 10)
        self.current_state = "IDLE"
        self.bridge = CvBridge()
        # state 변경 최소 간격(초) — 외부와 동일하게 1초로 제한
        self._min_state_change_interval = 1.0
        self._last_state_update = 0.0

        # 제어 파라미터
        self.forward_speed = 0.18        # 직진 속도 (m/s)
        self.lookahead_ratio = 0.6       # lookahead 비율 (ROI 높이에 대한)
        self.Kp = 1.6                    # heading 비례 이득 (검은 도로용으로 조정)
        self.Kd = 0.5                    # 미분 이득
        self.max_angular = 1.0
        self.ema_alpha = 0.7             # 제어값 EMA 계수 (0..1)

        # 내부 상태
        self.prev_heading = 0.0
        self.prev_time = None
        self.prev_angular = 0.0
        self.prev_road_width = None
        self.prev_road_center = None

        # 최신 이미지(신호등 판별에서 사용)
        self.latest_image = None

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warning(f'CvBridge conversion failed: {e}')
            return

        # 최신 프레임 저장 (traffic_light_callback 사용)
        self.latest_image = cv_image.copy()

        # GO_FORWARD일 때만 중앙 정렬 제어 실행
        # 다른 상태(TURN_RIGHT, PARK 등)일 때는 cmd_vel을 발행하지 않아
        # 해당 상태를 처리하는 노드가 제어권을 갖도록 함.
        if getattr(self, 'current_state', '') != "GO_FORWARD":
            return

        img_h, img_w = cv_image.shape[:2]
        if img_h == 0 or img_w == 0:
            return

        # ROI 확장: 하단 + 중간을 포함해서 중앙과 우측(우회전) 영역 확보
        roi_top = int(img_h * 0.55)         # 기존 2/3보다 위로 올려 원거리 정보 확보
        roi_left = int(img_w * 0.06)        # 더 좌측까지 포함
        roi_right = int(img_w * 0.98)       # 우측 끝까지 포함하여 우회전 후보 확보
        roi = cv_image[roi_top:img_h, roi_left:roi_right]
        roi_h, roi_w = roi.shape[:2]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        # 어두운 도로를 직접 픽셀 스캔으로 찾음 (임계값은 조정 가능)
        _, dark_mask = cv2.threshold(blur, 85, 255, cv2.THRESH_BINARY_INV)
        # 소형 노이즈 제거, 너무 큰 closing은 형태 왜곡할 수 있어 적당히 사용
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7,7))
        cleaned = cv2.morphologyEx(dark_mask, cv2.MORPH_OPEN, kernel)

        # 여러 수평 라인에서 가장 긴 연속 어두운 구간의 중심 추적
        scan_rows = np.linspace(roi_h - 1, int(roi_h * 0.35), 12).astype(int)  # 하단->중간
        centers = []
        min_seg_width = max(6, int(roi_w * 0.18))  # 최소 세그먼트 폭 (픽셀 or 비율)
        for y in scan_rows:
            row = cleaned[y, :]
            nz = np.nonzero(row)[0]
            if nz.size == 0:
                continue
            parts = np.split(nz, np.where(np.diff(nz) != 1)[0] + 1)
            # select widest segment (prefer right-biased if similar)
            best_seg = None
            best_w = 0
            for seg in parts:
                w_seg = seg.size
                if w_seg > best_w:
                    best_w = w_seg
                    best_seg = seg
            if best_seg is not None and best_w >= min_seg_width:
                cx = float(best_seg.mean())
                centers.append(cx)

        # 충분한 라인에서 검출되지 않으면 제어권 방해 금지
        if len(centers) < 4:
            self.get_logger().debug(f'not enough road scans ({len(centers)}) -> skip publish')
            self.prev_angular = self.ema_alpha * self.prev_angular
            return

        # 중앙값을 사용하여 아웃라이어 제거
        lane_center = float(np.median(centers))
        # 보정: 우회전이 존재하는 지점에서는 약간 우측을 선호하도록 가중치(환경에 따라 조정)
        # lane_center = lane_center * 0.95 + (roi_w * 0.55) * 0.05
        # 업데이트 이전값
        self.prev_road_width = self.prev_road_width if self.prev_road_width is not None else roi_w
        self.prev_road_center = lane_center

        # lookahead 계산: 이미지 좌표에서 lookahead 거리에 해당하는 y 위치
        lookahead_px = max(1.0, roi_h * self.lookahead_ratio)
        target_y = roi_h - lookahead_px

        # 단순화: 직사각형의 중심을 현재 기준으로 사용. 더 정확히는 contour에서 target_y의 x를 샘플링 가능.


        # error, desired heading, PD 제어
        error_px = lane_center - (roi_w / 2.0)
        desired_heading = math.atan2(error_px, lookahead_px)
        now = time.time()
        dt = 0.05
        if self.prev_time is not None:
            dt = max(1e-3, now - self.prev_time)
        self.prev_time = now

        P = self.Kp * desired_heading
        D = self.Kd * ((desired_heading - self.prev_heading) / dt)
        raw_ang = P + D
        self.prev_heading = desired_heading

        raw_ang = max(-self.max_angular, min(self.max_angular, raw_ang))
        ang = self.ema_alpha * self.prev_angular + (1.0 - self.ema_alpha) * raw_ang
        self.prev_angular = ang

        # 주의: 직진(linear.x)은 다른 노드(state 머신 등)이 제어한다고 가정.
        # lane_detection은 각속도(angular.z)만 발행하여 중앙 유지 제어 역할만 수행.
        twist = Twist()
        twist.angular.z = -ang  # 필요 시 부호 반전
        self.cmd_vel_publisher.publish(twist)
        # 짧은 디버그 로그
        self.get_logger().debug(f'road scans={len(centers)} median_cx={lane_center:.1f} err_px={error_px:.1f} ang={twist.angular.z:.3f}')

    def traffic_light_callback(self):
        cv_image = self.latest_image
        if cv_image is None:
            return
        height, width = cv_image.shape[:2]
        if height == 0 or width == 0:
            return
        # 신호등의 불빛을 체크해서 빨간불/초록불 판단
        roi = cv_image[0:int(height/2), int(width*2/3):width]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_red1 = (0, 100, 100)
        upper_red1 = (10, 255, 255)
        lower_red2 = (160, 100, 100)
        upper_red2 = (180, 255, 255)
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        lower_green = (40, 100, 100)
        upper_green = (90, 255, 255)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        red_count = cv2.countNonZero(mask_red)
        green_count = cv2.countNonZero(mask_green)
        threshold = 500
        if red_count > threshold and red_count > green_count:
            self.get_logger().info('Traffic light: RED detected')
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
        elif green_count > threshold and green_count > red_count:
            self.get_logger().info('Traffic light: GREEN detected')
            return

    def state_callback(self, msg):
        try:
            if msg is None or not hasattr(msg, 'data'):
                return
            val = msg.data
            new_state = val.strip() if isinstance(val, str) else str(val)
            now = time.time()
            # 최소 변화 간격 적용
            if now - self._last_state_update < self._min_state_change_interval:
                self.get_logger().debug(f'state update suppressed (too fast): {new_state}')
                return
            # 실제 업데이트
            self.current_state = new_state
            self._last_state_update = now
            self.get_logger().info(f'state set to {self.current_state}')
        except Exception:
            self.get_logger().warning('Failed to parse state message')

def main():
    rclpy.init()
    lane_detection_node = LaneDetection()
    rclpy.spin(lane_detection_node)
    lane_detection_node.destroy_node()
    rclpy.shutdown()