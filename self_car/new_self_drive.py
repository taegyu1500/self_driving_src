#!/usr/bin/env python3
# encoding: utf-8
# 통합된 자율주행 노드 - 메모리 최적화
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from interfaces.msg import ObjectsInfo
from std_msgs.msg import String
import cv2
import numpy as np
import math
import time
import threading
import os
from collections import defaultdict

class OptimizedSelfDriving(Node):
    def __init__(self):
        super().__init__('optimized_self_driving')
        self.get_logger().info('Optimized Self Driving Node initialized.')
        
        # 콜백 그룹 설정
        self.cbg = ReentrantCallbackGroup()
        
        # 공통 상태 관리
        self.current_state = "IDLE"
        self._stopped = True
        self._lock = threading.RLock()
        
        # OpenCV Bridge (하나만 사용)
        self.bridge = CvBridge()
        self.latest_image = None
        
        # 상태별 지속 시간
        self.ACTION_DURATIONS = {
            "TURN_RIGHT": 3.0,
            "PARK": 3.0
        }
        
        # 액션 타이머
        self._action_start_time = 0.0
        self._current_action_duration = 0.0
        self._park_complete_time = 0.0
        
        # 차선 추적 파라미터
        self.Kp = 1.6
        self.Kd = 0.5
        self.max_angular = 1.0
        self.ema_alpha = 0.7
        self.prev_heading = 0.0
        self.prev_time = None
        self.prev_angular = 0.0
        
        # 객체 감지 관리 (메모리 효율화)
        self._detection_counts = defaultdict(int)
        self._detection_thresholds = {
            'sign_straight': 2, 'sign_right': 2, 'sign_parking': 2,
            'crosswalk': 2, 'arrow_right': 2
        }
        self._last_detection_time = defaultdict(lambda: 0.0)
        self._detection_cooldowns = {
            'sign_straight': 2.0, 'sign_right': 2.0, 'sign_parking': 2.0,
            'crosswalk': 4.0, 'arrow_right': 2.0
        }
        
        # 초록불 감지 (단순화)
        self._green_count = 0
        self._green_threshold = 3
        self._green_pixel_min = 400
        
        # 머신 타입 설정
        self.machine_type = os.environ.get('MACHINE_TYPE', '')
        
        # Publishers/Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.image_sub = self.create_subscription(
            Image, '/ascamera/camera_publisher/rgb0/image', 
            self.image_callback, 1, callback_group=self.cbg)
        
        self.detection_sub = self.create_subscription(
            ObjectsInfo, '/yolov5_ros2/object_detect',
            self.detection_callback, 1, callback_group=self.cbg)
        
        # 메인 제어 루프 (20Hz로 최적화)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # 시작 후 잠시 대기
        self.create_timer(1.0, self._delayed_start)
        
    def _delayed_start(self):
        """시작 후 GO_FORWARD로 전환"""
        if self.current_state == "IDLE":
            self._stopped = False
            self.current_state = "GO_FORWARD"
            self.get_logger().info('Starting autonomous driving -> GO_FORWARD')
    
    def image_callback(self, msg):
        """이미지 콜백 - 최소한의 처리만"""
        try:
            # 메모리 효율화를 위해 desired_encoding 명시
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image
            
            # 정지 상태에서만 초록불 체크
            if self._stopped:
                self._check_green_light(cv_image)
                
        except Exception as e:
            self.get_logger().debug(f'Image callback error: {e}')
    
    def _check_green_light(self, image):
        """초록불 감지 (경량화)"""
        try:
            h, w = image.shape[:2]
            if h == 0 or w == 0:
                return
                
            # ROI 축소로 메모리 절약
            roi = image[0:int(h/3), int(w*2/3):w]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            
            # 초록색 마스크
            mask_green = cv2.inRange(hsv, (40,100,100), (90,255,255))
            green_pixels = int(cv2.countNonZero(mask_green))
            
            if green_pixels > self._green_pixel_min:
                self._green_count += 1
            else:
                self._green_count = 0
                
            if self._green_count >= self._green_threshold:
                self._stopped = False
                self.current_state = "GO_FORWARD"
                self._green_count = 0
                self.get_logger().info('Green light detected -> GO_FORWARD')
                
        except Exception:
            pass
    
    def detection_callback(self, msg):
        """객체 감지 콜백 - 경량화"""
        try:
            if self._stopped:
                return
                
            objects = getattr(msg, 'objects', []) or []
            if not objects:
                return
                
            now = time.time()
            detected_classes = {}
            
            # 객체별로 정리 (메모리 효율화)
            for obj in objects:
                class_name = getattr(obj, 'class_name', '')
                if class_name in self._detection_thresholds:
                    if class_name not in detected_classes:
                        detected_classes[class_name] = []
                    detected_classes[class_name].append(obj)
            
            # 감지 카운터 업데이트
            for class_name in self._detection_thresholds:
                if class_name in detected_classes:
                    self._detection_counts[class_name] = min(
                        self._detection_counts[class_name] + 1,
                        self._detection_thresholds[class_name]
                    )
                else:
                    self._detection_counts[class_name] = max(
                        self._detection_counts[class_name] - 1, 0
                    )
            
            # 액션 트리거 체크
            for class_name, threshold in self._detection_thresholds.items():
                if (self._detection_counts[class_name] >= threshold and
                    now - self._last_detection_time[class_name] > self._detection_cooldowns[class_name]):
                    
                    self._handle_detection(class_name, detected_classes.get(class_name, []))
                    self._last_detection_time[class_name] = now
                    self._detection_counts[class_name] = 0
                    
        except Exception as e:
            self.get_logger().debug(f'Detection callback error: {e}')
    
    def _handle_detection(self, class_name, objects):
        """감지된 객체 처리"""
        if not objects:
            return
            
        # 가장 큰 객체 선택 (면적 기준)
        best_obj = max(objects, key=self._get_object_area)
        
        with self._lock:
            if class_name == 'sign_right' or class_name == 'arrow_right':
                if self._is_object_close(best_obj):
                    self._start_action("TURN_RIGHT")
            elif class_name == 'sign_parking':
                if self._is_object_close(best_obj):
                    self._start_action("PARK")
            elif class_name == 'crosswalk':
                if self._is_object_close(best_obj):
                    self._start_action("IDLE")  # 잠시 정지
    
    def _get_object_area(self, obj):
        """객체 면적 계산"""
        try:
            if hasattr(obj, 'box') and len(obj.box) >= 4:
                return float(obj.box[2]) * float(obj.box[3])
        except Exception:
            pass
        return 0.0
    
    def _is_object_close(self, obj):
        """객체가 충분히 가까운지 판단"""
        try:
            if hasattr(obj, 'box') and len(obj.box) >= 4:
                # 정규화된 좌표 가정
                cy = obj.box[1] + obj.box[3] / 2.0
                h = obj.box[3]
                
                # 픽셀 좌표인 경우 정규화
                if cy > 1.0 or h > 1.0:
                    img_h = getattr(obj, 'height', 480)
                    cy = cy / img_h
                    h = h / img_h
                    
                return cy >= 0.6 or h >= 0.06
        except Exception:
            pass
        return False
    
    def _start_action(self, action):
        """액션 시작"""
        if action == self.current_state:
            return
            
        self.current_state = action
        self._action_start_time = time.time()
        
        if action in self.ACTION_DURATIONS:
            self._current_action_duration = self.ACTION_DURATIONS[action]
            if action == "PARK":
                self._park_complete_time = self._action_start_time + self._current_action_duration
        else:
            self._current_action_duration = 0.0
            
        self.get_logger().info(f'Action started: {action}')
    
    def control_loop(self):
        """메인 제어 루프"""
        try:
            now = time.time()
            twist = Twist()
            
            with self._lock:
                # 액션 타이머 체크
                if (self._current_action_duration > 0 and 
                    now - self._action_start_time > self._current_action_duration):
                    
                    if self.current_state == "TURN_RIGHT":
                        self.current_state = "GO_FORWARD"
                        self.get_logger().info('Turn complete -> GO_FORWARD')
                    elif self.current_state == "PARK":
                        self._stopped = True
                        self.current_state = "IDLE"
                        self.get_logger().info('Park complete -> STOPPED')
                    elif self.current_state == "IDLE" and not self._stopped:
                        # 횡단보도 정지 후 재시작
                        self.current_state = "GO_FORWARD"
                        
                    self._current_action_duration = 0.0
                
                # 상태별 제어
                if self.current_state == "GO_FORWARD":
                    twist.linear.x = 0.18
                    # 차선 추적
                    angular_z = self._calculate_lane_following()
                    if angular_z is not None:
                        twist.angular.z = angular_z
                        
                elif self.current_state == "TURN_RIGHT":
                    twist.linear.x = 0.0
                    twist.angular.z = -1.0
                    
                elif self.current_state == "PARK":
                    # 간단한 주차 동작
                    self._execute_park_sequence(twist, now)
                    
                elif self.current_state == "IDLE":
                    # 정지 또는 일시 정지
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
            
            # cmd_vel 발행
            self.cmd_vel_pub.publish(twist)
            
        except Exception as e:
            self.get_logger().debug(f'Control loop error: {e}')
    
    def _calculate_lane_following(self):
        """차선 추적 계산 (경량화)"""
        if self.latest_image is None:
            return None
            
        try:
            img_h, img_w = self.latest_image.shape[:2]
            if img_h == 0 or img_w == 0:
                return None
            
            # ROI 설정 (메모리 절약을 위해 축소)
            roi_top = int(img_h * 0.6)
            roi = self.latest_image[roi_top:img_h, :]
            roi_h, roi_w = roi.shape[:2]
            
            # 그레이스케일 변환
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5,5), 0)
            
            # 어두운 도로 감지
            _, dark_mask = cv2.threshold(blur, 85, 255, cv2.THRESH_BINARY_INV)
            
            # 중앙선 찾기 (간소화)
            scan_rows = np.linspace(roi_h - 1, int(roi_h * 0.5), 6).astype(int)
            centers = []
            
            for y in scan_rows:
                row = dark_mask[y, :]
                nz = np.nonzero(row)[0]
                if nz.size > int(roi_w * 0.1):  # 최소 폭 체크
                    centers.append(float(nz.mean()))
            
            if len(centers) < 3:
                return None
                
            # 중앙값 사용
            lane_center = float(np.median(centers))
            
            # PID 제어
            error_px = lane_center - (roi_w / 2.0)
            lookahead_px = roi_h * 0.6
            desired_heading = math.atan2(error_px, lookahead_px)
            
            now = time.time()
            dt = 0.05
            if self.prev_time is not None:
                dt = max(1e-3, now - self.prev_time)
            self.prev_time = now
            
            P = self.Kp * desired_heading
            D = self.Kd * (desired_heading - self.prev_heading) / dt
            raw_angular = P + D
            
            self.prev_heading = desired_heading
            
            # 제한 및 EMA 필터링
            raw_angular = max(-self.max_angular, min(self.max_angular, raw_angular))
            angular_z = self.ema_alpha * self.prev_angular + (1.0 - self.ema_alpha) * raw_angular
            self.prev_angular = angular_z
            
            return -angular_z
            
        except Exception:
            return None
    
    def _execute_park_sequence(self, twist, now):
        """주차 시퀀스 실행 (간소화)"""
        elapsed = now - self._action_start_time
        
        if self.machine_type == 'MentorPi_Mecanum':
            # 메카넘 휠: 좌측으로 이동
            twist.linear.y = -0.2
        elif self.machine_type == 'MentorPi_Acker':
            # 아커만: 3단계 주차
            if elapsed < 1.5:
                twist.linear.x = 0.15
                twist.angular.z = -0.8
            elif elapsed < 2.5:
                twist.linear.x = 0.15
                twist.angular.z = 0.8
            else:
                twist.linear.x = -0.1
        else:
            # 기본: 회전 후 전진
            if elapsed < 1.5:
                twist.angular.z = -1.0
            elif elapsed < 2.5:
                twist.linear.x = 0.2
            else:
                twist.angular.z = 1.0

def main():
    rclpy.init()
    node = OptimizedSelfDriving()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()