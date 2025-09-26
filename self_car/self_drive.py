import rclpy
from rclpy.node import Node
from interfaces.msg import ObjectsInfo
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import threading
import time
from collections import defaultdict
from rclpy.callback_groups import ReentrantCallbackGroup

# 단순화/최적화 포인트
# - bbox 정규화 헬퍼로 중복 제거
# - 클래스별 확인(frames) + 쿨다운 일반화
# - 콜백은 가볍게 유지(데이터 추출만), 실제 결정은 publish_order에서 처리
# - 타이머 일회성 구현 간소화

class SelfDrive(Node):
    def __init__(self, name='self_drive'):
        super().__init__(name)
        self.get_logger().info('Self Drive initialized.')

        # 상태
        self.current_order = "IDLE"
        self.order_queue = []            # (order, reason)
        self.objects_info = []
        self._lock = threading.RLock()

        # 액션 락 / 지속시간
        self._action_lock_until = 0.0
        self.ACTION_DURATIONS = {"TURN_RIGHT": 1.0, "PARK": 3.0}

        # 정지/시작(주차 후 정지, 초록불로 자동 시작)
         # 기본적으로 정지 상태로 시작하도록 변경
        self._stopped = True
        self._park_timer = None
        self._start_on_green = True
        self._green_confirm_frames = 3
        self._green_count = 0
        self._green_pixel_threshold = 400

        # state 최소 변화 단위(초) — 프레임 단위로 상태가 바뀌지 않도록 보장
        self._min_state_change_interval = 1.0
        self._last_state_change_time = time.time()

        # 감지 안정화 (연속 프레임 확인 + 쿨다운)
        self._seen_counts = defaultdict(int)
        self._seen_thresholds = {
            'sign_straight': 2, 'sign_right': 2, 'sign_parking': 2,
            'crosswalk': 2, 'arrow_right': 2, 'traffic_light': 2
        }
        self._seen_cooldowns = {
            'sign_straight': 2.0, 'sign_right': 2.0, 'sign_parking': 2.0,
            'crosswalk': 4.0, 'arrow_right': 2.0, 'traffic_light': 1.0
        }
        self._last_trigger_time = defaultdict(lambda: 0.0)

        # sign specific
        self._sign_close_y_thresh = 0.6
        self._sign_min_height_frac = 0.06
        self._sign_last_pub = {}

        # arrow specific
        self._arrow_close_y_thresh = 0.6
        self._arrow_min_height_frac = 0.06
        self._arrow_last_pub = 0.0

        # crossing specific
        self._crossing_last_action_time = 0.0
        self._crossing_action_cooldown = 4.0
        self._last_crossing_score = 0.0
        self._last_crossing_time = 0.0

        # CV / image
        self.bridge = CvBridge()
        self.latest_image = None
        # 허용 명령 집합과 큐 중복/충돌 제어
        self._allowed_orders = {"IDLE", "GO_FORWARD", "TURN_RIGHT", "PARK"}
        self._queue_max_age = 2.0  # 큐에 들어간 후 오래된 항목 무시(초)
        self._order_enqueue_time = {}  # order -> timestamp

        # publishers/subscriptions
        cbg = ReentrantCallbackGroup()
        self.order_publisher = self.create_publisher(String, 'order', 10)
        self.detection_sub = self.create_subscription(
            ObjectsInfo, '/yolov5_ros2/object_detect', self.detection_callback, 10, callback_group=cbg)
        self.image_sub = self.create_subscription(
            Image, '/ascamera/camera_publisher/rgb0/image', self._image_cb, 10, callback_group=cbg)

        # periodic order publisher
        self._started = False
        self._startup_timer = self.create_timer(0.5, self._delayed_startup, callback_group=cbg)

        # 초기 상태를 명확히 알리기 (정지 -> IDLE 발행)
        try:
            self.order_publisher.publish(String(data=self.current_order))
        except Exception:
            pass

    def _delayed_startup(self):
        if self._started:
            return
        self._started = True
        try:
            self._startup_timer.cancel()
        except Exception:
            pass
        self.create_timer(0.5, self.publish_order)  # publish 2Hz

    # --- helpers ---
    def _norm_bbox(self, obj):
        """Return (cy_norm, h_norm) in 0..1 if possible, else (None,None)."""
        try:
            if hasattr(obj, 'ymin') and hasattr(obj, 'ymax'):
                ymin = float(getattr(obj, 'ymin'))
                ymax = float(getattr(obj, 'ymax'))
                cy = (ymin + ymax) / 2.0
                h = ymax - ymin
            elif hasattr(obj, 'y') and hasattr(obj, 'h'):
                cy = float(getattr(obj, 'y')) + float(getattr(obj, 'h')) / 2.0
                h = float(getattr(obj, 'h'))
            elif hasattr(obj, 'box') and isinstance(obj.box, (list, tuple)) and len(obj.box) >= 4:
                bx = obj.box
                cy = float(bx[1]) + float(bx[3]) / 2.0
                h = float(bx[3])
            else:
                return None, None
            # normalize if appears in pixels (>1)
            if cy > 1.0 or h > 1.0:
                img_h = getattr(obj, 'image_height', None) or getattr(obj, 'img_h', None)
                if img_h:
                    cy = cy / float(img_h)
                    h = h / float(img_h)
                else:
                    cy = cy / 480.0
                    h = h / 480.0
            return float(cy), float(h)
        except Exception:
            return None, None

    def _area_estimate(self, obj):
        try:
            if hasattr(obj, 'box') and isinstance(obj.box, (list, tuple)) and len(obj.box) >= 4:
                return float(obj.box[2]) * float(obj.box[3])
            if hasattr(obj, 'xmin') and hasattr(obj, 'xmax') and hasattr(obj, 'ymin') and hasattr(obj, 'ymax'):
                return (float(obj.xmax)-float(obj.xmin))*(float(obj.ymax)-float(obj.ymin))
        except Exception:
            pass
        return 0.0

    def _one_shot_timer(self, duration, callback):
        # create_timer wrapper for one-shot
        t = None
        def _wrap():
            try:
                callback()
            finally:
                try:
                    if t is not None:
                        t.cancel()
                except Exception:
                    pass
        t = self.create_timer(duration, _wrap)
        return t

    def _set_stopped_after_park(self, duration):
        # cancel existing
        try:
            if self._park_timer:
                self._park_timer.cancel()
        except Exception:
            pass
        def _enter_stopped():
            try:
                if self._park_timer:
                    self._park_timer.cancel()
            except Exception:
                pass
            self._park_timer = None
            self._stopped = True
            self.get_logger().info('Park complete -> stopped')
        self._park_timer = self._one_shot_timer(duration, _enter_stopped)

    # --- image subscription (store latest, handle green->start when stopped) ---
    def _image_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = img
        except Exception:
            return
        if not self._start_on_green or not self._stopped:
            return
        # check green ROI only when stopped
        h, w = img.shape[:2]
        if h == 0 or w == 0:
            return
        roi = img[0:int(h/2), int(w*2/3):w]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask_green = cv2.inRange(hsv, (40,100,100), (90,255,255))
        gcount = int(cv2.countNonZero(mask_green))
        if gcount > self._green_pixel_threshold:
            self._green_count += 1
        else:
            self._green_count = 0
        if self._green_count >= self._green_confirm_frames:
            self._stopped = False
            self._green_count = 0
            self.current_order = "GO_FORWARD"
            self.order_queue.clear()
            try:
                if self._park_timer:
                    self._park_timer.cancel()
            except Exception:
                pass
            self.get_logger().info('Green detected -> resume GO_FORWARD')
            self.order_publisher.publish(String(data=self.current_order))

    # --- detection processing (stabilize, then dispatch to handlers) ---
    def detection_callback(self, msg):
        try:
            if time.time() < self._action_lock_until:
                return
            objs = getattr(msg, 'objects', []) or []
            present = {}
            for o in objs:
                cname = getattr(o, 'class_name', None)
                if cname:
                    present.setdefault(cname, []).append(o)
            now = time.time()
            with self._lock:
                # update seen counters
                for cname in self._seen_thresholds:
                    if cname in present:
                        self._seen_counts[cname] = min(self._seen_counts[cname] + 1, self._seen_thresholds[cname])
                    else:
                        self._seen_counts[cname] = max(self._seen_counts[cname] - 1, 0)
                # trigger for those reached threshold and not in cooldown
                for cname, threshold in self._seen_thresholds.items():
                    if self._seen_counts[cname] >= threshold:
                        if now - self._last_trigger_time[cname] < self._seen_cooldowns.get(cname, 0.0):
                            continue
                        objs_list = present.get(cname, [])
                        target = max(objs_list, key=self._area_estimate) if objs_list else None
                        # dispatch
                        if cname.startswith('sign'):
                            if target:
                                self._handle_sign(target)
                        elif cname == 'crosswalk' and target:
                            self._handle_crossing(target)
                        elif cname == 'arrow_right' and target:
                            self._handle_arrow(target)
                        elif cname == 'traffic_light':
                            # traffic light handled from image usually; still record trigger time
                            pass
                        self._last_trigger_time[cname] = now
                        self._seen_counts[cname] = 0
        except Exception as e:
            self.get_logger().error(f'detection_callback error: {e}')

    # --- handlers (only queue orders; keep light) ---
    def _handle_sign(self, obj):
        try:
            sign_name = getattr(obj, 'class_name', '')
            cy, h = self._norm_bbox(obj)
            if cy is None or h is None:
                return
            close = (cy >= self._sign_close_y_thresh) or (h >= self._sign_min_height_frac)
            now = time.time()
            last = self._sign_last_pub.get(sign_name, 0.0)
            if close and (now - last) > self._seen_cooldowns.get(sign_name, 1.0):
                if sign_name == 'sign_straight':
                    self._enqueue_order("GO_FORWARD", "sign")
                elif sign_name == 'sign_right':
                    self._enqueue_order("TURN_RIGHT", "sign")
                elif sign_name == 'sign_parking':
                    self._enqueue_order("PARK", "sign")
                self._sign_last_pub[sign_name] = now
        except Exception:
            pass

    def _handle_crossing(self, obj):
        try:
            # select closest by normalized cy,h
            cy, h = self._norm_bbox(obj)
            if cy is None or h is None:
                return
            now = time.time()
            if now - self._crossing_last_action_time < self._crossing_action_cooldown:
                return
            score = h * 0.7 + cy * 0.3
            if cy >= 0.6 or h >= 0.08:
                # prevent repeated for same crossing
                if (now - self._last_crossing_time) < 6.0 and abs(score - self._last_crossing_score) < 0.02:
                    return
                self._enqueue_order("IDLE", "crossing")
                self._crossing_last_action_time = now
                self._last_crossing_score = score
                self._last_crossing_time = now
        except Exception:
            pass

    def _handle_arrow(self, obj):
        try:
            cy, h = self._norm_bbox(obj)
            if cy is None or h is None:
                return
            now = time.time()
            close = (cy >= self._arrow_close_y_thresh) or (h >= self._arrow_min_height_frac)
            if close and (now - self._arrow_last_pub) > self._seen_cooldowns.get('arrow_right', 3.0):
                self._enqueue_order("TURN_RIGHT", "sign")
                self._arrow_last_pub = now
        except Exception:
            pass

    # --- enqueue helper: 중복/충돌 제어 ---
    def _enqueue_order(self, order, reason):
        """Queue에 넣기 전 필터: 허용 명령만, 중복 제거, 최근 동일 명령 쿨다운,
           큐 항목 오래되면 교체 등으로 떨림 방지."""
        try:
            if order not in self._allowed_orders:
                return
            now = time.time()
            # 이미 현재 명령이면 무시
            if order == self.current_order:
                return
            # 최근에 같은 명령을 발행한 적 있으면 무시(재발행 쿨다운)
            last = self._order_enqueue_time.get(order, 0.0)
            if now - last < self._seen_cooldowns.get(order.lower(), 1.0):
                return
            # 큐 안에 같은 종류가 있으면 갱신(최신 타임스탬프로)
            for i, (o, r) in enumerate(self.order_queue):
                if o == order:
                    self.order_queue[i] = (order, reason)
                    self._order_enqueue_time[order] = now
                    return
            # conflict resolution: 예를 들어 PARK이 있으면 TURN_RIGHT 등은 추가하지 않음
            # (우선순위는 publish 단계에서 최종 선택함 — 여기서는 단순 차단만)
            if ("PARK", "sign") in self.order_queue and order != "PARK":
                return
            # append new
            self.order_queue.append((order, reason))
            self._order_enqueue_time[order] = now
            # purge stale queue entries
            self._purge_old_queue_entries()
        except Exception:
            pass

    def _purge_old_queue_entries(self):
        now = time.time()
        newq = []
        for (o, r) in self.order_queue:
            t = self._order_enqueue_time.get(o, 0.0)
            if now - t <= self._queue_max_age:
                newq.append((o, r))
            else:
                # drop stale timestamp
                try:
                    del self._order_enqueue_time[o]
                except Exception:
                    pass
        self.order_queue = newq

    # --- order publishing ---
    def publish_order(self):
         try:
            now = time.time()
            # action lock: keep current
            if now < self._action_lock_until:
                self.order_publisher.publish(String(data=self.current_order))
                return

            # 결정할 원하는 주문(desired_order)을 먼저 계산
            if self._stopped:
                desired = "IDLE"
            else:
                if self.order_queue:
                    # collapse queue -> latest per order and pick highest priority
                    # priority small -> higher precedence
                    priority = {"traffic_light": 1, "crossing": 2, "sign": 3}
                    # build map order->reason keeping newest timestamp
                    order_map = {}
                    for (o, r) in self.order_queue:
                        order_map[o] = r
                    # choose by priority among unique orders
                    candidates = list(order_map.items())
                    candidates.sort(key=lambda x: priority.get(x[1], 99))
                    desired, reason = candidates[0]
                    # clear queue fully after selection to avoid repeats
                    self.order_queue.clear()
                else:
                    desired = "GO_FORWARD"

            # 변화 허용 검사: 최소 변화 간격 적용
            if desired != self.current_order:
                if now - self._last_state_change_time < self._min_state_change_interval:
                    # 변화가 너무 잦으면 현 상태를 유지
                    self.order_publisher.publish(String(data=self.current_order))
                    return
                # 허용된 변화: 적용 및 시간 기록
                self.current_order = desired
                self._last_state_change_time = now
                # 액션 락/주차 처리
                if self.current_order in self.ACTION_DURATIONS:
                    self._action_lock_until = now + self.ACTION_DURATIONS[self.current_order]
                    if self.current_order == "PARK":
                        self._set_stopped_after_park(self.ACTION_DURATIONS["PARK"])

            # publish 현재 상태(변경 되었든 유지 되었든)
            self.order_publisher.publish(String(data=self.current_order))
         except Exception as e:
             self.get_logger().error(f'publish_order error: {e}')

def main():
    rclpy.init()
    node = SelfDrive('self_drive')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()