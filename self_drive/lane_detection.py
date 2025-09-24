import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
from geometry_msgs.msg import Twist
# TODO: 카메라에서 들어오는 프레임 값을 받아서 양 쪽 노란 차선을 인식, 차선의 중앙을 따라 주행할 수 있도록 에러 보정을 Twist 메시지로 발행
class LaneDetection(Node):
    def __init__(self):
        super().__init__('lane_detection')
        self.input_image_subscription = self.create_subscription(
            Image, '/ascamera/camera_publisher/rgb0/image', self.image_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.bridge = CvBridge()
    
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #TODO: 이미지 처리해서 노란 차선 인식하고 차선 중앙에 맞춰 주행하도록 에러 보정 Twist 메시지 발행
        
        height, width = cv_image.shape[:2]
        # ROI는 이미지의 좌우 1/4, 하단 1/3 영역
        roi = cv_image[int(height*2/3):height, int(width/4):
                        int(width*3/4)]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        # 노란색 범위 (HSV)
        lower_yellow = (20, 100, 100)
        upper_yellow = (30, 255, 255)
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(roi, (cx, cy), 5, (0, 0, 255), -1)
                
                error = cx - (width / 4)  # ROI의 중앙과의 오차
                twist = Twist()
                twist.angular.z = -float(error) / 100  # 오차에 비례한 각속도
                # 전진속도는 이미 다른곳에서 제어중이니 필요없음
                self.cmd_vel_publisher.publish(twist)
        
def main():
    rclpy.init()
    lane_detection_node = LaneDetection()
    rclpy.spin(lane_detection_node)
    lane_detection_node.destroy_node()
    rclpy.shutdown()