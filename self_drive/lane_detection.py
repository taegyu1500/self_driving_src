import rclpy
from rclpy.node import Node

# TODO: 카메라에서 들어오는 프레임 값을 받아서 양 쪽 노란 차선을 인식, 차선의 중앙을 따라 주행할 수 있도록 에러 보정을 Twist 메시지로 발행
