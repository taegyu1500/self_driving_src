import os
from ament_index_python.packages import get_package_share_directory
import traceback
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
# TODO: self_drive 노드를 실행시키기 위한 최소한의 노드를 실행시키는 런치 파일
def launch_setup(context):
    # 환경변수 안전 읽기
    compiled = os.environ.get('need_compile', 'False')
    start = LaunchConfiguration('start', default='true')
    start_arg = DeclareLaunchArgument('start', default_value=start)
    only_line_follow = LaunchConfiguration('only_line_follow', default='false')
    only_line_follow_arg = DeclareLaunchArgument('only_line_follow', default_value=only_line_follow)
    # 준비된 액션들을 안전하게 구성
    actions = [start_arg, only_line_follow_arg]
    # 패키지 경로 안전 조회
    try:
        if str(compiled).lower() == 'true':
            peripherals_package_path = get_package_share_directory('peripherals')
            controller_package_path = get_package_share_directory('controller')
            package_share_directory = get_package_share_directory('yolov5_ros2')
        else:
            peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'
            controller_package_path = '/home/ubuntu/ros2_ws/src/driver/controller'
            package_share_directory = '/home/ubuntu/ros2_ws/src/yolov5_ros2'
    except Exception:
        print('Failed to resolve package paths:', traceback.format_exc())
        peripherals_package_path = None
        controller_package_path = None
        package_share_directory = None

    # IncludeLaunchDescription / Node 생성 시 예외를 무시하고 계속 진행
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')),
    )
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, 'launch/controller.launch.py')),
    )
    try:
        yolov5_node = Node(
            package='yolov5_ros2',
            executable='yolo_detect',
            output='screen',
            parameters=[{'classes': ['sign_straight', 'sign_right', 'sign_parking', 'traffic_light', 'crosswalk', 'arrow_right']},
                {"device": "cpu",
                "model": "traffic_signs",
                "image_topic": "/ascamera/camera_publisher/rgb0/image",
                "camera_info_topic": "/camera/camera_info",
                "camera_info_file": f"{package_share_directory}/config/camera_info.yaml" if package_share_directory else '',
                "pub_result_img": True}]
        )
        actions.append(yolov5_node)
    except Exception:
        print('Failed to add yolov5_node:', traceback.format_exc())

    # self_car 노드들도 안전하게 추가
    for pkg_exec in [('self_car','self_drive'), ('self_car','drive_state'), ('self_car','tmd'), ('self_car','lane_detection')]:
        try:
            node = Node(package=pkg_exec[0], executable=pkg_exec[1], output='screen')
            actions.append(node)
        except Exception:
            print(f'Failed to add node {pkg_exec}:', traceback.format_exc())

    return actions
     
def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])
 
if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()
 
    ls = LaunchService()
    try:
        ls.include_launch_description(ld)
        ls.run()
    except Exception:
        print('LaunchService failed:', traceback.format_exc())

