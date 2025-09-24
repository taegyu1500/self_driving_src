## 자율주행 프로젝트

# 기초 정보
이 레포지토리는 자율주행에 사용되는 ros 이미지 클론입니다.
self_drive에 TODO를 각자 브랜치를 열어 구현하고 한번에 합칠 예정입니다.
다른 파일들을 참조하여 코드를 작성하면 됩니다.

# 파일 설명
state.py: 차량이 가질 수 있는 상태값들을 정리
self_drive.py: 차량의 main 함수를 가짐
self_drive.launch.py: 실행할 때 최소한의 node를 실행하기 위해 작성
twistMiddleware.py: state에서 발행된 명령을 받아 모터에 맞는 명령 전달
yolo_detection.py: 신호등, 표지판을 탐지하는 yolo model과 관련된 함수들
lane_detection.py: rgb필터링을 통해 차선을 탐지하고 중앙 정렬을 다시 맞추는 파일
