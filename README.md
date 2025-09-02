# 개발 개요 

<img width="1012" height="824" alt="image" src="https://github.com/user-attachments/assets/02b16401-195a-4242-80e0-2077f7d529c7" />

본 프로젝트는 자율주행 로봇을 활용한 불법 주차 PM(Personal Mobility, 전동킥보드 등) 탐지 시스템을 구축하는 것을 목표로 합니다. 로봇은 보행로를 따라 주행하며, 보행자와 장애물을 회피하고, 딥러닝 기반 객체 탐지를 통해 불법 주차된 PM을 실시간으로 식별합니다. 탐지된 위치 정보는 GPS와 연동되어 서버 및 관리자 전용 애플리케이션으로 전송됩니다.

## 자율주행 (Autonomous Navigation)

로봇은 보행로 환경에서 안정적인 주행을 수행합니다.

보행로 경계 인식: 보행로 경계를 segmentation을 통해 인식하여 주행 가능 영역을 파악합니다.

중앙 주행: 경계 정보를 바탕으로 보행로 중앙을 따라 주행하여 안정성을 확보합니다.

장애물 회피: 보행자, 자전거, 차량 등 예상치 못한 장애물이 나타날 경우, 로컬 경로 재계획(local replanning)을 수행하고 부드럽게 회피합니다.

## 불법 주차 PM 탐지 시스템 구축 (PM Detection System)

로봇은 주행 중 불법 주차된 PM을 실시간으로 탐지하고 위치를 기록합니다.

딥러닝 기반 탐지: yolo 기반 딥러닝 모델을 활용하여 PM을 검출합니다.

GPS 연동: 탐지 시 해당 PM의 GPS 좌표를 로봇 내 시스템에 기록합니다.

데이터 전송: 기록된 탐지 정보는 서버 및 관리자 전용 앱으로 자동 전송되어, 현장 관리자가 즉시 확인할 수 있습니다.

## 주요 특징

자율주행과 불법주차 PM 탐지를 통합한 실시간 보행로 모니터링

GPS 기반 정밀 위치 기록 및 클라우드 연동

관리자 앱을 통한 실시간 알림 및 관리 기능 제공

# 시스템 구성도 

<img width="2010" height="1006" alt="image" src="https://github.com/user-attachments/assets/148f4dd3-d23a-4efb-924c-8e2f24e77fd2" />


# 경로수정 및 모델 다운로드

## nav2_custom

<</nav2_custom/launch/navigation_launch.py>>

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value='/경로/src/nav2_custom/params/nav2_params.yaml',
        description='Full path to the ROS2 parameters file to use for all launched nodes')

## genz_icp

<<< /genz_icp/ros/launch/odometry.launch.py >>> 

    DeclareLaunchArgument("config_file", default_value="/경로/src/genz_icp/ros/config/outdoor.yaml"),

## patrol_node

전용 어플리케이션 보안상의 이유로 json 유형의 파일은 따로 올리지 않았습니다. 
(배포시 무효화 문제로 데이터 통신의 중지를 유발)

<<< /patrol_nodes/patrol_nodes/firestore_bridge.py >>>
    cred = credentials.Certificate(
                    '/경로/src/patrol_nodes/patrol_nodes/json 키'
                )
## Segmentation

https://drive.google.com/file/d/1ajlTMQ3bdAtJHEt_Bup_Fybq22QRtYD1/view?usp=drive_link

1) 구글 링크에서 'best.pt' 파일 다운로드
2) 'best.pt' 파일을 /yolo_segmentation/models 경로에 옮기기


<<< /yolo_segmentation/yolo_segmentation/ros2_segmentation.py >>> 

*세그멘테이션 결과의 BEV 시각화 영상을 저장할 디렉토리 경로

    VIDEO_SAVE_DIR = '경로/..'

*YOLO 세그멘테이션 가중치(.pt) 파일 경로

    MODEL_PATH = '경로/models/best.pt'

   

## Detection


<<< /yolo_detect/yolo_detect/yolo_detect.py >>> 

    self.model = YOLO('/경로/yolo_detect/models/v11s.pt')
    
    self.output_path = os.path.join("/.../경로/", filename)
    
    self.log_file_path = os.path.join("/경로/yolo_detect", "scooter_location.txt")


<<< /yolo_detect/video_recorder.py >>> 

    self.output_dir = '/경로/.../'

