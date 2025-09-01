# 시스템 구성도 

<img width="2010" height="1006" alt="image" src="https://github.com/user-attachments/assets/148f4dd3-d23a-4efb-924c-8e2f24e77fd2" />


========================================================

# nav2_custom

<</nav2_custom/launch/navigation_launch.py>>

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value='/경로/src/nav2_custom/params/nav2_params.yaml',
        description='Full path to the ROS2 parameters file to use for all launched nodes')

# genz_icp

<<< /genz_icp/ros/launch/odometry.launch.py >>> 

    DeclareLaunchArgument("config_file", default_value="/경로/src/genz_icp/ros/config/outdoor.yaml"),

# Segmentation

https://drive.google.com/file/d/1ajlTMQ3bdAtJHEt_Bup_Fybq22QRtYD1/view?usp=drive_link

1) 구글 링크에서 'best.pt' 파일 다운로드
2) 'best.pt' 파일을 /yolo_segmentation/models 경로에 옮기기

< 경로 수정 >

<<< /yolo_segmentation/yolo_segmentation/ros2_segmentation.py >>> 

*세그멘테이션 결과의 BEV 시각화 영상을 저장할 디렉토리 경로
    VIDEO_SAVE_DIR = '경로/..'

*YOLO 세그멘테이션 가중치(.pt) 파일 경로
    MODEL_PATH = '경로/models/best.pt'

   
========================================================

# Detection

< 경로 수정 >

<<< /yolo_detect/yolo_detect/yolo_detect.py >>> 

    self.model = YOLO('/경로/yolo_detect/models/v11s.pt')
    
    self.output_path = os.path.join("/.../경로/", filename)
    
    self.log_file_path = os.path.join("/경로/yolo_detect", "scooter_location.txt")

< 경로 수정 >

<<< /yolo_detect/video_recorder.py >>> 

    self.output_dir = '/경로/.../'

========================================================
