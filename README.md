#Segmentation

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
#Detection

< 경로 수정 >

<<< /yolo_detect/yolo_detect/yolo_detect.py >>> 

self.model = YOLO('/경로/yolo_detect/models/v11s.pt')

self.output_path = os.path.join("/.../경로/", filename)

self.log_file_path = os.path.join("/경로/yolo_detect", "scooter_location.txt")

< 경로 수정 >

<<< /yolo_detect/video_recorder.py >>> 

self.output_dir = '/경로/.../'

========================================================
