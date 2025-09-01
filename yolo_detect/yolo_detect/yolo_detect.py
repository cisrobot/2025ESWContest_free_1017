import os
import cv2
import numpy as np
import datetime
from geopy import Point
from geopy.distance import geodesic

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float64, Int8
from cv_bridge import CvBridge
from ultralytics import YOLO

from .sort import Sort  # sort.py가 같은 패키지 내부에 있어야 함

# ─────────────────────────────────────────────────────────────────────
# [녹화 파일 이름 설정]
# - now: 현재 시각(YYYYMMDD_HHMMSS)
# - filename: 녹화 mp4 파일명
# ─────────────────────────────────────────────────────────────────────
now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
filename = f"record_{now}.mp4"

# ─────────────────────────────────────────────────────────────────────
# [투영 변환(원근→BEV) 설정 파라미터]
# - SRC_POINTS: 원본(카메라) 영상의 기준 4점 (픽셀 좌표, 1280x720 기준으로 추정)
# - SQUARE_SIZE: 변환 결과(BEV) 정사각형 한 변의 길이(픽셀)
# - DST_POINTS: BEV 목표 좌표계 상의 4점 (0~SQUARE_SIZE 범위)
# - M: cv2.getPerspectiveTransform으로 구한 3x3 투영행렬
#   ※ 이 행렬은 이후 픽셀 좌표를 BEV 평면으로 사상할 때 사용
# ─────────────────────────────────────────────────────────────────────
SRC_POINTS = np.float32([
    [3, 556],
    [1279, 566],
    [843, 419],
    [421, 419],
])
SQUARE_SIZE = 720
DST_POINTS = np.float32([
    [0, SQUARE_SIZE],
    [SQUARE_SIZE, SQUARE_SIZE],
    [SQUARE_SIZE, 0],
    [0, 0],
])
M = cv2.getPerspectiveTransform(SRC_POINTS, DST_POINTS)

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        # ───────────────────────────────────────────────────────────────
        # [인도 판정 구독]
        # - /scooter_on_sidewalk(Int8): 세그멘테이션 결과로 '인도 위(1)/아님(0)' 신호 수신
        # - self.sidewalk_result: 최근 판정 결과 보관 (0/1)
        # ───────────────────────────────────────────────────────────────
        self.sidewalk_result = 0
        self.sidewalk_sub = self.create_subscription(
            Int8,
            '/scooter_on_sidewalk',
            self.sidewalk_callback,
            10
        )

        # ───────────────────────────────────────────────────────────────
        # [카메라 이미지 구독]
        # - /camera/d455_camera/color/image_raw(Image): D455 컬러 프레임
        # - 콜백: image_callback
        # - 큐: 10
        # ───────────────────────────────────────────────────────────────
        self.subscription = self.create_subscription(
            Image,
            '/camera/d455_camera/color/image_raw',
            self.image_callback,
            10)

        # ───────────────────────────────────────────────────────────────
        # [브리지 및 장치/모델 설정]
        # - CvBridge: ROS Image ↔ OpenCV 변환
        # - self.device: YOLO 추론 장치 지정(ultralytics: 0→GPU:0, 'cpu'도 가능)
        # - self.tracker(SORT):
        #     max_age=60  → 감지 실패 후 몇 프레임 살아남을지(여기선 60프레임 유지)
        #     min_hits=3  → ID 확정 전 최소 연속 감지 횟수
        #     iou_threshold=0.20 → 매칭 IOU 하한(낮게 잡아 널널하게 매칭)
        # - self.detected_coords: 인도 위에서 최초 확정된(한 번만) 좌표 보관
        # - self.gps_history: 트랙별 좌표 이력(MAX_HISTORY로 길이 제한)
        # ───────────────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.device = 0
        self.model = YOLO('/home/marin/marine/src/yolo_detect/models/v11s.pt')
        self.tracker = Sort(max_age=60, min_hits=3, iou_threshold=0.20)
        self.detected_coords = {}
        self.gps_history = {}

        # ───────────────────────────────────────────────────────────────
        # [퍼블리셔]
        # - /scooter_location(NavSatFix): 확정된 킥보드 GPS 좌표 1회 발행(인도 위일 때)
        # - /scooter_pixel_bev(NavSatFix): BEV 상의 픽셀 좌표를 NavSatFix에 임시 적재하여 송신
        #   ※ latitude←v, longitude←u로 '재사용' (프로토타입 용도, 실제 GPS 아님)
        # ───────────────────────────────────────────────────────────────
        self.coord_pub = self.create_publisher(NavSatFix, '/scooter_location', 10)
        self.pixel_coord_pub = self.create_publisher(NavSatFix, '/scooter_pixel_bev', 10)

        # ───────────────────────────────────────────────────────────────
        # [GPS/헤딩 구독]
        # - self.latest_coords: 최신 로봇 GPS (lat, lon)
        # - /mavros/global_position/global(NavSatFix): GPS 좌표
        # - qos_profile_sensor_data: 센서 QoS 프로파일로 지연 최소화
        # - self.compass_heading: 최근 나침반 헤딩(도, 0~360)
        # - /mavros/global_position/compass_hdg(Float64): 헤딩
        # ───────────────────────────────────────────────────────────────
        self.latest_coords = None
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            qos_profile_sensor_data)

        self.compass_heading = 0.0
        self.heading_sub = self.create_subscription(
            Float64,
            '/mavros/global_position/compass_hdg',
            self.heading_callback,
            qos_profile_sensor_data)

        # ───────────────────────────────────────────────────────────────
        # [영상 기록 설정]
        # - output_path: mp4 저장 경로
        # - video_writer: 최초 프레임에서 크기 확정 후 생성
        # - fourcc='mp4v', fps=30.0, frame_size: (w,h)
        # ───────────────────────────────────────────────────────────────
        self.output_path = os.path.join(
            "/media/marin/4cca4ad9-422b-4ad3-b582-3f9c402dd434/home/omo/videos/yolo", filename)
        self.video_writer = None
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.fps = 30.0
        self.frame_size = None

        # ───────────────────────────────────────────────────────────────
        # [로그 파일 초기화]
        # - scooter_location.txt: 확정된 킥보드 좌표 및 이력 기록
        # - 최초 count: 0으로 기록 시작
        # ───────────────────────────────────────────────────────────────
        self.log_file_path = os.path.join("/home/marin/marine/src/yolo_detect", "scooter_location.txt")
        with open(self.log_file_path, 'w') as f:
            f.write("count: 0\n")

    def gps_callback(self, msg: NavSatFix):
        # 최신 GPS 좌표(lat, lon) 갱신
        lat, lon = msg.latitude, msg.longitude
        # 0 근처 값은 무효로 간주(초기화 중 등) → 좌표 사용 안 함
        if abs(lat) < 0.0001 or abs(lon) < 0.0001:
            self.latest_coords = None
            return
        self.latest_coords = (lat, lon)

    def heading_callback(self, msg: Float64):
        # 최근 헤딩(도) 저장 (0~360)
        self.compass_heading = msg.data

    def estimate_scooter_gps(self, lat, lon, heading, x1, y1, x2, y2,
                             image_width=768, hfov_deg=90.0):
        # ───────────────────────────────────────────────────────────────
        # [바운딩박스 → 대략 GPS 추정]
        # - 입력:
        #   lat, lon: 로봇 GPS
        #   heading: 로봇 진행 방향(도)
        #   (x1,y1,x2,y2): bbox 픽셀(리사이즈된 768x768 기준)
        #   image_width: 추론 입력 가로(기본 768)
        #   hfov_deg: 카메라 수평 FOV(도, 기본 90)
        # - 출력: [추정 위도, 경도] 또는 None
        # - 거리 추정: distance ≈ 615 / sqrt(area) (경험적 상수)
        # - 각도 보정: bbox 중심의 수평 오프셋을 각도로 환산 후 heading에 더함
        # ───────────────────────────────────────────────────────────────
        if lat is None or lon is None:
            return None

        width, height = abs(x2 - x1), abs(y2 - y1)
        area = width * height
        if area == 0:
            return None

        distance = 615 / (area ** 0.5)  # bbox가 작을수록 먼 것으로 가정(경험식)
        bbox_center_x = (x1 + x2) / 2
        image_cx = image_width / 2
        deg_per_pixel = hfov_deg / image_width  # 픽셀 → 도(deg) 변환 스케일
        relative_angle = (bbox_center_x - image_cx) * deg_per_pixel  # 좌우 오프셋 각도
        bearing = (heading + relative_angle) % 360  # 최종 방위각(도)
        origin = Point(lat, lon)  # 시작점(현재 로봇 위치)
        destination = geodesic(meters=distance).destination(origin, bearing)  # 거리/방위각으로 목적지 추정
        return [destination.latitude, destination.longitude]

    def update_log_file(self):
        # 로그 파일에 현재까지 확정된 좌표와 각 트랙의 이동 이력을 전부 갱신 기록
        lines = [f"count: {len(self.detected_coords)}\n"]
        for tid, coord in self.detected_coords.items():
            if coord is None:
                lines.append(f"id: {tid} | 위도: N/A | 경도: N/A\n")
            else:
                lat, lon = coord
                lines.append(f"id: {tid} | 위도: {lat:.6f} | 경도: {lon:.6f}\n")
            if tid in self.gps_history:
                lines.append("  └ 이동 기록:\n")
                for i, (lat, lon) in enumerate(self.gps_history[tid]):
                    lines.append(f"     [{i+1:02}] lat: {lat:.6f}, lon: {lon:.6f}\n")
        with open(self.log_file_path, 'w') as f:
            f.writelines(lines)

    def sidewalk_callback(self, msg: Int8):
        # 세그멘테이션 결과(인도 여부) 최신값 저장 (1: 인도 위)
        self.sidewalk_result = msg.data

    def image_callback(self, msg):
        # ───────────────────────────────────────────────────────────────
        # [메인 추론 파이프라인]  ← 줄 단위로 상세 주석
        # ───────────────────────────────────────────────────────────────
        MAX_HISTORY = 15  # 각 트랙의 좌표 이력 최대 보관 개수

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')         # ROS Image → OpenCV BGR
        resized_frame = cv2.resize(frame, (768, 768))          # YOLO 입력 크기에 맞게 리사이즈(768x768)

        # YOLO 추론
        # - imgsz=768: 입력 크기
        # - conf=0.85: confidence 임계값(85% 이상만 채택)
        # - device=self.device: 0이면 GPU:0 사용
        # - verbose=False: 콘솔 출력 억제
        results = self.model.predict(resized_frame, imgsz=768, conf=0.85, device=self.device, verbose=False)

        # 'scooter' 클래스만 필터링하여 [x1,y1,x2,y2,score]로 정리
        detections = [[*box.xyxy[0].tolist(), float(box.conf)] for r in results for box in r.boxes
                      if self.model.names[int(box.cls)] == "scooter"]

        dets_np = np.array(detections) if detections else np.empty((0, 5))  # 감지 없으면 (0,5) 배열
        tracks = self.tracker.update(dets_np)                                # SORT로 트랙 업데이트: [x1,y1,x2,y2,id]

        for track in tracks:
            x1, y1, x2, y2, track_id = track.astype(int)      # 박스 좌표와 ID 정수 변환
            bbox_center_x = (x1 + x2) / 2                      # 박스 중심 x
            # 좌/우 모서리 중 하나를 선택(인접 지면 접점 추정)
            selected_x, selected_y = (x2, y2) if bbox_center_x <= 363 else (x1, y2)

            # 리사이즈(768x768) → 원 해상도(1280x720)로 스케일 복원
            scale_x, scale_y = 1280 / 768, 720 / 768
            x_scaled, y_scaled = selected_x * scale_x, selected_y * scale_y

            # 원근→BEV 투영 (한 점 변환)
            pt = np.array([[[x_scaled, y_scaled]]], dtype=np.float32)
            bev_pt = cv2.perspectiveTransform(pt, M)[0][0]
            u, v = int(bev_pt[0]), int(bev_pt[1])             # BEV 좌표(정수화)

            # BEV 픽셀 좌표를 NavSatFix에 임시 실어 송신( latitude←v, longitude←u )
            pix_msg = NavSatFix()
            pix_msg.header.stamp = self.get_clock().now().to_msg()
            pix_msg.header.frame_id = 'map'
            pix_msg.latitude = float(v)
            pix_msg.longitude = float(u)
            pix_msg.altitude = 0.0
            pix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self.pixel_coord_pub.publish(pix_msg)

            # 현재 로봇 GPS/헤딩 + bbox로 킥보드 GPS 대략 추정
            coord = self.estimate_scooter_gps(*self.latest_coords, self.compass_heading, x1, y1, x2, y2) \
                    if self.latest_coords is not None else None

            if coord is None:
                continue  # 좌표 추정 불가 시 스킵

            # 트랙 이력 저장(길이 제한)
            if track_id not in self.gps_history:
                self.gps_history[track_id] = []
            self.gps_history[track_id].append(coord)
            if len(self.gps_history[track_id]) > MAX_HISTORY:
                self.gps_history[track_id] = self.gps_history[track_id][-MAX_HISTORY:]

            # 인도 위(=1)이고, 아직 한 번도 확정 좌표를 기록하지 않은 트랙이면 1회 확정/발행
            if track_id not in self.detected_coords and self.sidewalk_result == 1:
                self.detected_coords[track_id] = coord        # 최초 확정 좌표 등록
                self.update_log_file()                         # 로그 파일 갱신

                # 확정된 GPS 좌표를 /scooter_location으로 발행(한 번)
                gps_msg = NavSatFix()
                gps_msg.header.stamp = self.get_clock().now().to_msg()
                gps_msg.header.frame_id = 'base_link'
                gps_msg.latitude = coord[0]
                gps_msg.longitude = coord[1]
                gps_msg.altitude = 0.0
                gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                self.coord_pub.publish(gps_msg)

            # 시각화: 바운딩박스/ID 표시(디버그 모니터링)
            cv2.rectangle(resized_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(resized_frame, f"ID: {track_id}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 좌측 상단에 확정된 킥보드 개수 표시
        cv2.putText(resized_frame, f"Scooter count: {len(self.detected_coords)}", (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        # 첫 프레임에서 VideoWriter 생성(해상도 확정 후)
        if self.video_writer is None:
            self.frame_size = (resized_frame.shape[1], resized_frame.shape[0])  # (w,h)
            self.video_writer = cv2.VideoWriter(self.output_path, self.fourcc, self.fps, self.frame_size)
            self.get_logger().info(f'Started recording to: {self.output_path}')
        self.video_writer.write(resized_frame)  # 프레임 기록

        # 디버그 창 출력(실시간 확인)
        cv2.imshow("YOLO_detect", resized_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)         # 노드를 켜둔 채로 콜백 함수들이 계속 실행되도록 유지
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 비디오 자원 정리 및 로그
        if node.video_writer is not None:
            node.video_writer.release()
            node.get_logger().info('Video saved.')
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

