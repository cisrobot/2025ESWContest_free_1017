#!/usr/bin/env python3
import os
import cv2
import numpy as np
from datetime import datetime
import tf_transformations
from geometry_msgs.msg import Quaternion
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from cv_bridge import CvBridge

from ultralytics import YOLO
import torch
from tf2_ros import Buffer, TransformListener, LookupException
from builtin_interfaces.msg import Time as BuiltinTime
from tf_transformations import quaternion_from_euler
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration

# ========================== 파라미터(설정값) ==========================
VIDEO_SAVE_DIR = '/media/marin/4cca4ad9-422b-4ad3-b582-3f9c402dd434/home/omo/videos/seg'
# 세그멘테이션 결과의 BEV 시각화 영상을 저장할 디렉토리 경로

MODEL_PATH = '/home/marin/marine/src/yolo_segmentation/models/best.pt'
# YOLO 세그멘테이션 가중치(.pt) 파일 경로

CONF_THRESHOLD = 0.300
# YOLO 탐지 신뢰도 임계값(이 값 이상만 유효 탐지로 처리)

IMG_SIZE = 640
# YOLO 입력 이미지 크기(640x640). 속도/정확도 균형에 영향

SIDEWALK_CLASS_ID = 0
# YOLO에서 인도(sidewalk) 클래스의 라벨 ID

ALPHA = 0.5
# 시각화 시 원본과 마스크를 합성할 때의 투명도 (0~1)

MASK_COLOR = (0, 0, 255)
# 인도 마스크 색상 (BGR, 빨강)

CONTOUR_COLOR = (0, 255, 0)
# 인도 윤곽선 색상 (BGR, 초록)

camera_matrix = np.array([
    [910.0, 0.0, 640.0],
    [0.0, 910.0, 360.0],
    [0.0, 0.0, 1.0]
], dtype=np.float32)
# 카메라 내부 파라미터(초점거리 fx, fy / 주점 cx, cy)

dist_coeffs = np.array([-0.04, 0.02, 0.0, 0.0, 0.0], dtype=np.float32)
# 렌즈 왜곡 계수 [k1, k2, p1, p2, k3]

# 230x230용 원근변환 입력 좌표(원본 이미지 상의 4점)
SRC_POINTS = np.float32([
    [3, 556],
    [1279, 566],
    [843, 419],
    [421, 419],
])

SQUARE_SIZE = 720
# BEV로 투영한 출력 정사각 영상의 한 변 크기(px)

DST_POINTS = np.float32([
    [0, SQUARE_SIZE],
    [SQUARE_SIZE, SQUARE_SIZE],
    [SQUARE_SIZE, 0],
    [0, 0],
])
# BEV 변환 시 투영될 목표 평면의 4점(좌하→우하→우상→좌상)

RESOLUTION = 0.05
# OccupancyGrid 해상도(1 셀 = 0.05m)

MAP_FRAME = 'odom'
# OccupancyGrid가 속할 좌표 프레임 ID
# =====================================================================

from std_msgs.msg import Int8
from sensor_msgs.msg import NavSatFix  # 픽셀 좌표를 NavSatFix로 받음

class YoloSegmentationNode(Node):
    def __init__(self):
        super().__init__('ros2_segmentation_node')

        # 입력
        self.pixel_sub = self.create_subscription(NavSatFix, '/scooter_pixel_bev', self.pixel_callback, 10)

        # 출력 퍼블리셔
        self.sidewalk_mask_pub = self.create_publisher(OccupancyGrid, '/sidewalk_mask_grid', 10)  # 인도 마스크(인도=0)
        self.occupancy_pub = self.create_publisher(OccupancyGrid, '/sidewalk_costmap', 10)        # 내비용 코스트맵
        self.result_pub = self.create_publisher(Int8, '/scooter_on_sidewalk', 10)

        self.latest_mask = None      # BEV 이진 마스크(워프 후, 원근보정 이미지 기준)
        self.prev_mask = None        # 폴백용 이전 마스크
        self.last_transform = None   # TF 캐시

        self.get_logger().info("YOLO Segmentation Node 시작")

        # 모델/브리지
        self.model = YOLO(MODEL_PATH, task="segment")
        self.device = 0 if torch.cuda.is_available() else 'cpu'
        self.bridge = CvBridge()

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 주기 제한
        self.last_yolo_time = self.get_clock().now()

        # 카메라 이미지 구독 (최신 프레임만)
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(
            Image, '/camera/d455_camera/color/image_raw', self.image_callback, qos_profile=image_qos
        )

        # 영상 저장 및 BEV 매트릭스
        self.video_writer = None
        self.bev_size = (SQUARE_SIZE, SQUARE_SIZE)
        self.M = cv2.getPerspectiveTransform(SRC_POINTS, DST_POINTS)

    # -------------------- 콜백 --------------------
    def image_callback(self, msg):
        # 최소 5Hz
        now = self.get_clock().now()
        if (now - self.last_yolo_time).nanoseconds < 200_000_000:
            return
        self.last_yolo_time = now

        # 이미지 변환
        try:
            raw_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge 변환 오류: {e}")
            return

        frame = cv2.undistort(raw_frame, camera_matrix, dist_coeffs)

        # 비디오 초기화
        if self.video_writer is None:
            now_str = datetime.now().strftime('%Y%m%d_%H%M%S')
            video_save_path = os.path.join(VIDEO_SAVE_DIR, f'bev_segmented_{now_str}.mp4')
            os.makedirs(VIDEO_SAVE_DIR, exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(video_save_path, fourcc, 20, self.bev_size)
            self.get_logger().info(f"저장 경로: {video_save_path}")

        # TF: odom<-base_link (안쓰지만 체크)
        try:
            _ = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.5))
        except LookupException as e:
            self.get_logger().warn(f"TF lookup 실패 (image_callback): {e}")
            # 계속 진행 (occupancy/mask 그리드는 camera_link TF를 별도 조회함)

        # YOLO 추론
        results = self.model.predict(
            frame, device=self.device, conf=CONF_THRESHOLD, imgsz=IMG_SIZE,
            classes=[SIDEWALK_CLASS_ID], stream=False
        )
        if not results:
            self.get_logger().warn("YOLO 추론 결과 없음")
            return

        result = results[0]
        annotated_bev = cv2.warpPerspective(frame, self.M, self.bev_size, flags=cv2.INTER_CUBIC)

        has_detection = result.masks is not None and len(result.masks.data) > 0

        if has_detection:
            # 여러 인도 마스크 합치기
            masks_np = result.masks.data.cpu().numpy()
            binary_mask_total = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)
            for mask in masks_np:
                mask = (mask * 255).astype(np.uint8)
                mask_resized = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
                binary_mask_total = cv2.bitwise_or(binary_mask_total, mask_resized)

            # 시각화(원본)
            overlay_rgb = np.zeros_like(frame)
            overlay_rgb[binary_mask_total == 255] = MASK_COLOR
            cv2.imshow("Original Segmentation", cv2.addWeighted(frame, 1 - ALPHA, overlay_rgb, ALPHA, 0))

            # BEV로 워프 후 이진화
            warped_mask = cv2.warpPerspective(binary_mask_total, self.M, self.bev_size, flags=cv2.INTER_CUBIC)
            _, binary_mask = cv2.threshold(warped_mask, 127, 255, cv2.THRESH_BINARY)

            # 소노이즈 제거(모폴로지)
            binary_mask = self._postprocess_mask(binary_mask)

            # 최신 마스크 갱신
            self.latest_mask = binary_mask
            self.prev_mask = binary_mask.copy()

            # BEV 시각화
            overlay = np.zeros_like(annotated_bev)
            overlay[binary_mask == 255] = MASK_COLOR
            annotated_bev = cv2.addWeighted(annotated_bev, 1 - ALPHA, overlay, ALPHA, 0)
            contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(annotated_bev, contours, -1, CONTOUR_COLOR, 2)

            # 퍼블리시: 내비용 코스트맵(전방 1.25m free) + 인도 마스크 그리드
            occ_grid = self.binary_mask_to_occupancygrid(binary_mask, msg.header)
            mask_grid = self.binary_mask_to_maskgrid(binary_mask, msg.header)
        else:
            if self.prev_mask is not None:
                self.get_logger().warn("인도 인식 실패 → 최근 마스크 재사용")
                occ_grid = self.binary_mask_to_occupancygrid(self.prev_mask, msg.header)
                mask_grid = self.binary_mask_to_maskgrid(self.prev_mask, msg.header)
                self.latest_mask = self.prev_mask
            else:
                self.get_logger().warn("인식된 인도 없음, 전체를 장애물로 처리")
                occ_grid = self.full_obstacle_occupancygrid(msg.header)
                mask_grid = self.full_obstacle_maskgrid(msg.header)

        # 퍼블리시
        if occ_grid is not None:
            self.occupancy_pub.publish(occ_grid)
        if mask_grid is not None:
            self.sidewalk_mask_pub.publish(mask_grid)

        # 디스플레이/저장
        cv2.imshow("Bird Eye View Segmentation", annotated_bev)
        cv2.waitKey(1)
        self.video_writer.write(annotated_bev)

    # -------------------- 유틸/헬퍼 --------------------
    def _lookup_cam_in_odom(self):
        """odom <- camera_link TF 조회(실패 시 캐시 사용)"""
        try:
            transform = self.tf_buffer.lookup_transform('odom', 'camera_link', rclpy.time.Time(), timeout=Duration(seconds=0.5))
            self.last_transform = transform
        except LookupException:
            transform = self.last_transform
        return transform

    def _postprocess_mask(self, mask):
        """마스크 후처리(작은 잡음 제거, 경계 매끈함)"""
        # 바이너리 보장
        mask = (mask > 127).astype(np.uint8) * 255
        # 작은 구멍 메우기/돌기 제거
        kernel3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        kernel5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel5, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel3, iterations=1)
        return mask

    def _common_grid_pose(self, frame_header, real_width_m, real_height_m, yaw, base_x, base_y):
        """공통 OccupancyGrid 헤더/지오메트리 구성"""
        grid = OccupancyGrid()
        grid.header.stamp = frame_header.stamp
        grid.header.frame_id = 'odom'
        grid.info.resolution = 0.05
        grid.info.width = int(real_width_m / 0.05)
        grid.info.height = int(real_height_m / 0.05)

        # camera_link 기준 1.775m 전방이 그리드 중심(실측/튜닝 값)
        center_offset = 1.775
        cx = base_x + center_offset * np.cos(yaw)
        cy = base_y + center_offset * np.sin(yaw)
        dx = -(real_width_m / 2.0); dy = -(real_height_m / 2.0)
        origin_x = cx + dx * np.cos(yaw) - dy * np.sin(yaw)
        origin_y = cy + dx * np.sin(yaw) + dy * np.cos(yaw)

        grid.info.origin.position.x = float(origin_x)
        grid.info.origin.position.y = float(origin_y)
        q = quaternion_from_euler(0, 0, yaw)
        grid.info.origin.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return grid

    # -------------------- 그리드 생성 --------------------
    def binary_mask_to_occupancygrid(self, binary_mask, frame_header):
        """전방 3.55m × 폭 2.3m, 전방 1.25m는 항상 free, 인도 구간 free."""
        real_width_m = 3.55      # 전방
        real_height_m = 2.3      # 좌우 폭
        res = 0.05

        grid_width = int(real_width_m / res)       # 71
        grid_height = int(real_height_m / res)     # 46
        visible_mask_width = int(2.3 / res)        # 46 (인도 폭)
        # free_cols = grid_width - visible_mask_width  # 25 (1.25m)

        # TF (camera_link)
        transform = self._lookup_cam_in_odom()
        if transform is None:
            return self.full_obstacle_occupancygrid(frame_header)

        base_x = transform.transform.translation.x
        base_y = transform.transform.translation.y
        quat = transform.transform.rotation
        _, _, yaw = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        # BEV 마스크 → 좌우반전 + 시계90 회전 → 그리드 크기에 맞춤
        flipped = cv2.flip(binary_mask, 1)
        rotated = cv2.rotate(flipped, cv2.ROTATE_90_CLOCKWISE)
        visible_mask = cv2.resize(rotated, (visible_mask_width, grid_height), interpolation=cv2.INTER_NEAREST)

        # 코스트맵 데이터(0 free, 100 obstacle)
        occupancy = np.full((grid_height, grid_width), 100, dtype=np.int8)
        occupancy[:, -visible_mask_width:][visible_mask == 255] = 0   # 인도 인식 구간 free
        occupancy[:, :-visible_mask_width] = 0                        # 전방 1.25m 항상 free

        grid = self._common_grid_pose(frame_header, real_width_m, real_height_m, yaw, base_x, base_y)
        grid.data = occupancy.flatten().tolist()
        return grid

    def binary_mask_to_maskgrid(self, binary_mask, frame_header):
        """인도 마스크 그리드(인도=0, 비인도=100). 전방 1.25m free 적용 없음."""
        real_width_m = 3.55
        real_height_m = 2.3
        res = 0.05
        grid_width = int(real_width_m / res)
        grid_height = int(real_height_m / res)
        visible_mask_width = int(2.3 / res)

        transform = self._lookup_cam_in_odom()
        if transform is None:
            return self.full_obstacle_maskgrid(frame_header)

        base_x = transform.transform.translation.x
        base_y = transform.transform.translation.y
        quat = transform.transform.rotation
        _, _, yaw = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        flipped = cv2.flip(binary_mask, 1)
        rotated = cv2.rotate(flipped, cv2.ROTATE_90_CLOCKWISE)
        visible_mask = cv2.resize(rotated, (visible_mask_width, grid_height), interpolation=cv2.INTER_NEAREST)

        mask_arr = np.full((grid_height, grid_width), 100, dtype=np.int8)
        mask_arr[:, -visible_mask_width:][visible_mask == 255] = 0  # 인도 영역만 0

        grid = self._common_grid_pose(frame_header, real_width_m, real_height_m, yaw, base_x, base_y)
        grid.data = mask_arr.flatten().tolist()
        return grid

    def full_obstacle_occupancygrid(self, frame_header):
        """TF 불가/마스크 없음 시: 전체 장애물(100)."""
        real_width_m = 3.55
        real_height_m = 2.3
        res = 0.05
        grid_width = int(real_width_m / res)
        grid_height = int(real_height_m / res)
        occupancy = np.full((grid_height, grid_width), 100, dtype=np.int8)

        transform = self._lookup_cam_in_odom()
        if transform is None:
            return None

        base_x = transform.transform.translation.x
        base_y = transform.transform.translation.y
        quat = transform.transform.rotation
        _, _, yaw = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        grid = self._common_grid_pose(frame_header, real_width_m, real_height_m, yaw, base_x, base_y)
        grid.data = occupancy.flatten().tolist()
        return grid

    def full_obstacle_maskgrid(self, frame_header):
        """TF 불가/마스크 없음 시: 인도 미확정(=모두 비인도: 100)."""
        real_width_m = 3.55
        real_height_m = 2.3
        res = 0.05
        grid_width = int(real_width_m / res)
        grid_height = int(real_height_m / res)
        mask_arr = np.full((grid_height, grid_width), 100, dtype=np.int8)

        transform = self._lookup_cam_in_odom()
        if transform is None:
            return None

        base_x = transform.transform.translation.x
        base_y = transform.transform.translation.y
        quat = transform.transform.rotation
        _, _, yaw = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        grid = self._common_grid_pose(frame_header, real_width_m, real_height_m, yaw, base_x, base_y)
        grid.data = mask_arr.flatten().tolist()
        return grid

    # -------------------- 기타 --------------------
    def pixel_callback(self, msg: NavSatFix):
        """픽셀 좌표로 인도 여부 질의(BEV 마스크 기준)."""
        if self.latest_mask is None:
            return
        u = int(msg.longitude)
        v = int(msg.latitude)
        if 0 <= v < self.latest_mask.shape[0] and 0 <= u < self.latest_mask.shape[1]:
            is_on_sidewalk = int(self.latest_mask[v, u] == 255)
        else:
            is_on_sidewalk = 0
        result_msg = Int8()
        result_msg.data = is_on_sidewalk
        self.result_pub.publish(result_msg)

    def destroy_node(self):
        if self.video_writer:
            try:
                self.video_writer.release()
            except Exception:
                pass
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloSegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
