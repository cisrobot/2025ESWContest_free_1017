#!/usr/bin/env python3
import os
import cv2
import numpy as np
from datetime import datetime
import tf_transformations  # 추가
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
from rclpy.duration import Duration  # ✅ FIX: import Duration

VIDEO_SAVE_DIR = '/media/marin/4cca4ad9-422b-4ad3-b582-3f9c402dd434/home/omo/videos/seg'
MODEL_PATH = '/home/marin/marine/src/yolo_segmentation/models/best.pt'
CONF_THRESHOLD = 0.300
IMG_SIZE = 640
SIDEWALK_CLASS_ID = 0
ALPHA = 0.5
MASK_COLOR = (0, 0, 255)
CONTOUR_COLOR = (0, 255, 0)

camera_matrix = np.array([
    [910.0, 0.0, 640.0],
    [0.0, 910.0, 360.0],
    [0.0, 0.0, 1.0]
], dtype=np.float32)

dist_coeffs = np.array([-0.04, 0.02, 0.0, 0.0, 0.0], dtype=np.float32)


#기존 사용
# SRC_POINTS = np.float32([
#     [88, 503],
#     [1192, 503],
#     [932, 443],
#     [341, 443],
# ])

#230x440
# SRC_POINTS = np.float32([
#         [6, 563],    # point 0
#         [1278, 563], # point 1
#         [770, 391],  # point 2
#         [495, 391],  # point 3
# ])
#230X230
SRC_POINTS = np.float32([
    [3, 556],    # point 0
    [1279, 566], # point 1    
    [843, 419],  # point 2
    [421, 419],  # point 3
])

SQUARE_SIZE = 720
DST_POINTS = np.float32([
    [0, SQUARE_SIZE],
    [SQUARE_SIZE, SQUARE_SIZE],
    [SQUARE_SIZE, 0],
    [0, 0],
])

RESOLUTION = 0.05
MAP_FRAME = 'odom'

from std_msgs.msg import Int8
from sensor_msgs.msg import NavSatFix  # 픽셀 좌표를 NavSatFix로 받음

class YoloSegmentationNode(Node):
    def __init__(self):
        super().__init__('ros2_segmentation_node')
        self.pixel_sub = self.create_subscription(
            NavSatFix,
            '/scooter_pixel_bev',
            self.pixel_callback,
            10
        )
        # (상단) 퍼블리셔 추가
        self.sidewalk_mask_pub = self.create_publisher(
            OccupancyGrid, '/sidewalk_mask_grid', 10
        )

        self.result_pub = self.create_publisher(Int8, '/scooter_on_sidewalk', 10)
        self.latest_mask = None
        self.get_logger().info("YOLO Segmentation Node 시작")

        self.model = YOLO(MODEL_PATH, task="segment")
        self.device = 0 if torch.cuda.is_available() else 'cpu'
        self.bridge = CvBridge()
        # TF2 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_success_stamp = None
        self.prev_mask = None
        # OccupancyGrid 누적을 위한 변수
        self.grid_resolution = 0.05
        self.grid_width = int(10.0 / self.grid_resolution)  # 10m x 10m 영역
        self.grid_height = int(10.0 / self.grid_resolution)
        self.last_transform = None  # TF 캐싱
        self.last_yolo_time = self.get_clock().now()  # ← YOLO 추론 주기 제한을 위한 초기값


        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1  # 꼭 1로 설정: 최신 프레임만 받아 처리
        )

        self.subscription = self.create_subscription(
            Image,
            '/camera/d455_camera/color/image_raw',
            self.image_callback,
            qos_profile=image_qos
        )

        self.video_writer = None
        self.bev_size = (SQUARE_SIZE, SQUARE_SIZE)
        self.M = cv2.getPerspectiveTransform(SRC_POINTS, DST_POINTS)

        self.occupancy_pub = self.create_publisher(
            OccupancyGrid,
            '/sidewalk_costmap',
            10
        )


    def initialize_global_costmap(self, header, robot_x, robot_y):
        grid = OccupancyGrid()
        grid.header = header
        grid.header.frame_id = 'odom'
        grid.info.resolution = self.grid_resolution
        grid.info.width = self.grid_width
        grid.info.height = self.grid_height

        # 로봇이 중심에 오도록 origin 설정
        grid.info.origin.position.x = robot_x - (self.grid_width * self.grid_resolution) / 2.0
        grid.info.origin.position.y = robot_y - (self.grid_height * self.grid_resolution) / 2.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (self.grid_width * self.grid_height)
        return grid

    def image_callback(self, msg):
        now = self.get_clock().now()
        if (now - self.last_yolo_time).nanoseconds < 200_000_000:  # 최소 5Hz
            return
        self.last_yolo_time = now

        try:
            raw_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge 변환 오류: {e}")
            return

        frame = cv2.undistort(raw_frame, camera_matrix, dist_coeffs)

        if self.video_writer is None:
            now_str = datetime.now().strftime('%Y%m%d_%H%M%S')
            video_save_path = os.path.join(VIDEO_SAVE_DIR, f'bev_segmented_{now_str}.mp4')
            os.makedirs(VIDEO_SAVE_DIR, exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(video_save_path, fourcc, 20, self.bev_size)
            self.get_logger().info(f"저장 경로: {video_save_path}")

        try:
            transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.5))  # ✅ FIX: Duration
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
        except LookupException as e:
            self.get_logger().warn(f"TF lookup 실패 (image_callback): {e}")
            return

        results = self.model.predict(
            frame,
            device=self.device,
            conf=CONF_THRESHOLD,
            imgsz=IMG_SIZE,
            classes=[SIDEWALK_CLASS_ID],
            stream=False  # 리스트 반환
        )

        if not results:
            self.get_logger().warn("YOLO 추론 결과 없음")
            return

        result = results[0]
        annotated_bev = cv2.warpPerspective(frame, self.M, self.bev_size, flags=cv2.INTER_CUBIC)

        has_detection = result.masks is not None and len(result.masks.data) > 0

        if has_detection:
            masks_np = result.masks.data.cpu().numpy()
            binary_mask_total = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)

            for mask in masks_np:
                mask = (mask * 255).astype(np.uint8)
                mask_resized = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
                binary_mask_total = cv2.bitwise_or(binary_mask_total, mask_resized)

            # ✅ 원본 이미지 위에 마스크 시각화 추가
            overlay_rgb = np.zeros_like(frame)
            overlay_rgb[binary_mask_total == 255] = MASK_COLOR
            original_seg = cv2.addWeighted(frame, 1 - ALPHA, overlay_rgb, ALPHA, 0)
            cv2.imshow("Original Segmentation", original_seg)

            warped_mask = cv2.warpPerspective(binary_mask_total, self.M, self.bev_size, flags=cv2.INTER_CUBIC)
            _, binary_mask = cv2.threshold(warped_mask, 127, 255, cv2.THRESH_BINARY)

            # ✅ FIX: latest_mask 갱신 (핵심 오류 수정)
            self.latest_mask = binary_mask

            overlay = np.zeros_like(annotated_bev)
            overlay[binary_mask == 255] = MASK_COLOR
            annotated_bev = cv2.addWeighted(annotated_bev, 1 - ALPHA, overlay, ALPHA, 0)

            contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(annotated_bev, contours, -1, CONTOUR_COLOR, 2)

            self.prev_mask = binary_mask
            occupancy_grid = self.binary_mask_to_occupancygrid(binary_mask, msg.header)
        else:
            if self.prev_mask is not None:
                self.get_logger().warn("인도 인식 실패 → 최근 마스크 재사용")
                occupancy_grid = self.binary_mask_to_occupancygrid(self.prev_mask, msg.header)
                # ✅ FIX: prev 사용 시에도 latest_mask 동기화
                self.latest_mask = self.prev_mask
            else:
                self.get_logger().warn("인식된 인도 없음, 전체를 장애물로 처리")
                occupancy_grid = self.full_obstacle_occupancygrid(msg.header)

        self.occupancy_pub.publish(occupancy_grid)

        cv2.imshow("Bird Eye View Segmentation", annotated_bev)
        cv2.waitKey(1)
        self.video_writer.write(annotated_bev)



    def binary_mask_to_occupancygrid(self, binary_mask, frame_header):
        # === 1. 실제 BEV 영역 정의 ===
        real_width_m = 3.55      # ← 로봇 전방 방향
        real_height_m = 2.3      # ← 인도 폭 (좌우)
        occupancy_resolution = 0.05

        grid_width = int(real_width_m / occupancy_resolution)         # 71 ← 전방 3.55m
        grid_height = int(real_height_m / occupancy_resolution)       # 46 ← 인도 폭 2.3m
        visible_mask_width = int(2.3 / occupancy_resolution)          # YOLO 마스크 유효 영역: 2.3m
        free_cols = grid_width - visible_mask_width                   # 1.25m → 약 25칸

        # === 2. TF: odom → camera_link 변환 ===
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'camera_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)  # ✅ FIX: Duration
            )
            self.last_transform = transform
        except LookupException as e:
            if self.last_transform:
                self.get_logger().warn(f"TF lookup 실패 → 이전 transform 재사용: {e}")
                transform = self.last_transform
            else:
                self.get_logger().warn(f"TF lookup 실패: {e}")
                return self.full_obstacle_occupancygrid(frame_header)

        base_x = transform.transform.translation.x
        base_y = transform.transform.translation.y
        rotation = transform.transform.rotation
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(quat)

        # === 3. 마스크 보정: 좌우 반전 + 시계 방향 90도 회전 ===
        flipped_mask = cv2.flip(binary_mask, 1)  # 좌우 반전
        rotated_mask = cv2.rotate(flipped_mask, cv2.ROTATE_90_CLOCKWISE)  # 시계 방향 90도 회전

        # === 4. YOLO 마스크를 가로 2.3m (width=46)로 맞춤
        visible_mask = cv2.resize(rotated_mask, (visible_mask_width, grid_height), interpolation=cv2.INTER_NEAREST)

        # === 5. OccupancyGrid 데이터 구성 ===
        occupancy = np.full((grid_height, grid_width), 100, dtype=np.int8)
        occupancy[:, -visible_mask_width:][visible_mask == 255] = 0   # YOLO 인식 결과는 뒤쪽에 배치
        occupancy[:, :-visible_mask_width] = 0                        # 앞쪽 1.25m는 무조건 free
        # === 6. OccupancyGrid 메시지 생성 ===
        grid = OccupancyGrid()
        grid.header.stamp = frame_header.stamp
        grid.header.frame_id = 'odom'
        grid.info.resolution = occupancy_resolution
        grid.info.width = grid_width
        grid.info.height = grid_height

        # === 7. Origin 설정 (camera_link 기준 2.4m 전방이 중심)
        center_offset = 1.775
        cx = base_x + center_offset * np.cos(yaw)
        cy = base_y + center_offset * np.sin(yaw)

        dx = -(real_width_m / 2)
        dy = -(real_height_m / 2)

        origin_x = cx + dx * np.cos(yaw) - dy * np.sin(yaw)
        origin_y = cy + dx * np.sin(yaw) + dy * np.cos(yaw)

        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y

        q = quaternion_from_euler(0, 0, yaw)
        grid.info.origin.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        grid.data = occupancy.flatten().tolist()
        return grid





    def full_obstacle_occupancygrid(self, frame_header):
        real_width_m = 3.55
        real_height_m = 2.3
        occupancy_resolution = 0.05

        grid_width = int(real_width_m / occupancy_resolution)
        grid_height = int(real_height_m / occupancy_resolution)

        occupancy = np.full((grid_height, grid_width), 100, dtype=np.int8)

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'odom'
        grid.info.resolution = occupancy_resolution
        grid.info.width = grid_width
        grid.info.height = grid_height

        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'camera_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)  # ✅ FIX: Duration
            )
            self.last_transform = transform
        except LookupException as e:
            if self.last_transform:
                self.get_logger().warn(f"TF lookup 실패 → 이전 transform 재사용: {e}")
                transform = self.last_transform
            else:
                self.get_logger().warn(f"TF lookup 실패: {e}")
                return None  # 재귀 호출 제거

        base_x = transform.transform.translation.x
        base_y = transform.transform.translation.y
        rotation = transform.transform.rotation
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(quat)

        # OccupancyGrid 중심 기준 좌표 계산 (camera_link에서 전방 2.4m)
        center_offset = 1.775
        cx = base_x + center_offset * np.cos(yaw)
        cy = base_y + center_offset * np.sin(yaw)

        # grid origin 좌표 계산 (왼쪽 아래 꼭짓점)
        dx = -(real_width_m / 2)
        dy = -(real_height_m / 2)

        origin_x = cx + dx * np.cos(yaw) - dy * np.sin(yaw)
        origin_y = cy + dx * np.sin(yaw) + dy * np.cos(yaw)

        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y

        q = quaternion_from_euler(0, 0, yaw)
        grid.info.origin.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        grid.data = occupancy.flatten().tolist()
        return grid

    

    
    def pixel_callback(self, msg: NavSatFix):
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
    
    # ✅ FIX: destroy_node를 클래스 메서드로 정의(자원 해제)
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
