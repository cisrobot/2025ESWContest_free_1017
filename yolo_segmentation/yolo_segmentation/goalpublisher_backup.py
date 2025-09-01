#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import copy
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Path


def quat_to_yaw(q):
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


class AutoGoalPublisher(Node):
    """
    local_costmap 기반으로
    - base_link 기준 반지름 R의 반원(기본 ±90°) 위 샘플을 생성
    - 샘플 지점의 점유(장애물) 여부를 확인
    - 좌/우 접점(장애물 구간) 각도를 찾고 가운데 각도 = goal 각도로 결정
    - (예외1) 접점 1개면 폭 W 가정해 가운데 각도 추정
    - (예외2) 접점 0개면 정면(0rad)
    - goal은 costmap의 frame_id(보통 odom)에 보냄
    """

    def __init__(self):
        super().__init__('auto_goal_publisher')

        # ---------------- Parameters ----------------
        self.declare_parameter('costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('global_frame', '')  # 비워두면 costmap.header.frame_id 사용
        self.declare_parameter('goal_radius', 2.9)  # m (복도 실험 및 인도 공통)
        self.declare_parameter('assumed_sidewalk_width', 3.65)  # m (접점 1개일 때 사용)
        self.declare_parameter('angle_min_deg', -90.0)
        self.declare_parameter('angle_max_deg', 90.0)
        self.declare_parameter('angle_step_deg', 2.0)  # 촘촘할수록 접점 검출 안정
        self.declare_parameter('update_period', 0.5)   # s
        self.declare_parameter('occupancy_threshold', 65)  # 코스트맵 0~100, 이 이상이면 장애물
        self.declare_parameter('unknown_is_occupied', False)
        self.declare_parameter('min_goal_move', 0.30)  # m, 이만큼 이상 이동/회전 시에만 새 goal 전송
        self.declare_parameter('yaw_from_center_angle', True)  # True면 로봇 yaw+중심각으로 바라봄
        self.declare_parameter('global_costmap_topic', '/global_costmap/costmap')
        self.global_costmap_topic = self.get_parameter('global_costmap_topic').get_parameter_value().string_value
        self.plan_end_marker_pub = self.create_publisher(Marker, '/plan_end_marker', 1)
        self.plan_sub = self.create_subscription(Path, '/plan', self.plan_cb, 10)


        self.costmap_topic = self.get_parameter('costmap_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.global_frame_param = self.get_parameter('global_frame').get_parameter_value().string_value
        self.R = float(self.get_parameter('goal_radius').value)
        self.W = float(self.get_parameter('assumed_sidewalk_width').value)
        self.ang_min = math.radians(float(self.get_parameter('angle_min_deg').value))
        self.ang_max = math.radians(float(self.get_parameter('angle_max_deg').value))
        self.ang_step = math.radians(float(self.get_parameter('angle_step_deg').value))
        self.period = float(self.get_parameter('update_period').value)
        self.occ_th = int(self.get_parameter('occupancy_threshold').value)
        self.unknown_occ = bool(self.get_parameter('unknown_is_occupied').value)
        self.min_goal_move = float(self.get_parameter('min_goal_move').value)
        self.yaw_from_center_angle = bool(self.get_parameter('yaw_from_center_angle').value)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, self.costmap_topic, self.costmap_cb, qos
        )

        # Action Client (NavigateToPose)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers (Markers)
        self.marker_pub = self.create_publisher(MarkerArray, '/auto_goal_markers', 10)
        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid, self.global_costmap_topic, self.global_costmap_cb, qos
        )

        # State
        self.latest_costmap = None
        self.global_frame = None
        self.last_goal_xyyaw = None  # (x,y,yaw)
        self.latest_global_costmap = None

        # Timer
        self.timer = self.create_timer(self.period, self.on_timer)

        self.get_logger().info(
            f'auto_goal_publisher: topic={self.costmap_topic}, R={self.R:.2f}m, W={self.W:.2f}m, '
            f'angles=[{math.degrees(self.ang_min):.1f},{math.degrees(self.ang_max):.1f}] step={math.degrees(self.ang_step):.1f}°'
        )

    # ---------------- Callbacks ----------------


    def plan_cb(self, msg: Path):
        if not msg.poses:
            return
        p = msg.poses[-1].pose.position
        m = Marker()
        m.header.frame_id = self.global_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'plan_end'
        m.id = 5000
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = float(p.x)
        m.pose.position.y = float(p.y)
        m.pose.position.z = 0.05
        m.scale.x = 0.18
        m.scale.y = 0.18
        m.scale.z = 0.18
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0  # 초록
        self.plan_end_marker_pub.publish(m)

    def costmap_cb(self, msg: OccupancyGrid):
        self.latest_costmap = msg
        if not self.global_frame:
            self.global_frame = self.global_frame_param if self.global_frame_param else msg.header.frame_id

    def on_timer(self):
        if self.latest_costmap is None or self.global_frame is None:
            return
        if not self.nav_client.server_is_ready():
            # 서버 준비 전엔 아무 것도 안 보냄
            return

        # base_link pose in global_frame (costmap frame)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed {self.global_frame}->{self.base_frame}: {ex}")
            return

        bx = tf.transform.translation.x
        by = tf.transform.translation.y
        yaw = quat_to_yaw(tf.transform.rotation)

        # 샘플링
        angles, world_pts, occupied = self.sample_circle(bx, by, yaw)

        # 가장 넓은 자유공간 run의 중앙 각도 사용
        center_ang, run_a, run_b = self.pick_center_angle_from_free_runs(angles, occupied)
        left_idx, right_idx = None, None  # 더 이상 사용하지 않음





        # goal 후보 계산 (원 위 지점)
        gx = bx + self.R * math.cos(yaw + center_ang)
        gy = by + self.R * math.sin(yaw + center_ang)

        # goal 셀 유효성 확인 (장애물/미지인 경우 반경을 줄여가며 보정)
        gx, gy = self.backoff_if_occupied(bx, by, yaw, center_ang, gx, gy)

        # goal yaw: 로봇 yaw + 중심각(진행방향) 또는 로봇 yaw
        gyaw = yaw + center_ang if self.yaw_from_center_angle else yaw

        # 변화량이 충분할 때만 전송 (너무 잦은 프리엠프 방지)
        if not self.should_send((gx, gy, gyaw)):
            self.publish_markers(
                bx, by, yaw,
                angles, world_pts, occupied,
                left_idx, right_idx,
                (gx, gy),
                free_run=(run_a, run_b)
            )
            return

        # PoseStamped 전송
        goal = PoseStamped()
        goal.header.frame_id = self.global_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(gx)
        goal.pose.position.y = float(gy)
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(gyaw * 0.5)
        goal.pose.orientation.w = math.cos(gyaw * 0.5)

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal

        # 새 goal 보내면 기존 goal은 자동 preempt됨 (BT + Nav2)
        self.nav_client.send_goal_async(nav_goal)
        self.last_goal_xyyaw = (gx, gy, gyaw)

        # 마커 갱신
        self.publish_markers(
            bx, by, yaw,
            angles, world_pts, occupied,
            left_idx, right_idx,
            (gx, gy),
            free_run=(run_a, run_b)
        )

    # ---------------- Geometry / Logic ----------------
    def is_occupied_in(self, costmap: OccupancyGrid, x, y, unknown_is_occ: bool):
        res = costmap.info.resolution
        ox = costmap.info.origin.position.x
        oy = costmap.info.origin.position.y
        w = costmap.info.width
        h = costmap.info.height
        mx = int((x - ox) / res); my = int((y - oy) / res)
        if 0 <= mx < w and 0 <= my < h:
            v = costmap.data[my * w + mx]
            if v < 0:
                return unknown_is_occ
            return v >= self.occ_th
        return True
    def global_costmap_cb(self, msg: OccupancyGrid):
        self.latest_global_costmap = msg
    
    def is_occupied_global(self, x, y):
        if self.latest_global_costmap is None:
            return False  # 아직 글로벌 없음 → 로컬만으로 판단
        # 글로벌은 unknown도 위험 취급 권장
        return self.is_occupied_in(self.latest_global_costmap, x, y, unknown_is_occ=True)


    def pick_center_angle_from_free_runs(self, angles, occupied):
        """
        호 위에서 연속된 자유공간(run)들을 찾고,
        - 양쪽 경계가 'occupied'인 '진짜 갭'을 우선
        - 갭의 각폭이 기대폭(인도/복도 폭 W에 해당)과 가장 가까운 run 선택
        - 후보가 없으면 가장 넓은 run으로 폴백
        반환: (중앙각, run 시작 idx, run 끝 idx)
        """
        # 1) run 찾기 (free=True)
        free = [not o for o in occupied]
        runs = []
        s = None
        for i, f in enumerate(free):
            if f and s is None:
                s = i
            elif (not f) and s is not None:
                runs.append((s, i - 1)); s = None
        if s is not None:
            runs.append((s, len(free) - 1))

        if not runs:
            return 0.0, None, None

        # 2) 기대 각폭 (W @ 반경 R)
        ratio = min(0.999, self.W / (2.0 * self.R))
        expected_width = 2.0 * math.asin(ratio)  # rad

        def is_occ(idx):
            return 0 <= idx < len(occupied) and occupied[idx]

        # 3) 각 run 점수화: 폭이 expected_width에 가까울수록 ↑,
        #    양끝이 실제 '벽'이면 가점, 끝단(FOV 경계) run은 감점
        scored = []
        for a, b in runs:
            width = float(angles[b] - angles[a])
            left_wall  = is_occ(a - 1)
            right_wall = is_occ(b + 1)
            proper_gap = left_wall and right_wall
            edge_touched = (a == 0) or (b == len(angles) - 1)

            # 기본 점수 = -|width - expected|
            score = -abs(width - expected_width)
            if proper_gap:
                score += 0.5       # 양쪽이 벽이면 가점
            if edge_touched:
                score -= 0.5       # 시야 끝에 붙은 run은 감점

            scored.append((score, a, b, width, proper_gap, edge_touched))

        # 4) 최고 점 run 선택
        scored.sort(key=lambda x: x[0], reverse=True)
        best = scored[0]
        a, b = best[1], best[2]
        mid_idx = (a + b) // 2

        # 디버그 로그(원하면 info로)
        self.get_logger().debug(
            f"[gap] chosen width={best[3]*180/math.pi:.1f}deg "
            f"(expected={expected_width*180/math.pi:.1f}), "
            f"proper_gap={best[4]}, edge_touched={best[5]}"
        )

        return angles[mid_idx], a, b



    def sample_circle(self, bx, by, yaw):
        """
        base_link 기준 로컬 각도 ang ∈ [ang_min, ang_max]에서
        반지름 R의 점을 글로벌 좌표로 변환 후 점유 여부 확인
        """
        angles = np.arange(self.ang_min, self.ang_max + 1e-6, self.ang_step)
        world_pts = []
        occupied = []

        for a in angles:
            # base_link frame에서의 점
            dx = self.R * math.cos(a)
            dy = self.R * math.sin(a)
            # global frame으로 회전/이동
            wx = bx + (math.cos(yaw) * dx - math.sin(yaw) * dy)
            wy = by + (math.sin(yaw) * dx + math.cos(yaw) * dy)
            world_pts.append((wx, wy))
            occupied.append(self.is_occupied(wx, wy))

        return angles, world_pts, occupied

    def is_occupied(self, x, y):
        m = self.latest_costmap
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        w = m.info.width
        h = m.info.height

        mx = int((x - ox) / res)
        my = int((y - oy) / res)
        if 0 <= mx < w and 0 <= my < h:
            v = m.data[my * w + mx]
            if v < 0:
                return self.unknown_occ
            return v >= self.occ_th
        # 맵 밖은 위험 취급
        return True

    def pick_edge_indices(self, angles, occupied):
        """
        좌측(+각도)과 우측(-각도)에서 '장애물'이 검출된 가장 바깥쪽(절대값 큰 각도)의 인덱스 선택
        복도/인도에서는 벽/가장자리가 반원 바깥쪽에 연속적으로 검출되는 경향이 있어 이 방식이 안정적
        """
        left_idxs = [i for i, (a, occ) in enumerate(zip(angles, occupied)) if a > 0 and occ]
        right_idxs = [i for i, (a, occ) in enumerate(zip(angles, occupied)) if a < 0 and occ]

        left_idx = max(left_idxs, key=lambda i: angles[i]) if left_idxs else None
        right_idx = min(right_idxs, key=lambda i: angles[i]) if right_idxs else None
        return left_idx, right_idx

    def decide_center_angle(self, angles, left_idx, right_idx):
        """
        - 둘 다 있으면: 가운데 각도
        - 하나만 있으면: 폭 W 가정 → Δθ = 2*asin(W/(2R)), 가운데 각도 = edge ± Δθ/2
        - 없으면: 0
        """
        if left_idx is not None and right_idx is not None:
            return 0.5 * (angles[left_idx] + angles[right_idx])

        # one edge case
        if self.W > 0.0 and self.R > 0.0:
            ratio = min(0.999, self.W / (2.0 * self.R))
            dtheta = 2.0 * math.asin(ratio)
        else:
            dtheta = math.radians(40.0)  # 안전한 기본값

        if left_idx is not None:
            # 왼쪽 벽만 보이면 가운데는 그보다 오른쪽
            return angles[left_idx] - 0.5 * dtheta
        if right_idx is not None:
            # 오른쪽 벽만 보이면 가운데는 그보다 왼쪽
            return angles[right_idx] + 0.5 * dtheta

        # fallback: 정면
        return 0.0

    def backoff_if_occupied(self, bx, by, yaw, center_ang, gx, gy):
        def bad(xx, yy):
            # 로컬 OR 글로벌 중 하나라도 막히면 불가
            return self.is_occupied(xx, yy) or self.is_occupied_global(xx, yy)

        if not bad(gx, gy):
            return gx, gy

        r = self.R
        while r > 1.0:
            r -= 0.2
            tx = bx + r * math.cos(yaw + center_ang)
            ty = by + r * math.sin(yaw + center_ang)
            if not bad(tx, ty):
                return tx, ty
        return gx, gy

    def should_send(self, new_xyyaw):
        if self.last_goal_xyyaw is None:
            return True
        dx = new_xyyaw[0] - self.last_goal_xyyaw[0]
        dy = new_xyyaw[1] - self.last_goal_xyyaw[1]
        dyaw = self.norm_ang(new_xyyaw[2] - self.last_goal_xyyaw[2])
        if (dx * dx + dy * dy) ** 0.5 > self.min_goal_move:
            return True
        if abs(dyaw) > math.radians(10.0):
            return True
        return False

    @staticmethod
    def norm_ang(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    # ---------------- Visualization ----------------

    def publish_markers(self, bx, by, yaw, angles, world_pts, occupied,
                    left_idx, right_idx, goal_xy, free_run=None):
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        # 1) 샘플 포인트 (노랑=free, 빨강=occupied)
        for i, (wx, wy) in enumerate(world_pts):
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp = now
            m.ns = 'circ_samples'
            m.id = 1000 + i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(wx)
            m.pose.position.y = float(wy)
            m.pose.position.z = 0.02
            m.scale.x = 0.08
            m.scale.y = 0.08
            m.scale.z = 0.08
            if occupied[i]:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.9
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 0.9
            arr.markers.append(m)

        # 2) 좌/우 접점 마커 (초록)
        for tag, idx in (('left', left_idx), ('right', right_idx)):
            if idx is None:
                continue
            wx, wy = world_pts[idx]
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp = now
            m.ns = 'edges'
            m.id = 2000 + (0 if tag == 'left' else 1)
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(wx)
            m.pose.position.y = float(wy)
            m.pose.position.z = 0.04
            m.scale.x = 0.16
            m.scale.y = 0.16
            m.scale.z = 0.16
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.2, 0.9
            arr.markers.append(m)

        if free_run is not None and free_run[0] is not None and free_run[1] is not None:
            a, b = free_run
            for run_tag, run_idx, mid in (('run_start', a, 2100), ('run_end', b, 2101)):
                wx, wy = world_pts[run_idx]
                m = Marker()
                m.header.frame_id = self.global_frame
                m.header.stamp = now
                m.ns = 'free_run_edges'
                m.id = mid
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = float(wx)
                m.pose.position.y = float(wy)
                m.pose.position.z = 0.04
                m.scale.x = 0.16
                m.scale.y = 0.16
                m.scale.z = 0.16
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.2, 0.9
                arr.markers.append(m)

        # 3) goal 마커 (파랑) + base_link->goal 화살표
        gx, gy = goal_xy
        m_goal = Marker()
        m_goal.header.frame_id = self.global_frame
        m_goal.header.stamp = now
        m_goal.ns = 'goal'
        m_goal.id = 3000
        m_goal.type = Marker.SPHERE
        m_goal.action = Marker.ADD
        m_goal.pose.position.x = float(gx)
        m_goal.pose.position.y = float(gy)
        m_goal.pose.position.z = 0.05
        m_goal.scale.x = 0.22
        m_goal.scale.y = 0.22
        m_goal.scale.z = 0.22
        m_goal.color.r, m_goal.color.g, m_goal.color.b, m_goal.color.a = 0.2, 0.6, 1.0, 1.0
        arr.markers.append(m_goal)

        m_arrow = Marker()
        m_arrow.header.frame_id = self.global_frame
        m_arrow.header.stamp = now
        m_arrow.ns = 'goal_arrow'
        m_arrow.id = 4000
        m_arrow.type = Marker.ARROW
        m_arrow.action = Marker.ADD
        m_arrow.scale.x = 0.05  # shaft diameter
        m_arrow.scale.y = 0.1   # head diameter
        m_arrow.scale.z = 0.1   # head length
        m_arrow.color.r, m_arrow.color.g, m_arrow.color.b, m_arrow.color.a = 0.2, 0.6, 1.0, 1.0
        m_arrow.points = [
            Point(x=float(bx), y=float(by), z=0.05),
            Point(x=float(gx), y=float(gy), z=0.05),
        ]
        arr.markers.append(m_arrow)

        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = AutoGoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
