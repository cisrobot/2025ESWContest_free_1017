#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import euler_from_quaternion


def quat_to_yaw(q):
    """Quaternion → yaw(rad)로 변환."""
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


class AutoGoalPublisher(Node):
    """
    인도 중심 주행을 위한 자동 goal 생성 노드.

    핵심 아이디어:
    - 반원 샘플링 링의 중심을 camera_link에 두고, base_link의 yaw를 진행 방향 기준으로 사용.
    - '로컬 코스트맵의 free' AND '인도 마스크 내부'를 free로 간주.
    - 링 반경 R에서 내측(R-ε)은 인도, 외측(R+ε)은 비인도이면 해당 각도를 '경계'로 인식.
    - 두 경계가 있으면 중앙각, 한 쪽만 보이면 인도 폭 W로 중앙각 추정, 경계가 없으면 best free run 중앙각 선택.
    - 선정된 center angle을 기반으로 goal을 계산하되, 인도 밖/장애물일 경우 반경 백오프/미세각 탐색으로 스냅.
    - 중심각 IIR(α)과 최대 변화량 제한으로 각도 지터를 완화.
    """

    def __init__(self):
        """파라미터 선언/로드, 구독·퍼블리셔·액션클라이언트·타이머 초기화."""
        super().__init__('auto_goal_publisher')

        # ---------------- Parameters ----------------
        self.declare_parameter('costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('sidewalk_mask_topic', '/sidewalk_mask_grid')  # 인도=0, 비인도=100
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ring_center_frame', 'camera_link')  # 반원 중심 프레임 (요구: camera_link)
        self.declare_parameter('global_frame', '')  # 비우면 구독한 맵의 frame_id 사용
        self.declare_parameter('goal_radius', 2.5)  # 반원 반지름 R [m]
        self.declare_parameter('assumed_sidewalk_width', 3.65)  # 인도 폭 W [m] (한쪽 경계 시 사용)
        self.declare_parameter('angle_min_deg', -90.0)
        self.declare_parameter('angle_max_deg', 90.0)
        self.declare_parameter('angle_step_deg', 2.0)
        self.declare_parameter('update_period', 0.5)
        self.declare_parameter('occupancy_threshold', 65)
        self.declare_parameter('unknown_is_occupied', False)  # 로컬 코스트맵 unknown을 점유로 볼지
        self.declare_parameter('min_goal_move', 0.30)  # 이전 goal 대비 최소 이동거리
        self.declare_parameter('yaw_from_center_angle', True)
        self.declare_parameter('boundary_eps_m', 0.10)        # 경계 판별용 ε
        self.declare_parameter('global_costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('smooth_alpha', 0.6)           # IIR α
        self.declare_parameter('max_center_step_deg', 12.0)   # 한 주기 최대 각 변화 제한

        # 파라미터 로드
        self.costmap_topic = self.get_parameter('costmap_topic').get_parameter_value().string_value
        self.sidewalk_mask_topic = self.get_parameter('sidewalk_mask_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.center_frame = self.get_parameter('ring_center_frame').get_parameter_value().string_value
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
        self.boundary_eps = float(self.get_parameter('boundary_eps_m').value)
        self.global_costmap_topic = self.get_parameter('global_costmap_topic').get_parameter_value().string_value

        self.alpha = float(self.get_parameter('smooth_alpha').value)
        self.max_step = math.radians(float(self.get_parameter('max_center_step_deg').value))

        # TF 버퍼/리스너
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)  # ← 들여쓰기 보정

        # Subscribers
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.costmap_sub = self.create_subscription(OccupancyGrid, self.costmap_topic, self.costmap_cb, qos)
        self.mask_sub = self.create_subscription(OccupancyGrid, self.sidewalk_mask_topic, self.mask_cb, qos)
        self.global_costmap_sub = self.create_subscription(OccupancyGrid, self.global_costmap_topic, self.global_costmap_cb, qos)

        # Action Client (Nav2 navigate_to_pose)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers (Markers/Plan)
        self.marker_pub = self.create_publisher(MarkerArray, '/auto_goal_markers', 10)
        self.plan_end_marker_pub = self.create_publisher(Marker, '/plan_end_marker', 1)
        self.plan_sub = self.create_subscription(Path, '/plan', self.plan_cb, 10)

        # State
        self.latest_costmap = None
        self.latest_maskmap = None
        self.latest_global_costmap = None
        self.global_frame = None
        self.last_goal_xyyaw = None
        self.last_center_ang = None  # 지터 완화용

        # Timer
        self.timer = self.create_timer(self.period, self.on_timer)

        self.get_logger().info(
            f'auto_goal_publisher: local={self.costmap_topic}, mask={self.sidewalk_mask_topic}, '
            f'R={self.R:.2f}m, W={self.W:.2f}m, center_frame={self.center_frame}, '
            f'angles=[{math.degrees(self.ang_min):.1f},{math.degrees(self.ang_max):.1f}] '
            f'step={math.degrees(self.ang_step):.1f}°, eps={self.boundary_eps:.2f}m'
        )

    # ---------------- Callbacks ----------------
    def plan_cb(self, msg: Path):
        """현재 Nav2 계획 경로의 마지막 지점을 큐브 마커로 표시."""
        if not msg.poses or not self.global_frame:
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
        m.scale.x = 0.18; m.scale.y = 0.18; m.scale.z = 0.18
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
        self.plan_end_marker_pub.publish(m)

    def costmap_cb(self, msg: OccupancyGrid):
        """로컬 코스트맵 수신 및 frame_id 설정."""
        self.latest_costmap = msg
        if not self.global_frame:
            self.global_frame = self.global_frame_param if self.global_frame_param else msg.header.frame_id

    def mask_cb(self, msg: OccupancyGrid):
        """인도 마스크 그리드 수신 및 frame_id 설정."""
        self.latest_maskmap = msg
        if not self.global_frame:
            self.global_frame = self.global_frame_param if self.global_frame_param else msg.header.frame_id

    def global_costmap_cb(self, msg: OccupancyGrid):
        """글로벌 코스트맵 수신(있으면 추가 충돌 판정에 반영)."""
        self.latest_global_costmap = msg

    # ---------------- Main loop ----------------
    def on_timer(self):
        """주기적으로 샘플링 → 중심각 선택 → goal 후보 계산 → 스냅/보정 → 전송/마커 표시."""
        if self.latest_costmap is None or self.latest_maskmap is None or self.global_frame is None:
            return
        if not self.nav_client.server_is_ready():
            return

        # yaw 및 진행 방향 기준은 base_link에서 취득
        try:
            tf_base = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed {self.global_frame}->{self.base_frame}: {ex}")
            return
        bx = tf_base.transform.translation.x
        by = tf_base.transform.translation.y
        yaw = quat_to_yaw(tf_base.transform.rotation)

        # 반원 중심은 camera_link(요구사항)에서 취득 (실패 시 base_link 폴백)
        try:
            tf_center = self.tf_buffer.lookup_transform(self.global_frame, self.center_frame, rclpy.time.Time())
            cx = tf_center.transform.translation.x
            cy = tf_center.transform.translation.y
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed {self.global_frame}->{self.center_frame}: {ex} (fallback to {self.base_frame})")
            cx, cy = bx, by

        # 링 샘플링 및 경계 플래그 계산
        angles, world_pts, occupied, boundary = self.sample_circle(cx, cy, yaw)

        # free-run들 중 규칙에 따라 중심각 선택
        center_ang_raw, run_a, run_b = self.pick_center_angle_from_free_runs(angles, occupied, boundary)

        # 중심각 지터 완화
        center_ang = self.smooth_center_angle(center_ang_raw)

        # goal 후보 계산 (반경 R, 선택 중심각)
        gx = cx + self.R * math.cos(yaw + center_ang)
        gy = cy + self.R * math.sin(yaw + center_ang)

        # 인도 밖/장애물인 경우 백오프/미세각 탐색으로 스냅
        gx, gy = self.backoff_if_occupied_or_offsidewalk(cx, cy, yaw, center_ang, gx, gy)

        # goal yaw 설정
        gyaw = yaw + center_ang if self.yaw_from_center_angle else yaw

        # 과도한 재전송 방지(변화량 기준)
        if not self.should_send((gx, gy, gyaw)):
            self.publish_markers(bx, by, cx, cy, yaw, angles, world_pts,
                                 [self.is_occupied_local(x, y) or (not self.on_sidewalk(x, y)) for (x, y) in world_pts],
                                 boundary, (gx, gy), free_run=(run_a, run_b))
            return

        # PoseStamped 생성 및 Nav2 goal 전송
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
        self.nav_client.send_goal_async(nav_goal)

        self.last_goal_xyyaw = (gx, gy, gyaw)
        self.publish_markers(bx, by, cx, cy, yaw, angles, world_pts,
                             [self.is_occupied_local(x, y) or (not self.on_sidewalk(x, y)) for (x, y) in world_pts],
                             boundary, (gx, gy), free_run=(run_a, run_b))

    # ---------------- Geometry / Logic ----------------
    def is_occupied_in(self, costmap: OccupancyGrid, x, y, unknown_is_occ: bool):
        """특정 OccupancyGrid에서 (x,y)가 점유인지 판정. unknown 처리정책 포함."""
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
        return True  # 맵 밖은 위험 취급

    def is_occupied_local(self, x, y):
        """로컬 코스트맵 기준 점유 여부."""
        return self.is_occupied_in(self.latest_costmap, x, y, unknown_is_occ=self.unknown_occ)

    def on_sidewalk(self, x, y):
        """인도 마스크 그리드 기준 인도(0) 여부."""
        return not self.is_occupied_in(self.latest_maskmap, x, y, unknown_is_occ=True)

    def is_occupied_global(self, x, y):
        """글로벌 코스트맵이 있으면 추가 충돌 판정, 없으면 False."""
        if self.latest_global_costmap is None:
            return False
        return self.is_occupied_in(self.latest_global_costmap, x, y, unknown_is_occ=True)

    def sample_circle(self, cx, cy, yaw):
        """
        반경 R에서 [ang_min, ang_max] 각도로 링 샘플링.
        - occupied[i] = (로컬장애물) OR (인도 바깥)
        - boundary[i] = 내측(R-ε)은 인도 AND 외측(R+ε)은 비인도 → 경계 True
        """
        angles = np.arange(self.ang_min, self.ang_max + 1e-6, self.ang_step)
        world_pts, occupied, boundary = [], [], []
        eps = self.boundary_eps

        for a in angles:
            # 링 위 샘플점
            wx = cx + self.R * math.cos(yaw + a)
            wy = cy + self.R * math.sin(yaw + a)
            world_pts.append((wx, wy))

            occ_local = self.is_occupied_local(wx, wy)
            on_sw = self.on_sidewalk(wx, wy)
            occupied.append(occ_local or (not on_sw))

            # 경계 판별(내측 인도 & 외측 비인도)
            rin = max(0.4, self.R - eps)
            rout = self.R + eps
            wx_in = cx + rin * math.cos(yaw + a); wy_in = cy + rin * math.sin(yaw + a)
            wx_out = cx + rout * math.cos(yaw + a); wy_out = cy + rout * math.sin(yaw + a)
            boundary.append(self.on_sidewalk(wx_in, wy_in) and (not self.on_sidewalk(wx_out, wy_out)))

        return angles, world_pts, occupied, boundary

    def pick_center_angle_from_free_runs(self, angles, occupied, boundary):
        """
        free run들 중 규칙에 따라 중심각 선택.
        우선순위:
          1) run의 양끝이 '마스크 경계'인 경우 최우선 → 중앙각.
          2) 한쪽만 경계인 경우 → 인도 폭 W로 기대되는 호 길이(Δθ=2asin(W/2R)) 기반 보정.
          3) 경계가 전혀 없으면 → run의 중앙각.
        반환: (선택각, run 시작 인덱스, run 끝 인덱스)
        """
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

        ratio = min(0.999, self.W / (2.0 * self.R))
        expected = 2.0 * math.asin(ratio)

        def edge_score(a, b):
            """run[a:b]가 폭 기대치에 얼마나 근접한지 + 경계 보너스로 스코어링."""
            both = boundary[a] and boundary[b]
            one  = (boundary[a] != boundary[b]) and (boundary[a] or boundary[b])
            width = float(angles[b] - angles[a])
            score = -abs(width - expected)
            if both: score += 0.6
            elif one: score += 0.3
            if a == 0 or b == len(angles) - 1: score -= 0.5
            return score, both, one, width

        best = None
        for a, b in runs:
            sc, both, one, _ = edge_score(a, b)
            cand = (sc, a, b, both, one)
            if best is None or sc > best[0]:
                best = cand

        _, a, b, both, one = best
        if both:
            mid_idx = (a + b) // 2
            return angles[mid_idx], a, b
        if one:
            dtheta = expected
            if boundary[a] and (not boundary[b]):
                # run 시작쪽이 경계 → 그쪽에서 +Δθ/2
                return self._clamp_angle(angles[a] + 0.5 * dtheta), a, b
            else:
                # run 끝쪽이 경계 → 그쪽에서 -Δθ/2
                return self._clamp_angle(angles[b] - 0.5 * dtheta), a, b

        # 경계가 없으면 run 중앙
        mid_idx = (a + b) // 2
        return angles[mid_idx], a, b

    def _clamp_angle(self, ang):
        """선택각을 허용 각 범위[ang_min, ang_max]로 클램핑."""
        return max(self.ang_min, min(self.ang_max, ang))

    def smooth_center_angle(self, center_ang_raw):
        """
        중심각 지터 완화(IIR + 급변 제한).
        - max_step으로 급격한 변화 제한
        - α 계수로 IIR 필터 적용
        """
        if self.last_center_ang is None:
            self.last_center_ang = float(center_ang_raw)
            return center_ang_raw

        # 급변 제한
        delta = self._wrap_to_pi(center_ang_raw - self.last_center_ang)
        if abs(delta) > self.max_step:
            center_ang_raw = self.last_center_ang + self._sign(delta) * self.max_step

        # IIR
        smoothed = (1.0 - self.alpha) * self.last_center_ang + self.alpha * center_ang_raw
        smoothed = self._wrap_to_pi(smoothed)
        self.last_center_ang = float(smoothed)
        return smoothed

    def backoff_if_occupied_or_offsidewalk(self, cx, cy, yaw, center_ang, gx, gy):
        """
        goal 후보가 인도 밖이거나 점유(로컬/글로벌)일 경우,
        - 반경을 줄이며 스냅,
        - 그래도 안 되면 ±10° 범위에서 미세각 탐색 + 반경 백오프로 스냅.
        """
        def blocked(xx, yy):
            if not self.on_sidewalk(xx, yy):
                return True
            if self.is_occupied_local(xx, yy):
                return True
            if self.is_occupied_global(xx, yy):
                return True
            return False

        if not blocked(gx, gy):
            return gx, gy

        # 반경 백오프
        r = self.R
        while r > 1.0:
            r -= 0.15
            tx = cx + r * math.cos(yaw + center_ang)
            ty = cy + r * math.sin(yaw + center_ang)
            if not blocked(tx, ty):
                return tx, ty

        # 각도 미세 탐색(±10°)
        for ddeg in range(2, 11, 2):
            for sign in (+1, -1):
                a2 = center_ang + math.radians(sign * ddeg)
                r = self.R
                while r > 1.0:
                    r -= 0.15
                    tx = cx + r * math.cos(yaw + a2)
                    ty = cy + r * math.sin(yaw + a2)
                    if not blocked(tx, ty):
                        return tx, ty
        return gx, gy

    def should_send(self, new_xyyaw):
        """
        goal 재전송 여부 판단.
        - 위치 변화가 min_goal_move 초과거나
        - yaw 변화가 10° 초과면 전송.
        """
        if self.last_goal_xyyaw is None:
            return True
        dx = new_xyyaw[0] - self.last_goal_xyyaw[0]
        dy = new_xyyaw[1] - self.last_goal_xyyaw[1]
        dyaw = self._wrap_to_pi(new_xyyaw[2] - self.last_goal_xyyaw[2])
        if (dx * dx + dy * dy) ** 0.5 > self.min_goal_move:
            return True
        if abs(dyaw) > math.radians(10.0):
            return True
        return False

    @staticmethod
    def _wrap_to_pi(a):
        """각도를 [-π, π] 범위로 래핑."""
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    @staticmethod
    def _sign(x):
        """부호 함수(+1 또는 -1)."""
        return 1.0 if x >= 0.0 else -1.0

    # ---------------- Visualization ----------------
    def publish_markers(self, bx, by, cx, cy, yaw, angles, world_pts, occ_or_off, boundary, goal_xy, free_run=None):
        """
        디버깅/가시화를 위한 마커 퍼블리시:
        - 반원 중심(청록), 샘플 포인트(노랑/빨강), 경계(초록),
        - 선택된 free run 양끝(초록), goal(파랑), base→goal 화살표(파랑).
        """
        if self.global_frame is None:
            return
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        # 0) 반원 중심 마커
        m_center = Marker()
        m_center.header.frame_id = self.global_frame
        m_center.header.stamp = now
        m_center.ns = 'ring_center'
        m_center.id = 900
        m_center.type = Marker.SPHERE
        m_center.action = Marker.ADD
        m_center.pose.position.x = float(cx)
        m_center.pose.position.y = float(cy)
        m_center.pose.position.z = 0.03
        m_center.scale.x = 0.16; m_center.scale.y = 0.16; m_center.scale.z = 0.16
        m_center.color.r, m_center.color.g, m_center.color.b, m_center.color.a = 0.0, 0.8, 0.8, 0.9
        arr.markers.append(m_center)

        # 1) 샘플 포인트
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
            m.scale.x = 0.08; m.scale.y = 0.08; m.scale.z = 0.08
            if occ_or_off[i]:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.9
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 0.9
            arr.markers.append(m)

        # 2) 경계 접점 마커
        for i, is_b in enumerate(boundary):
            if not is_b:
                continue
            wx, wy = world_pts[i]
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp = now
            m.ns = 'boundary'
            m.id = 2000 + i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(wx)
            m.pose.position.y = float(wy)
            m.pose.position.z = 0.04
            m.scale.x = 0.16; m.scale.y = 0.16; m.scale.z = 0.16
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.2, 0.95
            arr.markers.append(m)

        # free run 경계 마커(선택 run의 양끝)
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
                m.scale.x = 0.16; m.scale.y = 0.16; m.scale.z = 0.16
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.2, 0.9
                arr.markers.append(m)

        # 3) goal 마커 + base_link→goal 화살표
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
        m_goal.scale.x = 0.22; m_goal.scale.y = 0.22; m_goal.scale.z = 0.22
        m_goal.color.r, m_goal.g, m_goal.b, m_goal.a = 0.2, 0.6, 1.0, 1.0
        arr.markers.append(m_goal)

        m_arrow = Marker()
        m_arrow.header.frame_id = self.global_frame
        m_arrow.header.stamp = now
        m_arrow.ns = 'goal_arrow'
        m_arrow.id = 4000
        m_arrow.type = Marker.ARROW
        m_arrow.action = Marker.ADD
        m_arrow.scale.x = 0.05; m_arrow.scale.y = 0.1; m_arrow.scale.z = 0.1
        m_arrow.color.r, m_arrow.g, m_arrow.b, m_arrow.a = 0.2, 0.6, 1.0, 1.0
        m_arrow.points = [Point(x=float(bx), y=float(by), z=0.05),
                          Point(x=float(gx), y=float(gy), z=0.05)]
        arr.markers.append(m_arrow)

        self.marker_pub.publish(arr)


def main(args=None):
    """rclpy 초기화 및 노드 스핀."""
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
