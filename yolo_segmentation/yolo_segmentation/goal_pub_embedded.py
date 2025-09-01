#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
goal_pub_embedded.py
- local_costmap + sidewalk_mask_grid(인도=0, 비인도=100) 기반으로
  R 반원 샘플링 → 경계(마스크 in/out) 검출 → 중앙각 선택 → goal 전송
- 규칙:
  1) goal은 항상 '인도' 내부.
  2) 좌/우 두 경계가 있으면 그 중앙.
  3) 한쪽 경계만 있으면 인도 폭 W와 코스트맵 FOV를 고려해 중앙 추정.
  4) 경계가 없으면 정면.
"""

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
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


class GoalPubEmbedded(Node):
    def __init__(self):
        super().__init__('goal_pub_embedded')

        # -------- Parameters --------
        self.declare_parameter('costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('sidewalk_mask_topic', '/sidewalk_mask_grid')   # 인도=0, 비인도=100
        self.declare_parameter('global_costmap_topic', '/global_costmap/costmap')  # 선택 안전판
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('global_frame', '')
        self.declare_parameter('goal_radius', 2.9)         # R (m)
        self.declare_parameter('sidewalk_width', 3.65)     # 인도 폭 W (m)
        self.declare_parameter('angle_min_deg', -90.0)     # 샘플 시작 각
        self.declare_parameter('angle_max_deg',  90.0)     # 샘플 끝 각
        self.declare_parameter('angle_step_deg',   2.0)    # 샘플 간격
        self.declare_parameter('occupancy_threshold', 65)  # 점유 임계(0~100)
        self.declare_parameter('unknown_is_occupied', False)
        self.declare_parameter('update_period', 0.5)       # s
        self.declare_parameter('min_goal_move', 0.30)      # m
        self.declare_parameter('yaw_from_center_angle', True)
        self.declare_parameter('boundary_eps_m', 0.10)     # 경계 판정용 R±ε

        # Read params
        self.costmap_topic = self.get_parameter('costmap_topic').get_parameter_value().string_value
        self.mask_topic = self.get_parameter('sidewalk_mask_topic').get_parameter_value().string_value
        self.global_costmap_topic = self.get_parameter('global_costmap_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.global_frame_param = self.get_parameter('global_frame').get_parameter_value().string_value

        self.R = float(self.get_parameter('goal_radius').value)
        self.W = float(self.get_parameter('sidewalk_width').value)
        self.ang_min = math.radians(float(self.get_parameter('angle_min_deg').value))
        self.ang_max = math.radians(float(self.get_parameter('angle_max_deg').value))
        self.ang_step = math.radians(float(self.get_parameter('angle_step_deg').value))
        self.occ_th = int(self.get_parameter('occupancy_threshold').value)
        self.unknown_occ = bool(self.get_parameter('unknown_is_occupied').value)
        self.period = float(self.get_parameter('update_period').value)
        self.min_goal_move = float(self.get_parameter('min_goal_move').value)
        self.yaw_from_center_angle = bool(self.get_parameter('yaw_from_center_angle').value)
        self.boundary_eps = float(self.get_parameter('boundary_eps_m').value)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subs
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.costmap_sub  = self.create_subscription(OccupancyGrid, self.costmap_topic,  self._costmap_cb, qos)
        self.mask_sub     = self.create_subscription(OccupancyGrid, self.mask_topic,     self._mask_cb, qos)
        self.global_sub   = self.create_subscription(OccupancyGrid, self.global_costmap_topic, self._global_cb, qos)

        # Action
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Viz
        self.marker_pub = self.create_publisher(MarkerArray, '/goal_pub_embedded/markers', 10)

        # State
        self.global_frame = None
        self.latest_costmap: OccupancyGrid | None = None
        self.latest_mask: OccupancyGrid | None = None
        self.latest_global: OccupancyGrid | None = None
        self.last_goal_xyyaw = None

        # Timer
        self.create_timer(self.period, self._on_timer)

        self.get_logger().info(
            f'goal_pub_embedded: cm={self.costmap_topic}, mask={self.mask_topic}, '
            f'R={self.R:.2f}m, W={self.W:.2f}m, range=[{math.degrees(self.ang_min):.0f},{math.degrees(self.ang_max):.0f}]'
        )

    # ----------------- Callbacks -----------------
    def _costmap_cb(self, msg: OccupancyGrid):
        self.latest_costmap = msg
        if not self.global_frame:
            self.global_frame = self.global_frame_param if self.global_frame_param else msg.header.frame_id

    def _mask_cb(self, msg: OccupancyGrid):
        self.latest_mask = msg
        if not self.global_frame:
            self.global_frame = self.global_frame_param if self.global_frame_param else msg.header.frame_id

    def _global_cb(self, msg: OccupancyGrid):
        self.latest_global = msg

    # ----------------- Core loop -----------------
    def _on_timer(self):
        if self.latest_costmap is None or self.latest_mask is None or self.global_frame is None:
            return
        if not self.nav_client.server_is_ready():
            return

        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f"TF {self.global_frame}->{self.base_frame} 실패: {ex}")
            return

        bx = tf.transform.translation.x
        by = tf.transform.translation.y
        yaw = quat_to_yaw(tf.transform.rotation)

        # 1) 샘플링 + 경계 판정
        angles, pts, free_good, boundary = self._sample_and_classify(bx, by, yaw)

        # 2) 중앙각 선택 (두 경계/한 경계/무경계)
        center_ang, run_a, run_b = self._pick_center_angle(angles, free_good, boundary)

        # 3) 후보 goal
        gx = bx + self.R * math.cos(yaw + center_ang)
        gy = by + self.R * math.sin(yaw + center_ang)

        # 4) 인도 안으로 스냅(막히면 백오프 + ±10° 미세탐색)
        gx, gy = self._snap_to_sidewalk(bx, by, yaw, center_ang, gx, gy)

        gyaw = yaw + center_ang if self.yaw_from_center_angle else yaw

        # 5) 최소 변화량 확인
        if not self._should_send((gx, gy, gyaw)):
            self._publish_markers(bx, by, angles, pts, free_good, (gx, gy), run=(run_a, run_b))
            return

        # 6) 전송
        goal = PoseStamped()
        goal.header.frame_id = self.global_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(gx)
        goal.pose.position.y = float(gy)
        goal.pose.orientation.z = math.sin(gyaw * 0.5)
        goal.pose.orientation.w = math.cos(gyaw * 0.5)

        nav_goal = NavigateToPose.Goal(); nav_goal.pose = goal
        self.nav_client.send_goal_async(nav_goal)
        self.last_goal_xyyaw = (gx, gy, gyaw)

        self._publish_markers(bx, by, angles, pts, free_good, (gx, gy), run=(run_a, run_b))

    # ----------------- Geometry / Logic -----------------
    def _is_occ_in(self, grid: OccupancyGrid, x, y, unknown_occ=True):
        res = grid.info.resolution
        w, h = grid.info.width, grid.info.height
        ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
        mx = int((x - ox) / res); my = int((y - oy) / res)
        if 0 <= mx < w and 0 <= my < h:
            v = grid.data[my * w + mx]
            if v < 0:
                return unknown_occ
            return v >= self.occ_th
        return True

    def _is_occ_local(self, x, y):
        return self._is_occ_in(self.latest_costmap, x, y, self.unknown_occ)

    def _is_occ_global(self, x, y):
        if self.latest_global is None:
            return False
        return self._is_occ_in(self.latest_global, x, y, True)

    def _on_sidewalk(self, x, y):
        # sidewalk_mask_grid: 0=sidewalk, 100=non-sidewalk
        return not self._is_occ_in(self.latest_mask, x, y, True)

    def _sample_and_classify(self, bx, by, yaw):
        """
        각도별 샘플.
        free_good[i] = (local free) AND (on sidewalk)
        boundary[i]  = on_sw(R-ε)=True AND on_sw(R+ε)=False  (마스크 경계)
        """
        angles = np.arange(self.ang_min, self.ang_max + 1e-6, self.ang_step)
        pts, free_good, boundary = [], [], []
        eps = self.boundary_eps

        for a in angles:
            # 반경 R 지점
            dx = self.R * math.cos(a); dy = self.R * math.sin(a)
            xR = bx + (math.cos(yaw) * dx - math.sin(yaw) * dy)
            yR = by + (math.sin(yaw) * dx + math.cos(yaw) * dy)
            pts.append((xR, yR))

            good = (not self._is_occ_local(xR, yR)) and self._on_sidewalk(xR, yR)
            free_good.append(good)

            # 경계 판정 (in: R-ε, out: R+ε)
            rin = max(0.5, self.R - eps)
            rout = self.R + eps
            xi = bx + (math.cos(yaw) * (rin * math.cos(a)) - math.sin(yaw) * (rin * math.sin(a)))
            yi = by + (math.sin(yaw) * (rin * math.cos(a)) + math.cos(yaw) * (rin * math.sin(a)))
            xo = bx + (math.cos(yaw) * (rout * math.cos(a)) - math.sin(yaw) * (rout * math.sin(a)))
            yo = by + (math.sin(yaw) * (rout * math.cos(a)) + math.cos(yaw) * (rout * math.sin(a)))
            boundary.append(self._on_sidewalk(xi, yi) and (not self._on_sidewalk(xo, yo)))

        return angles, pts, free_good, boundary

    def _pick_center_angle(self, angles, free_good, boundary):
        """
        free run(연속 True) 들 중에서 선택:
          - 양 끝이 경계면 → 최우선, 중앙각
          - 한쪽만 경계면 → 인도 폭 W를 R에서 이루는 각폭 Δθ=2 asin(min(0.999,W/(2R)))
                            + run/FOV 범위를 넘지 않도록 클램프해 중앙 추정
          - 경계 없으면 → run 중앙
          - run 자체 없으면 → 0 rad
        """
        # run 수집
        runs = []
        s = None
        for i, f in enumerate(free_good):
            if f and s is None:
                s = i
            elif (not f) and s is not None:
                runs.append((s, i - 1)); s = None
        if s is not None:
            runs.append((s, len(free_good) - 1))
        if not runs:
            return 0.0, None, None

        exp = 2.0 * math.asin(min(0.999, self.W / (2.0 * self.R)))  # 기대 각폭

        def score_run(a, b):
            width = float(angles[b] - angles[a])
            left_bd  = boundary[a]
            right_bd = boundary[b]
            both = left_bd and right_bd
            one  = (left_bd != right_bd)
            edge = (a == 0) or (b == len(angles) - 1)
            score = -abs(width - exp)
            if both: score += 0.6
            elif one: score += 0.3
            if edge: score -= 0.5
            return score, left_bd, right_bd, width

        # 최고 점수 run 선택
        best = None
        for a, b in runs:
            sc, lb, rb, w = score_run(a, b)
            if best is None or sc > best[0]:
                best = (sc, a, b, lb, rb, w)

        _, a, b, lb, rb, width = best
        if lb and rb:  # 두 경계
            mid = (a + b) // 2
            return angles[mid], a, b

        if lb ^ rb:   # 한 경계
            dtheta = exp
            if lb and not rb:
                # run 시작쪽이 경계 → 그쪽에서 +Δθ/2 (단, run/FOV 내로 클램프)
                center = angles[a] + 0.5 * dtheta
                center = min(center, angles[b])  # run의 끝 넘지 않기
                return center, a, b
            else:
                # run 끝쪽이 경계 → 그쪽에서 -Δθ/2
                center = angles[b] - 0.5 * dtheta
                center = max(center, angles[a])
                return center, a, b

        # 경계 없으면 run 중앙
        mid = (a + b) // 2
        return angles[mid], a, b

    def _snap_to_sidewalk(self, bx, by, yaw, ang, gx, gy):
        """goal이 인도 밖/장애물이면 R를 줄이며 인도 안 안전지점으로 스냅"""
        def bad(x, y):
            if not self._on_sidewalk(x, y):
                return True
            if self._is_occ_local(x, y):
                return True
            if self._is_occ_global(x, y):
                return True
            return False

        if not bad(gx, gy):
            return gx, gy

        r = self.R
        while r > 1.0:
            r -= 0.15
            tx = bx + r * math.cos(yaw + ang)
            ty = by + r * math.sin(yaw + ang)
            if not bad(tx, ty):
                return tx, ty

        # 각도 미세탐색(±10°)
        for ddeg in range(2, 11, 2):
            for sgn in (+1, -1):
                a2 = ang + math.radians(sgn * ddeg)
                r = self.R
                while r > 1.0:
                    r -= 0.15
                    tx = bx + r * math.cos(yaw + a2)
                    ty = by + r * math.sin(yaw + a2)
                    if not bad(tx, ty):
                        return tx, ty
        return gx, gy

    # ----------------- Helpers -----------------
    def _should_send(self, new_xyyaw):
        if self.last_goal_xyyaw is None:
            return True
        dx = new_xyyaw[0] - self.last_goal_xyyaw[0]
        dy = new_xyyaw[1] - self.last_goal_xyyaw[1]
        dyaw = self._norm_ang(new_xyyaw[2] - self.last_goal_xyyaw[2])
        if (dx * dx + dy * dy) ** 0.5 > self.min_goal_move:
            return True
        if abs(dyaw) > math.radians(10.0):
            return True
        return False

    @staticmethod
    def _norm_ang(a):
        while a > math.pi: a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a

    # ----------------- Visualization -----------------
    def _publish_markers(self, bx, by, angles, pts, free_good, goal_xy, run=None):
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        # 샘플 포인트(노랑=유효, 빨강=무효)
        for i, (wx, wy) in enumerate(pts):
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp = now
            m.ns = 'samples'
            m.id = 1000 + i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(wx)
            m.pose.position.y = float(wy)
            m.pose.position.z = 0.02
            m.scale.x = m.scale.y = m.scale.z = 0.08
            if free_good[i]:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 0.9
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.6
            arr.markers.append(m)

        if run and run[0] is not None and run[1] is not None:
            a, b = run
            for tag, idx, mid in (('run_start', a, 2100), ('run_end', b, 2101)):
                wx, wy = pts[idx]
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
                m.scale.x = m.scale.y = m.scale.z = 0.16
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.2, 0.9
                arr.markers.append(m)

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
        m_goal.scale.x = m_goal.scale.y = m_goal.scale.z = 0.22
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
    rclpy.init(args=args)
    node = GoalPubEmbedded()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
