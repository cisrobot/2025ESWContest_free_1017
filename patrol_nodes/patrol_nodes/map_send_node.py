#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

class MapSend(Node):
    def __init__(self):
        super().__init__('map_send')  # 노드 이름 설정
        # ROS2 퍼블리셔 설정: GPS 및 상태 데이터 발행
        self.event_publisher_ = self.create_publisher(NavSatFix, 'Event_GPS', 10)  # 이벤트 GPS 토픽
        self.realtime_publisher_ = self.create_publisher(NavSatFix, 'RealTime_GPS', 10)  # 실시간 GPS 토픽
        self.state_publisher_ = self.create_publisher(String, 'Robot_State', 10)  # 로봇 상태 토픽

        # 주기적 발행 타이머 설정
        self.event_timer = self.create_timer(30.0, self.publish_event_gps)  # 30초마다 이벤트 GPS 발행
        self.realtime_timer = self.create_timer(1.0, self.publish_realtime_gps)  # 1초마다 실시간 GPS 발행
        self.state_timer = self.create_timer(10.0, self.publish_state)  # 10초마다 상태 발행

        # 초기 GPS 좌표 및 상태 설정
        self.latitude = 37.5665  # 초기 위도 (예: 서울)
        self.longitude = 126.9780  # 초기 경도
        self.state_index = 0  # 상태 인덱스
        self.states = ['00', '01', '10', '11']  # 가능한 로봇 상태 목록

        self.get_logger().info('MapSend node started and publishing to ROS 2 topics.')

    def publish_event_gps(self):
        # 이벤트 GPS 데이터 발행 (30초 주기)
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()  # 현재 시간 스탬프
        msg.header.frame_id = 'gps'  # 프레임 ID
        msg.latitude = self.latitude  # 위도
        msg.longitude = self.longitude  # 경도
        msg.altitude = 10.0  # 고도 (고정값)
        self.event_publisher_.publish(msg)
        self.get_logger().info(f'Event_GPS published: lat={msg.latitude}, lon={msg.longitude}')

        # 다음 발행을 위해 좌표 업데이트
        self.latitude += 0.000025  # 위도 미세 조정
        self.longitude += 0.000025  # 경도 미세 조정

    def publish_realtime_gps(self):
        # 실시간 GPS 데이터 발행 (1초 주기)
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()  # 현재 시간 스탬프
        msg.header.frame_id = 'gps'  # 프레임 ID
        msg.latitude = self.latitude  # 위도
        msg.longitude = self.longitude  # 경도
        msg.altitude = 10.0  # 고도 (고정값)
        self.realtime_publisher_.publish(msg)
        self.get_logger().info(f'RealTime_GPS published: lat={msg.latitude}, lon={msg.longitude}')

        # 다음 발행을 위해 좌표 업데이트
        self.latitude += 0.000025  # 위도 미세 조정
        self.longitude += 0.000025  # 경도 미세 조정

    def publish_state(self):
        # 로봇 상태 발행 (10초 주기)
        msg = String()
        msg.data = self.states[self.state_index]  # 현재 상태 선택
        self.state_publisher_.publish(msg)
        self.get_logger().info(f'Robot_State published: state={msg.data}')

        # 다음 상태로 전환
        self.state_index = (self.state_index + 1) % len(self.states)

def main(args=None):
    rclpy.init(args=args)  # ROS2 초기화
    node = MapSend()  # 노드 생성
    rclpy.spin(node)  # 노드 실행
    node.destroy_node()  # 노드 종료
    rclpy.shutdown()  # ROS2 종료

if __name__ == '__main__':
    main()
