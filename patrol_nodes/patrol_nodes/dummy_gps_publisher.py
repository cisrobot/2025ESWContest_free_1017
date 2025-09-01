#!/usr/bin/env python3
import rclpy                
import time                 
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.qos import qos_profile_sensor_data

class DummyGpsPublisher(Node):
    def __init__(self):
        super().__init__('dummy_gps_publisher')

        # 발행자: 특정 토픽으로 GPS 데이터 발행
        self.event_gps_publisher = self.create_publisher(NavSatFix, '/scooter_location', 10)
        self.realtime_gps_publisher = self.create_publisher(NavSatFix, '/mavros/global_position/global', qos_profile_sensor_data)

        # 타이머: 1초마다 데이터 발행 함수 호출
        self.timer = self.create_timer(1.0, self.publish_dummy_gps_data)

        self.get_logger().info('DummyGpsPublisher 노드 시작. 중계 노드로 데이터 전송 중.')

    def publish_dummy_gps_data(self):
        # 1. 이벤트 GPS 데이터 (가상의 킥보드 위치) 생성 및 발행
        event_msg = NavSatFix()
        event_msg.header.stamp = self.get_clock().now().to_msg()
        event_msg.latitude = 36.100 + (time.time() % 100) / 10000.0
        event_msg.longitude = 128.400 + (time.time() % 100) / 10000.0
        event_msg.altitude = 10.0
        self.event_gps_publisher.publish(event_msg)
        self.get_logger().info(f'이벤트 GPS 발행: lat={event_msg.latitude:.6f}, lon={event_msg.longitude:.6f}')

        # 2. 실시간 GPS 데이터 (가상의 로봇 현재 위치) 생성 및 발행
        realtime_msg = NavSatFix()
        realtime_msg.header.stamp = self.get_clock().now().to_msg()
        realtime_msg.latitude = 36.101 + (time.time() % 50) / 10000.0
        realtime_msg.longitude = 128.401 + (time.time() % 50) / 10000.0
        realtime_msg.altitude = 15.0
        self.realtime_gps_publisher.publish(realtime_msg)
        self.get_logger().info(f'실시간 GPS 발행: lat={realtime_msg.latitude:.6f}, lon={realtime_msg.longitude:.6f}')

def main(args=None):
    rclpy.init(args=args)
    node = DummyGpsPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
