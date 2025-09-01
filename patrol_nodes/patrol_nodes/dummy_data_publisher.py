#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import time

class DummyDataPublisher(Node):
    def __init__(self):
        super().__init__('dummy_data_publisher')

        # 코드 내에서 리매핑: /scooter_location -> Event_GPS, /mavros/global_position/global -> RealTime_GPS
        self.event_gps_publisher = self.create_publisher(NavSatFix, 'Event_GPS', 10)
        self.realtime_gps_publisher = self.create_publisher(NavSatFix, 'RealTime_GPS', 10)
        self.state_publisher = self.create_publisher(String, 'Robot_State', 10)

        # 상태 배열과 인덱스 추가
        self.states = ['00', '01', '10', '11']
        self.state_index = 0

        self.timer = self.create_timer(2.0, self.publish_dummy_data)
        self.get_logger().info('Dummy data publisher node started')

    def publish_dummy_data(self):
        # 1. Event_GPS 데이터 (킥보드 위치)
        event_msg = NavSatFix()
        event_msg.latitude = 36.100 + (time.time() % 100) / 10000.0
        event_msg.longitude = 128.400 + (time.time() % 100) / 10000.0
        event_msg.altitude = 0.0
        self.event_gps_publisher.publish(event_msg)
        self.get_logger().info(f'Publishing Event_GPS: lat={event_msg.latitude}, lon={event_msg.longitude}')

        # 2. RealTime_GPS 데이터 (로봇 현재 위치)
        realtime_msg = NavSatFix()
        realtime_msg.latitude = 36.101 + (time.time() % 50) / 10000.0
        realtime_msg.longitude = 128.401 + (time.time() % 50) / 10000.0
        realtime_msg.altitude = 5.0
        self.realtime_gps_publisher.publish(realtime_msg)
        self.get_logger().info(f'Publishing RealTime_GPS: lat={realtime_msg.latitude}, lon={realtime_msg.longitude}')

        # 3. Robot_State 데이터 (00, 01, 10, 11 순차적으로 발행)
        state_msg = String()
        state_msg.data = self.states[self.state_index]
        self.state_publisher.publish(state_msg)
        self.get_logger().info(f'Publishing Robot_State: state={state_msg.data}')

        # 다음 상태로 순환
        self.state_index = (self.state_index + 1) % len(self.states)

def main(args=None):
    rclpy.init(args=args)
    node = DummyDataPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
