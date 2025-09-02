#!/usr/bin/env python3
'''
    - 킥보드 위치 이벤트 GPS(/scooter_location) -> /scooter로 리매핑 
    - 로봇의 실시간 위치 GPS(/mavros/global_position/global) -> /gps_data로 리매핑 
    - 로봇 상태 코드 status [2]를 2초마다 -> /Event_msg로 발행 
    ** 외부에서 들어온 GPS 데이터를 다른 토픽으로 리매핑해주고 로봇 상태 및 이벤트를 주기적으로 알리는 "데이터 릴레이 + 이벤트 발행기" 역할
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data # 센서 데이터 QoS 프로파일 임포트

class DataRelayAndStatePublisher(Node):
    def __init__(self):
        super().__init__('data_relay_and_state_publisher')

        # ------발행자 설정-------------
        self.event_publisher_ = self.create_publisher(NavSatFix, '/scooter', 10)
        self.realtime_publisher_ = self.create_publisher(NavSatFix, '/gps_data', qos_profile_sensor_data)
        self.state_publisher_ = self.create_publisher(String, '/Event_msg', 10)

        self.flag = 0

        # ------구독자 설정 ------------
        # 킥보드 위치 이벤트 구독 (외부에서 들어오는 이벤트 GPS 데이터)
        self.event_subscription_ = self.create_subscription(
            NavSatFix,
            '/scooter_location', # 이 토픽에서 데이터를 구독합니다.
            self.event_callback,
            10)
        self.get_logger().info('Subscribing to /scooter_location for event GPS.')

        # 로봇 현재 위치 구독 
        self.realtime_subscription_ = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global', # 이 토픽에서 데이터를 구독합니다.
            self.realtime_callback,
            qos_profile_sensor_data) # 실시간 데이터이므로 센서 QoS 적용
        self.get_logger().info('Subscribing to /mavros/global_position/global for realtime GPS.')

        # 3. 로봇 이벤트 관리
        self.states = ['00', '01', '10', '11']
        self.state_index = 0
        # 2초마다 로봇 상태를 발행하는 타이머
        self.state_publish_timer_ = self.create_timer(2.0, self.publish_robot_state)

        self.get_logger().info('DataRelayAndStatePublisher node started, acting as data relay and state generator.')

    # 구독: /scooter_location 콜백 함수
    def event_callback(self, msg: NavSatFix):
        # 수신된 이벤트 GPS 데이터를 /scooter 토픽으로 그대로 발행
        self.event_publisher_.publish(msg)
        self.get_logger().info(f'Relayed Event_GPS from /scooter_location: lat={msg.latitude}, lon={msg.longitude}')

    # 구독: /mavros/global_position/global 콜백 함수
    def realtime_callback(self, msg: NavSatFix):
        # 수신된 실시간 GPS 데이터를 /gps_data 토픽으로 그대로 발행
        self.realtime_publisher_.publish(msg)
        self.get_logger().info(f'Relayed RealTime_GPS from /mavros/global_position/global: lat={msg.latitude}, lon={msg.longitude}')

    # 로봇 이벤트를 주기적으로 발행하는 함수
    def publish_robot_state(self):
        state_msg = String()
        state_msg.data = self.states[self.state_index]
        self.state_publisher_.publish(state_msg)
        self.get_logger().info(f'Publishing Robot_State: state={state_msg.data}')
        self.state_index = 2

def main(args=None):
    rclpy.init(args=args)
    node = DataRelayAndStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()