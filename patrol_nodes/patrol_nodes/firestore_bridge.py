#!/usr/bin/env python3
'''
    no_dummmy 파일에서의 노드로부터 로봇의 위치 및 이벤트 데이터를 Firebase Firestore에 올려
    외부 어플에서 실시간으로 확인가능하게 해주는 노드 

    1. /scooter (이벤트 GPS)  → 'Event_Gps_Data' 컬렉션에 조건부 저장
    Firestore의 'Robot_State/current_state' 값이 '10'일 때만 이벤트 기록
    같은 좌표가 10초 안에 또 들어오면 중복으로 간주(로그만 남김)

    2. /gps_data (실시간 로봇 GPS)
    지도를 매끄럽게 갱신하기 위한 최신 위치 저장 용도

    3. /Event_msg 
    Robot_State/current_state' 문서에 상태+타임스탬프(그리고 최근 GPS) 저장
    외부에서 로봇 현재 상태를 한눈에 확인하기 위함
'''
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from firebase_admin import credentials, firestore, initialize_app, _apps
from rclpy.qos import qos_profile_sensor_data

class FirestoreBridge(Node):
    def __init__(self):
        super().__init__('firestore_bridge')

        # Firebase 초기화 (Admin SDK)
        try:
            if not _apps:
                cred = credentials.Certificate(
                    '/home/marin/marine/src/patrol_nodes/patrol_nodes/campuspatrolrobotapp-firebase-adminsdk-fbsvc-78007f818b.json'
                )
                initialize_app(cred)
            self.db = firestore.client()
            self.get_logger().info('Firebase initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Firebase initialization failed: {e}')
            raise e

         # 구독자 설정, no_dummy 파일에서 3가지 토픽을 구독
        self.subscription_event_gps = self.create_subscription( 
            NavSatFix, '/scooter', self.event_gps_callback, 10)
        self.subscription_realtime_gps = self.create_subscription(
            NavSatFix, '/gps_data', self.realtime_gps_callback, qos_profile_sensor_data)
        self.subscription_state = self.create_subscription(
            String, '/Event_msg', self.state_callback, 10)

        # 상태 변수 초기화,  # 내부 상태(중복 방지/최근값 보관)
        self.last_event_coords = None
        self.last_event_time = 0.0
        self.last_gps_data = None
        self.last_state = None

        self.get_logger().info('FirestoreBridge node started')

    def event_gps_callback(self, msg: NavSatFix):
        """
        Event_GPS 토픽 콜백 함수.
        이 함수는 특정 이벤트(예: 킥보드 감지)와 관련된 GPS 데이터를 처리하는 부분이며
        로봇 상태가 '10'일 경우에만 Firestore의 'Event_Gps_Data' 컬렉션에
        이벤트 데이터를 저장한다.
        """
        self.get_logger().info(f'Event_GPS received: lat={msg.latitude}, lon={msg.longitude}')
        if msg.latitude is not None and msg.longitude is not None:
            current_coords = (msg.latitude, msg.longitude)
            current_time = time.time()

            if self.last_event_coords == current_coords and (current_time - self.last_event_time) < 10.0:
                self.get_logger().info('Duplicate Event_GPS within 3 seconds, storing but no notification')

            state_doc_ref = self.db.collection('Robot_State').document('current_state')
            state_doc = state_doc_ref.get()
            current_state = state_doc.to_dict().get('state', 'unknown') if state_doc.exists else 'unknown'
            self.last_state = current_state

            if current_state == '10':
                event_data = {
                    'latitude': msg.latitude,
                    'longitude': msg.longitude,
                    'altitude': msg.altitude,
                    'eventType': 'Kickboard Detected',
                    'state': current_state,
                    'timestamp': firestore.SERVER_TIMESTAMP
                }
                try:
                    self.db.collection('Event_Gps_Data').add(event_data)
                    self.get_logger().info('Event sent to Event_Gps_Data Firestore for state 10')
                except Exception as e:
                    self.get_logger().error(f'Failed to send event data to Firestore: {e}')

            self.last_event_coords = current_coords
            self.last_event_time = current_time
            self.last_gps_data = msg
        else:
            self.get_logger().warning('Invalid GPS data in Event_GPS')

    def realtime_gps_callback(self, msg: NavSatFix):
        """
        RealTime_GPS 토픽 콜백 함수.
        이 함수는 로봇의 실시간 GPS 데이터를 처리한다. 
        Firestore의 전용 문서('Robot_Current_Location/robot_1')를 지속적으로 업데이트하여
        부드러운 지도 표시를 가능하게 한다.
        """
        self.get_logger().info(f'RealTime_GPS received: lat={msg.latitude}, lon={msg.longitude}')
        if msg.latitude is not None and msg.longitude is not None:
            current_location_data = {
                'latitude': msg.latitude,
                'longitude': msg.longitude,
                'altitude': msg.altitude,
                'timestamp': firestore.SERVER_TIMESTAMP
            }
            try:
                self.db.collection('Robot_Current_Location').document('robot_1').set(current_location_data)
                self.get_logger().info('Robot current location updated in Robot_Current_Location Firestore.')
            except Exception as e:
                self.get_logger().error(f'Failed to update robot current location in Firestore: {e}')

            self.last_gps_data = msg
        else:
            self.get_logger().warning('Invalid GPS data in RealTime_GPS')

    def state_callback(self, msg: String):
        """
        Robot_State 토픽 콜백 함수.
        이 함수는 로봇의 현재 상태를 Firestore의 전용 문서
        ('Robot_State/current_state')에 업데이트한다.
        또한, 가장 최근에 알려진 GPS 좌표를 상태 업데이트와 함께 포함하여
        정보를 제공한다.
        """
        self.get_logger().info(f'Robot_State received: state={msg.data}')
        state_data = {
            'state': msg.data,
            'timestamp': firestore.SERVER_TIMESTAMP
        }
        if self.last_gps_data:
            state_data['latitude'] = self.last_gps_data.latitude
            state_data['longitude'] = self.last_gps_data.longitude
            state_data['altitude'] = self.last_gps_data.altitude

        try:
            self.db.collection('Robot_State').document('current_state').set(state_data)
            self.get_logger().info(f'State sent to Robot_State Firestore: state={msg.data}, lat={state_data.get("latitude")}, lon={state_data.get("longitude")}')
        except Exception as e:
            self.get_logger().error(f'Failed to send state to Firestore: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = FirestoreBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()