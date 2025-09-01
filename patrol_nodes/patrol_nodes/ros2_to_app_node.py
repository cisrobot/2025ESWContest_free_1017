#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import firebase_admin
from firebase_admin import credentials, firestore
from std_msgs.msg import String

class ROS2ToAppNode(Node):
    def __init__(self):
        super().__init__('ros2_to_app_node')  # 노드 이름 설정
        # Firebase 초기화: 앱이 초기화되지 않은 경우에만 실행 ※ json 파일 경로 틀리면 안됨.
        if not firebase_admin._apps:
            cred = credentials.Certificate('/home/ubuntu/Desktop/ros2_ws/src/patrol_nodes/patrol_nodes/campuspatrolrobotapp-firebase-adminsdk-fbsvc-78007f818b.json')
            firebase_admin.initialize_app(cred)
        self.db = firestore.client()  # Firestore 클라이언트 생성

        # ROS2 구독자 설정: 'robot_to_app_topic' 토픽에서 String 메시지 수신
        self.subscription = self.create_subscription(
            String,
            'robot_to_app_topic',
            self.listener_callback,
            10
        )
        self.get_logger().info("ROS2ToAppNode 시작: ROS2 메시지 수신 및 Firebase 저장 대기 중...")

    def listener_callback(self, msg):
        # 수신된 ROS2 메시지 처리
        text = msg.data
        self.get_logger().info(f"ROS2 토픽 '{self.subscription.topic_name}'에서 수신된 메시지: '{text}'")
        self.send_to_firebase(text)  # 메시지를 Firebase로 전송

    def send_to_firebase(self, text):
        # Firebase Firestore에 메시지 저장
        try:
            doc_ref = self.db.collection("Messages").add({
                "text": text,
                "sender": "robot",
                "timestamp": firestore.SERVER_TIMESTAMP,
                "status": "unread" 
            })
            self.get_logger().info(f"Firebase Firestore에 메시지 저장 성공: '{text}' (문서 ID: {doc_ref[1].id})")
        except Exception as e:
            self.get_logger().error(f"Firebase 저장 오류: {str(e)}")

def main(args=None):
    rclpy.init(args=args)  # ROS2 초기화
    node = ROS2ToAppNode()  # 노드 생성
    try:
        rclpy.spin(node)  # 노드 실행
    except KeyboardInterrupt:
        node.get_logger().info("ROS2ToAppNode 종료 중...")
    finally:
        node.destroy_node()  # 노드 종료
        rclpy.shutdown()  # ROS2 종료

if __name__ == "__main__":
    main()
