#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import firebase_admin
from firebase_admin import credentials, firestore
from std_msgs.msg import String

class AppToROS2Node(Node):
    def __init__(self):
        super().__init__('app_to_ros2_node')  # 노드 이름 설정
        # Firebase 초기화: 앱이 초기화되지 않은 경우에만 실행 ※ json 파일 경로 틀리면 안됨.
        if not firebase_admin._apps:
            cred = credentials.Certificate('/home/ubuntu/Desktop/ros2_ws/src/patrol_nodes/patrol_nodes/campuspatrolrobotapp-firebase-adminsdk-fbsvc-78007f818b.json')
            firebase_admin.initialize_app(cred)
        self.db = firestore.client()  # Firestore 클라이언트 생성

        # ROS2 퍼블리셔 설정: 'app_to_robot_topic' 토픽으로 String 메시지 발행
        self.publisher_ = self.create_publisher(String, 'app_to_robot_topic', 10)
        self.get_logger().info("AppToROS2Node 시작: Firebase 메시지 모니터링 및 ROS2 발행 중...")

        # Firebase 메시지 실시간 모니터링 시작
        self.listen_to_firebase_messages()

    def listen_to_firebase_messages(self):
        # Firestore에서 사용자 메시지("sender": "user", "status": "pending") 쿼리
        query = self.db.collection("Messages") \
                       .where(filter=firestore.FieldFilter("sender", "==", "user")) \
                       .where(filter=firestore.FieldFilter("status", "==", "pending"))
        
        def on_snapshot(col_snapshot, changes, read_time):
            # Firestore 스냅샷 콜백: 변경 사항 처리
            self.get_logger().info(f"Snapshot received, changes: {len(changes)}")
            for change in changes:
                self.get_logger().info(f"Change type: {change.type.name}, Doc ID: {change.document.id}")
                if change.type.name == "ADDED":
                    doc = change.document
                    data = doc.to_dict()
                    text = data.get("text")
                    
                    if data.get("status") != "pending":
                        self.get_logger().info(f"Firebase에서 'pending'이 아닌 사용자 메시지 감지 (ID: {doc.id}, Status: {data.get('status')}), 건너뜀.")
                        continue

                    if text:
                        self.get_logger().info(f"Firebase에서 수신된 앱 메시지: '{text}'")
                        self.publish_to_ros2(text)  # ROS2로 메시지 발행
                        
                        try:
                            doc.reference.update({"status": "processed"})  # 메시지 상태 업데이트
                            self.get_logger().info(f"Firebase 문서 상태 'processed'로 업데이트 완료: {doc.id}")
                        except Exception as e:
                            self.get_logger().error(f"Firebase 문서 상태 업데이트 오류 (ID: {doc.id}): {str(e)}")
                elif change.type.name == "MODIFIED":
                    pass
                elif change.type.name == "REMOVED":
                    pass

        self.firestore_watch = query.on_snapshot(on_snapshot)  # Firestore 실시간 리스너 등록

    def publish_to_ros2(self, text):
        # ROS2 토픽으로 메시지 발행
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f"ROS2 토픽 '{self.publisher_.topic_name}'으로 발행: '{text}'")

def main(args=None):
    rclpy.init(args=args)  # ROS2 초기화
    node = AppToROS2Node()  # 노드 생성
    try:
        rclpy.spin(node)  # 노드 실행
    except KeyboardInterrupt:
        node.get_logger().info("AppToROS2Node 종료 중...")
    finally:
        if hasattr(node, 'firestore_watch') and node.firestore_watch:
            node.firestore_watch.unsubscribe()  # Firestore 리스너 해제
        node.destroy_node()  # 노드 종료
        rclpy.shutdown()  # ROS2 종료

if __name__ == '__main__':
    main()
