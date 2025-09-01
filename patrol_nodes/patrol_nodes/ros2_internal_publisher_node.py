#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ROS2InternalPublisherNode(Node):
    def __init__(self):
        super().__init__('ros2_internal_publisher_node')  # 노드 이름 설정
        # ROS2 퍼블리셔 설정: 'robot_to_app_topic' 토픽으로 String 메시지 발행
        self.publisher_ = self.create_publisher(String, 'robot_to_app_topic', 10)
        self.get_logger().info('ROS2InternalPublisherNode 시작: 메시지 입력 대기 중...')

    def run(self):
        # 사용자 입력을 받아 메시지 발행
        while rclpy.ok():
            text = input("로봇 메시지 입력 (엔터로 전송, 'q'로 종료): ")
            if text.lower() == 'q':
                break
            elif text:
                msg = String()
                msg.data = text
                self.publisher_.publish(msg)
                self.get_logger().info(f'ROS2 토픽 "{self.publisher_.topic_name}"으로 메시지 발행: "{text}"')

def main(args=None):
    rclpy.init(args=args)  # ROS2 초기화
    node = ROS2InternalPublisherNode()  # 노드 생성
    try:
        node.run()  # 노드 실행
    except KeyboardInterrupt:
        node.get_logger().info("ROS2InternalPublisherNode 종료 중...")
    finally:
        node.destroy_node()  # 노드 종료
        rclpy.shutdown()  # ROS2 종료

if __name__ == '__main__':
    main()
