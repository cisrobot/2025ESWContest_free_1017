"""
이 런치 파일은 patrol_nodes 패키지 안의 두 노드를 동시에 실행한다.

- no_dummmmy.py :
  로봇이 잘 켜지고 메시지를 받는 다리용 노드

- firestore_bridge.py :
  로봇이 지금 어디 있는지, 무슨 상태인지
    '파이어스토어(Firebase DB)'에 계속 올려주는 노드
  (밖에서도 로봇 위치를 확인할 수 있음)
"""

from launch import LaunchDescription
from launch_ros.actions import Node

# ROS2 런치 시스템이 실행할 노드 정의 및 객체 변환
def generate_launch_description():
    return LaunchDescription([
        #no_dummmmy.py
        Node(
            package='patrol_nodes',
            executable='no_dummmmy',
            name='no_dummmmy_node',
            output='screen'
        ),
        #firestore_bridge.py
        Node(
            package='patrol_nodes',
            executable='firestore_bridge',
            name='firestore_bridge_node',
            output='screen'
        ),
    ])