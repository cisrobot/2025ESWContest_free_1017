import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from datetime import datetime
import os

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/d455_camera/color/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()
        self.video_writer = None
        self.save_dir = '/home/marin/marine/src/yolo_segmentation'  # 저장할 경로 (필요시 변경)
        os.makedirs(self.save_dir, exist_ok=True)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge 변환 오류: {e}')
            return

        if self.video_writer is None:
            now = datetime.now().strftime('%Y%m%d_%H%M%S')
            video_path = os.path.join(self.save_dir, f'original_video_{now}.mp4')

            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            height, width, _ = frame.shape
            fps = 20.0  # 고정 fps, 혹은 동적으로 받으려면 별도 처리 필요

            self.video_writer = cv2.VideoWriter(video_path, fourcc, fps, (width, height))
            self.get_logger().info(f'원본 영상 저장 시작: {video_path}')
            self.video_path = video_path

        self.video_writer.write(frame)
        cv2.imshow("Original Camera Image", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        if self.video_writer:
            self.video_writer.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
