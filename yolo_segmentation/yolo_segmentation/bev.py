import cv2
import numpy as np

DST_POINTS = None  # 나중에 동적으로 만듦

def calculate_meter_per_pixel(p1, p2, real_distance_m):
    """
    두 점 픽셀 거리와 실제 거리를 받아 픽셀 당 미터 비율 계산
    """
    pixel_dist = np.linalg.norm(np.array(p2) - np.array(p1))
    meter_per_pixel = real_distance_m / pixel_dist
    return meter_per_pixel

def bev_transform_and_show(image, src_points, real_dist_width, real_dist_height):
    """
    src_points 기반 BEV 변환 수행 후 결과 출력
    픽셀당 미터 비율도 출력
    """
    # 두 축 방향 픽셀 거리 계산
    pixel_dist_width = np.linalg.norm(np.array(src_points[1]) - np.array(src_points[0]))
    pixel_dist_height = np.linalg.norm(np.array(src_points[2]) - np.array(src_points[3]))

    # 축별 픽셀 당 미터 비율 계산
    meter_per_pixel_width = real_dist_width / pixel_dist_width
    meter_per_pixel_height = real_dist_height / pixel_dist_height

    print(f"x축 픽셀당 미터 비율: {meter_per_pixel_width:.5f} m/pixel")
    print(f"y축 픽셀당 미터 비율: {meter_per_pixel_height:.5f} m/pixel")

    # BEV 출력 이미지 크기: 픽셀 단위로 width, height 각각
    bev_width = int(pixel_dist_width)
    bev_height = int(pixel_dist_height)

    global DST_POINTS
    DST_POINTS = np.float32([
        [0, bev_height],
        [bev_width, bev_height],
        [bev_width, 0],
        [0, 0],
    ])

    M = cv2.getPerspectiveTransform(np.float32(src_points), DST_POINTS)
    bev_img = cv2.warpPerspective(image, M, (bev_width, bev_height))

    cv2.imshow("BEV Image", bev_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def main():
    VIDEO_INPUT_PATH = '/home/marin/marine/src/yolo_segmentation/original_video_20250527_201058.mp4'

    cap = cv2.VideoCapture(VIDEO_INPUT_PATH)
    if not cap.isOpened():
        print(f"Error: Cannot open video file {VIDEO_INPUT_PATH}")
        return

    ret, frame = cap.read()
    if not ret:
        print("Error: Cannot read the first frame.")
        cap.release()
        return

    SRC_POINTS = np.float32([
        [6, 563],    # point 0
        [1278, 563], # point 1
        [770, 391],  # point 2
        [495, 391],  # point 3
    ])

    # 실제 거리 (m) 입력: 너비, 높이 방향 각각
    real_dist_width = 2.3  # 230cm
    real_dist_height = 4.4  # 440cm

    bev_transform_and_show(frame, SRC_POINTS, real_dist_width, real_dist_height)

    cap.release()

if __name__ == "__main__":
    main()
