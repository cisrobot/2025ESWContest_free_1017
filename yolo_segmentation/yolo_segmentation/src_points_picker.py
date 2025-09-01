import cv2
import numpy as np

VIDEO_INPUT_PATH = '/home/marin/marine/src/yolo_segmentation/original_video_20250527_200631.mp4'
clicked_points = []

def mouse_callback(event, x, y, flags, param):
    global clicked_points
    if event == cv2.EVENT_LBUTTONDOWN and len(clicked_points) < 4:
        clicked_points.append((x, y))
        print(f"Point {len(clicked_points)}: ({x}, {y})")

cap = cv2.VideoCapture(VIDEO_INPUT_PATH)
if not cap.isOpened():
    print(f"Error: Cannot open video file {VIDEO_INPUT_PATH}")
    exit()

ret, frame = cap.read()
if not ret:
    print("Error: Cannot read the first frame.")
    cap.release()
    exit()

cv2.namedWindow("Select 4 Points")
cv2.setMouseCallback("Select 4 Points", mouse_callback)

while True:
    display = frame.copy()

    for idx, point in enumerate(clicked_points):
        cv2.circle(display, point, 8, (0, 255, 0), -1)
        cv2.putText(display, f"{idx+1}", (point[0]+10, point[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    cv2.imshow("Select 4 Points", display)

    if len(clicked_points) == 4:
        print("\nFinal SRC_POINTS (use this in your code):")
        print("SRC_POINTS = np.float32([")
        for pt in clicked_points:
            print(f"    [{pt[0]}, {pt[1]}],")
        print("])")
        break

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Quit before selecting 4 points.")
        break

cap.release()
cv2.destroyAllWindows()
