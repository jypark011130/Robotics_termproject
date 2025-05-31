import cv2
import numpy as np

# 마우스 클릭 콜백 함수
def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Clicked pixel coordinates: ({x}, {y})")

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # Windows라면 CAP_DSHOW 옵션이 빠질 수 있음

# 해상도 설정
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

cv2.namedWindow("Camera Feed")
cv2.setMouseCallback("Camera Feed", mouse_callback)  # 콜백 등록

while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame Error")
        break

    # 프레임 출력
    cv2.imshow("Camera Feed", frame)

    # 1ms 대기 후 'q' 누르면 루프 탈출
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 해제 및 윈도우 닫기
cap.release()
cv2.destroyAllWindows()
