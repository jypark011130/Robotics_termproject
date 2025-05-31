import cv2
import numpy as np

# 최소·최대 면적 (픽셀 단위)
MIN_AREA = 500
MAX_AREA = 200000

# 마우스 클릭 시 호출되는 함수
def show_pixel(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # 전역 frame에서 y,x 위치의 BGR 값 읽기
        b, g, r = frame[y, x]
        # np.uint8 → int 변환
        b, g, r = int(b), int(g), int(r)
        # RGB 순서로 뒤집기
        rgb = (r, g, b)
        print(f"Clicked @ ({x},{y})  BGR = ({b}, {g}, {r})  |  RGB = {rgb}")

# 0: 내장, 1: USB 외장
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
# 해상도 설정
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

cv2.namedWindow("Camera Feed")
cv2.setMouseCallback("Camera Feed", show_pixel)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame Error")
        break

    # 1) 그레이스케일 & 블러 & 이진화
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    #source = camera feed
    src = frame.copy()

    # 2) 외곽선 그리기 + 무게중심 표시
    for cnt in contours:
        M = cv2.moments(cnt)
        area = M['m00']  # contourArea(cnt) 와 동일

        # 면적이 MIN_AREA 이상, MAX_AREA 이하일 때만 외곽선과 중심을 그림
        if MIN_AREA <= area <= MAX_AREA:
            # 외곽선
            cv2.drawContours(src, [cnt], -1, (0, 0, 255), 2)

            # 무게중심 계산 및 표시
            cX = int(M['m10'] / area)
            cY = int(M['m01'] / area)
            cv2.circle(src, (cX, cY), 3, (255, 0, 0), -1)

    # 3) 결과 표시
    cv2.imshow("Camera Feed", frame)
    cv2.imshow("gray", gray)
    cv2.imshow("thresholding", binary)
    cv2.imshow("Camera Feed + contours", src)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
