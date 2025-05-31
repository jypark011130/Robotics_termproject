import cv2
import numpy as np
from FOV import pixel_to_real_with_fov
from pixel_to_meter import pixel_to_real_position
import time

# ──────────────────────────────────────────────────────────────────
# 출력 빈도 제어용 변수
last_print_time = 0
print_interval = 1.0  # 1초 간격 터미널 출력

# 최소·최대 면적 (픽셀 단위) — 환경에 맞춰 튜닝하세요
MIN_AREA = 500
MAX_AREA = 10000
# ──────────────────────────────────────────────────────────────────

# 카메라 설정 (장치 번호는 환경에 맞게 0 또는 1 등으로 변경)
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

cv2.namedWindow("Camera Feed + Classified", cv2.WINDOW_NORMAL)
cv2.namedWindow("Edges", cv2.WINDOW_NORMAL)

# Morphology용 커널 (에지 연결)
morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # (1) Blur → Grayscale
    blurred = cv2.GaussianBlur(frame, (7, 7), 0)
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)

    # (2) Canny Edge 검출 (파라미터는 환경에 맞춰 튜닝)
    edges = cv2.Canny(gray, 40, 180)

    # (3) Morphology Close로 끊긴 에지 연결
    edges_closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, morph_kernel, iterations=2)

    # (4) Contour 검출 (RETR_EXTERNAL: 최외곽 contour만)
    contours, _ = cv2.findContours(edges_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # (5) 결과 그릴 복사본
    src = frame.copy()

    # RGB 프레임 준비 (분류할 때 사용)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        # (5-1) 면적 필터링: 너무 작거나 너무 큰 컨투어 건너뛰기
        if area < MIN_AREA or area > MAX_AREA:
            continue

        # (5-2) 컨투어 내부 마스크 생성
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [cnt], -1, 255, -1)

        # (5-3) RGB 평균값 계산 (r_mean, g_mean, b_mean)
        r_mean, g_mean, b_mean = cv2.mean(rgb_frame, mask=mask)[:3]

        # (5-4) 평균값 기반으로 색 분류 → BGR 순서로 표시 색 결정
        if r_mean > g_mean and r_mean > b_mean:
            box_color = (0, 0, 255)   # 빨강
            label = "RED"
        elif g_mean > r_mean and g_mean > b_mean:
            box_color = (0, 255, 0)   # 초록
            label = "GREEN"
        else:
            box_color = (255, 0, 0)   # 파랑
            label = "BLUE"

        # (5-5) 분류된 색으로 컨투어 외곽선만 그리기
        cv2.drawContours(src, [cnt], -1, box_color, 2)

        # (5-6) 무게중심 계산 및 표시
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(src, (cX, cY), 5, box_color, -1)

            # (5-7) 픽셀→실제(mm) 좌표 변환 (FOV 함수 주석 처리 가능)
            # X_fov, Y_fov = pixel_to_real_with_fov(cX, cY)
            X_simple, Y_simple = pixel_to_real_position(cX, cY)

            # (5-8) 터미널 출력 (1초에 한 번)
            now = time.time()
            if now - last_print_time >= print_interval:
                #print(f"FOV 변환 실제 좌표 (mm): X={X_fov:.1f}, Y={Y_fov:.1f}")
                print(f"[{label}] Centroid Pixel: ({cX},{cY})  |  Real(mm): ({X_simple:.1f}, {Y_simple:.1f})")
                last_print_time = now

            # (5-9) 텍스트 표시: Pixel, Real 좌표 및 라벨
            text = f"{label} Pix({cX},{cY}) Real({int(X_simple)},{int(Y_simple)})"
            cv2.putText(src, text, (cX + 10, cY - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)

    # (6) 결과 화면 출력
    cv2.imshow("Camera Feed + Classified", src)
    cv2.imshow("Edges", edges_closed)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
