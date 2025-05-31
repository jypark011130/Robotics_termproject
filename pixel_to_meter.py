import numpy as np

# 카메라와 작업 공간 설정 (이미지 기반)
PIXEL_WIDTH = 1280
PIXEL_HEIGHT = 720

# 실제 작업 공간 최대 거리(mm 단위) - 직접 측정하거나 실험으로 정해야 함
X_max = 600  # 예: 가로 작업 가능 최대 600mm
Y_max = 400  # 예: 세로 작업 가능 최대 400mm

# 카메라 중심 픽셀 (이미지 중심)
P_cx = PIXEL_WIDTH / 2
P_cy = PIXEL_HEIGHT / 2

def pixel_to_real_position(Px, Py):
    """
    카메라 픽셀 좌표 (Px, Py)를 작업 공간 실제 좌표 (X, Y)로 변환

    Args:
        Px, Py : 픽셀 좌표 (정수 또는 실수)
    Returns:
        X, Y : 실제 작업 공간 좌표 (mm 단위)
    """

    dx_camera = Px - P_cx
    dy_camera = Py - P_cy

    # 픽셀 길이 대비 실제 길이 비율 계산
    X = (X_max / PIXEL_WIDTH) * dx_camera
    Y = (Y_max / PIXEL_HEIGHT) * dy_camera

    return X, Y


# 테스트: 픽셀 좌표 -> 실제 좌표 출력
test_pixels = [(640, 360),  # 카메라 중심점
               (0, 0),      # 좌상단
               (1280, 720), # 우하단
               (960, 540)]  # 중심에서 약간 떨어진 점

for px, py in test_pixels:
    real_x, real_y = pixel_to_real_position(px, py)
    print(f"Pixel ({px}, {py}) -> Real Position (X={real_x:.1f} mm, Y={real_y:.1f} mm)")
