import numpy as np

# 카메라와 작업 공간 설정 (이미지 기반)
PIXEL_WIDTH = 1280
PIXEL_HEIGHT = 720

# 실제 작업 공간 최대 거리(mm 단위) - 직접 측정하거나 실험으로 정해야 함
X_max = 122.5  # 예: 가로 작업 가능 최대 122.5mm (양쪽 대칭으로 ±61.25mm라 가정)
Y_max = 487.5  # 예: 세로 작업 가능 최대 487.5mm (양쪽 대칭으로 ±243.75mm라 가정)

# 카메라 중심 픽셀 (이미지 중심)
P_cx = PIXEL_WIDTH / 2
P_cy = PIXEL_HEIGHT / 2

# 로봇 베이스 기준으로 카메라 중심이 x축 방향으로 162mm만큼
# (+ 부호 방향) 떨어져 있다고 가정. (필요에 따라 -162로 조정)
X_offset_mm = 162.0

def pixel_to_real_position(Px, Py):
    """
    카메라 픽셀 좌표 (Px, Py)를 
    로봇 베이스 기준 실제 좌표 (X, Y)로 변환

    Args:
        Px, Py : 픽셀 좌표 (정수 또는 실수)
    Returns:
        X, Y : 실제 작업 공간 좌표 (mm 단위, 로봇 베이스 기준)
    """

    # (1) 픽셀 → 카메라 중심 기준(mm)
    dx_camera = Px - P_cx           # x 픽셀 차이
    dy_camera = Py - P_cy           # y 픽셀 차이

    # 픽셀 길이 대비 실제 길이 비율 계산
    X_cam = (X_max / PIXEL_WIDTH) * dx_camera
    Y_cam = (Y_max / PIXEL_HEIGHT) * dy_camera

    # (2) 카메라 중심 기준(mm) → 로봇 베이스 기준(mm)
    #     로봇 원점 오프셋을 더해 준다.
    X_robot = X_cam + X_offset_mm
    Y_robot = Y_cam

    return X_robot, Y_robot
