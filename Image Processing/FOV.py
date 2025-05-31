import math

# 카메라와 영상 정보
PIXEL_WIDTH = 1280
PIXEL_HEIGHT = 720
P_cx = PIXEL_WIDTH / 2
P_cy = PIXEL_HEIGHT / 2

# 카메라 설치 높이(mm) - 실제 환경에 맞게 설정
H_dist = 1000  

# 좌우 FOV 반각도 (degrees)
FOV_half_deg = 27.5
FOV_half_rad = math.radians(FOV_half_deg)

# 영상 기준 X_max (가로 작업 가능 범위 계산)
X_max = 2 * H_dist * math.tan(FOV_half_rad)

# 수직 FOV도 비슷하게 구해봄 (필요하면 직접 측정 또는 명확히 지정)
# 여기서는 단순히 세로 FOV 비율로 계산 예시
aspect_ratio = PIXEL_HEIGHT / PIXEL_WIDTH
FOV_vertical_half_rad = FOV_half_rad * aspect_ratio
Y_max = 2 * H_dist * math.tan(FOV_vertical_half_rad)

# 픽셀 기준 각도 계산 (픽셀당 각도 단위, radians)
angle_per_pixel_x = (2 * FOV_half_rad) / PIXEL_WIDTH
angle_per_pixel_y = (2 * FOV_vertical_half_rad) / PIXEL_HEIGHT

def pixel_to_real_with_fov(Px, Py):
    """
    픽셀 좌표를 카메라 설치 높이와 FOV 고려해 실제 거리(mm)로 변환
    중심 픽셀 기준 좌우/상하 각도로 변환 후 삼각함수로 위치 계산

    Args:
        Px, Py : 픽셀 좌표
    Returns:
        X, Y : 실제 작업 공간 좌표 (mm)
    """
    dx_pixel = Px - P_cx
    dy_pixel = Py - P_cy

    # 각도 offset (radians)
    angle_x = dx_pixel * angle_per_pixel_x
    angle_y = dy_pixel * angle_per_pixel_y

    # 거리 계산: 카메라에서 바닥까지 수직 거리 H_dist 기준으로 각도 활용해 수평 거리 산출
    X = H_dist * math.tan(angle_x)
    Y = H_dist * math.tan(angle_y)

    return X, Y

# 테스트
test_points = [(640, 360), (0, 0), (1280, 720), (960, 540)]
for px, py in test_points:
    real_x, real_y = pixel_to_real_with_fov(px, py)
    print(f"Pixel ({px}, {py}) -> Real (X={real_x:.1f} mm, Y={real_y:.1f} mm)")
