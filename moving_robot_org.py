import time
import math
from motorLLC_Protocol_TP_GPT import motorLLC   # motorLLC 클래스가 정의된 파일명으로 바꿔주세요

def rad2pulse(theta_rad: float) -> int:
    # ① 사용-좌표계(−150° ~ +150°) → 기계좌표계(0° ~ 300°)
    mech_deg = math.degrees(theta_rad) + 150.0   # offset +150°

    # ② 허용 범위 밖이면 잘라냄
    mech_deg = max(0.0, min(300.0, mech_deg))

    # ③ 0 ~ 300°  →  0 ~ 1023 펄스
    pulse = int(round(mech_deg * 1023.0 / 300.0))

    return pulse

if __name__ == "__main__":
    # 1) 객체 생성
    motor = motorLLC()

    # 2) 포트 오픈 & Baudrate 설정
    motor.open()

    # 3) 토크(On) 활성화
    motor.torque_enable()
    motor.ids = motor.ids[:7]
    # 4) 목표 위치(positions)와 속도(velocities) 리스트 정의
    #    여기선 예시로 모두 512, 100으로 채웠지만, 원하는 값으로 바꿔주세요
    theta_deg = [-0.00, -16.22, -77.82, 4.04, -0.00, -90.00,-11]
    theta_rad = [angle * math.pi / 180.0 for angle in theta_deg]
    positions  = [rad2pulse(t) for t in theta_rad]
    velocities = [50] * len(motor.ids)   # 100 pulse/s

    # 5) 동작 명령 전송
    motor.moveTo(positions, velocities)

    # 6) 잠깐 대기 (모터가 목표로 이동할 시간)
    time.sleep(1.0)

    # 7) 현재 위치·속도 읽기
    pos = motor.readPos()
    vel = motor.readVelocity()
    print("현재 위치:", pos)
    print("현재 속도:", vel)

    # 8) 토크(Off) 비활성화 & 포트 종료
    #motor.close()
