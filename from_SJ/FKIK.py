from pickletools import read_long1

from sympy import *
import numpy as np
import math

mcos = math.cos
msin = math.sin
matan2 = math.atan2
msqrt = math.sqrt
mpi = math.pi


def DH2TF(alpha, a, theta, d):
    T = Matrix([[cos(theta), -sin(theta), 0, a],
                [sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -d * sin(alpha)],
                [sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), d * cos(alpha)],
                [0, 0, 0, 1]])
    return T


def ForwardKinematics(th, a2, a3, a4, d1, d5, d6):
    th1 = th[0] * mpi / 180;
    th2 = th[1] * mpi / 180;
    th3 = th[2] * mpi / 180;
    th4 = th[3] * mpi / 180;
    th5 = th[4] * mpi / 180;
    th6 = th[5] * mpi / 180;
    th7 = 0
    T01 = DH2TF(0, 0, th1, d1)
    T12 = DH2TF(pi / 2, 0, th2 + pi / 2, 0)
    T23 = DH2TF(0, a2, th3, 0)
    T34 = DH2TF(0, a3, th4, 0)
    T45 = DH2TF(pi / 2, a4, th5 + pi / 2, 0)
    T56 = DH2TF(pi / 2, 0, th6 + pi / 2, d5)
    T67 = DH2TF(0, 0, 0, d6)
    T07 = T01 * T12 * T23 * T34 * T45 * T56 * T67
    return T07


def theta_calculator(T07, a2, a3, a4, d1, d5, d6):
    # 회전부와 위치벡터로 분리
    R = T07[:3, :3]
    P = T07[:3, 3]

    # 각 요소 지정
    r11, r12, r13 = R[0, 0], R[0, 1], R[0, 2]
    r21, r22, r23 = R[1, 0], R[1, 1], R[1, 2]
    r31, r32, r33 = R[2, 0], R[2, 1], R[2, 2]
    px, py, pz = P[0], P[1], P[2]

    print("r13=",r13)
    print("r23=", r23)
    # wrist-center 구하기(tool-offset 제거)
    px_w = px
    py_w = py
    pz_w = pz

    # 1) θ1 계산을 위한 M, N 정의
    M = (d5 + d6) * r13 - px
    N = py - (d5 + d6) * r23

    # 2) 가능한 θ1 해 두 개
    th1_1 = math.atan2(-N, M)
    th1_2 = math.atan2(N, -M)

    # 3) 절대값이 더 작은 θ1을 선택
    th1 = th1_1 if abs(th1_1) < abs(th1_2) else th1_2

    # 4) 선택된 θ1의 sin, cos 계산
    s1 = math.sin(th1)
    c1 = math.cos(th1)

    # 5) s5 = s1 * r13 - c1 * r23 (사진의 K)
    s5 = (-s1*px+c1*py)/(-d5-d6)
    print("s5=",s5)
    # 6) c5 = sqrt(1 - s5^2)
    th5 = math.asin(s5)
    c5 = math.cos(th5)
    print("c5=",c5)
    c234 = r33 / math.cos(th5)
    print("c1*r13+s1*r23=",c1*r13+s1*r23)
    s234 = -(c1*r13+s1*r23)/math.cos(th5)
    th234 = math.atan2(s234,c234)
    print("th234=", th234)
    # 9) s234, c234

    k=c1*px+s1*py
    Q1=a4*s234 + d5*s234*c5 + d6*s234*c5
    Q2=a4*c234 + d5*c234*c5 + d6*c234*c5
    Q = -(k + Q1)
    H = -Q2 + pz
    E = math.sqrt(Q*Q+H*H)
    c3 = -(-E*E+a2*a2+a3*a3)/(2*a2*a3)
    c3 = math.cos(-20*3.1417/180)
    print("c3=", c3)
    s3 = math.sqrt(1-c3*c3)
    th3_1 = math.atan2(s3,c3)
    print('th3_1=', th3_1)
    th3_2 = math.atan2(-s3, c3)
    print('th3_2=', th3_2)

    '''
    D=a4+(d5+d6)*c5
    u=-k-D*s234
    w=pz-D*c234
    r=math.sqrt(u*u+w*w)


    ##############################################
    c3=(r*r-a2*a2-a3*a3)/(2*a2*a3)
    print(c3)
    c3 = max(-1.0, min(1.0, c3))

    ##############################################


    print("분자=", r*r+D*D-(a2*a2+a3*a3))
    print("2*a2*a3=분모=", 2*a2*a3)
    print("c3=", c3)
    th3_1=math.atan2(math.sqrt(1-c3*c3),c3)
    print('th3_1=',th3_1)
    th3_2 = math.atan2(-math.sqrt(1 - c3 * c3), c3)
    print('th3_2=', th3_2)
'''
    abs1 = abs(th3_1)
    abs2 = abs(th3_2)

    if abs1 < abs2:
        th3 = th3_1
    elif abs1 > abs2:
        th3 = th3_2
    else:
        # abs1 == abs2 인 경우엔 양수인 쪽 선택
        th3 = max(th3_1, th3_2)

    print("th3=", th3)

    s3 = math.sin(th3)
    print("s3=", s3)


    al = math.atan2(Q,H)
    print("al=", al)
    be = math.atan2(a3*s3,a2+a3*c3)
    th2 = al - be
    print("be=", be)
    print("th2=", th2)

    # 16) θ4 = θ₂₃₄ − θ2 − θ3
    th4 = th234 - th2 - th3
    print("th4=", th4)
    # 17) θ6 계산
    # sinθ6 = (−s1·r12 + c1·r22) / cosθ5
    s6 = s5 * c234 * r31 - s234 * r32
    print("s6=",s6)
    # cosθ6 = ±√(1−s6²) – 분기 처리 필요
    c6 = s234 * r31 + s5 * c234 * r32
    th6 = math.atan2(s6, c6)

    return th1, th2, th3, th4, th5, th6


if __name__ == "__main__":
    # 테스트용 관절각 (deg)과 파라미터
    th_deg = [0, 40, -20, -20, 20, 20]
    a2, a3, a4 = 97.5, 94, 68
    d1, d5, d6 = 83, 61, 46

    # 1) Forward Kinematics
    T07 = ForwardKinematics(th_deg, a2, a3, a4, d1, d5, d6)
    print(T07)
    # 2) Sympy 행렬을 숫자로 근사(evalf)만 해 줍니다
    T07_num = T07.evalf()

    # 3) Inverse Kinematics
    th1, th2, th3, th4, th5, th6 = theta_calculator(
        T07_num, a2, a3, a4, d1, d5, d6
    )

    # 4) rad → deg 변환
    degs = [math.degrees(th) for th in (th1, th2, th3, th4, th5, th6)]

    # 5) 결과 출력
    print("Recovered θ (rad):", th1, th2, th3, th4, th5, th6)
    print("Recovered θ (deg):",
          f"{degs[0]:.2f}, {degs[1]:.2f}, {degs[2]:.2f}, "
          f"{degs[3]:.2f}, {degs[4]:.2f}, {degs[5]:.2f}")
