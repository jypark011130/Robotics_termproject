import numpy as np
from numpy import cos, sin, pi

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import sympy as sp
from sympy import *
import numpy as np
import math

mcos = math.cos
msin = math.sin
matan2 = math.atan2
msqrt = math.sqrt
mpi = math.pi

# list of DH parameters
# d, theta, a, alpha

DOF = 7

th1, th2, th3, th4, th5, th6 = sp.symbols('th1 th2 th3 th4 th5 th6')

# Value of pi from the symbolic library for convenience
spi = sp.pi

# Define DH table

DH_params = []

DH_params.append([83, th1, 0, 0])
DH_params.append([0, th2 + spi/2, 0, spi/2])
DH_params.append([0, th3, 97.5, 0])
DH_params.append([0, th4, 94, 0])
DH_params.append([0, th5 + spi/2, 68, spi/2])
DH_params.append([61, th6 + spi /2, 0, spi/2])
DH_params.append([46, 0, 0, 0])


# Using the combined DH transformation matrix
def DH_trans_matrix(params):
    d, theta, a, alpha = (params[0], params[1], params[2], params[3])

    T = Matrix([[cos(theta), -sin(theta), 0, a],
                [sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -d * sin(alpha)],
                [sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), d * cos(alpha)],
                [0, 0, 0, 1]])
    return T


# Get the transformations from the origin to each of the joints and the end effector
def joint_transforms(DH_params):
    transforms = []

    transforms.append(sp.eye(4))  # Assuming the first first joint is at the origin

    for el in DH_params:
        transforms.append(DH_trans_matrix(el))

    return transforms


# To get the jacobain we can use the cross product method since we have all of the transformations

# Get the total transformation to the end effector
# This function gives the symbolic expression for the jacobian
def jacobian_expr(DH_params):
    transforms = joint_transforms(DH_params)

    trans_EF = transforms[0]

    for mat in transforms[1:]:
        trans_EF = trans_EF * mat

    pos_EF = trans_EF[0:3, 3]

    J = sp.zeros(6, DOF)

    for joint in range(DOF):

        trans_joint = transforms[0]

        for mat in transforms[1:joint + 1]:
            trans_joint = trans_joint * mat

        z_axis = trans_joint[0:3, 2]

        pos_joint = trans_joint[0:3, 3]

        Jv = z_axis.cross(pos_EF - pos_joint)

        Jw = z_axis

        J[0:3, joint] = Jv
        J[3:6, joint] = Jw

    #J = sp.simplify(J)
    return J



# This function evaluates a symbolic jacobian expression using provided joint angles
def jacobian_subs(joints, jacobian_sym):
    # Convert to list if it's an ndarray
    if (isinstance(joints, np.ndarray)):
        joints = joints.flatten().tolist()

    J_l = jacobian_sym

    J_l = J_l.subs(th1, joints[0])
    J_l = J_l.subs(th2, joints[1])
    J_l = J_l.subs(th3, joints[2])
    J_l = J_l.subs(th4, joints[3])
    J_l = J_l.subs(th5, joints[4])
    J_l = J_l.subs(th6, joints[5])

    return J_l


def trans_EF_eval(joints, DH_params):
    # Convert to list if it's an ndarray
    if (isinstance(joints, np.ndarray)):
        joints = joints.flatten().tolist()

    transforms = joint_transforms(DH_params)

    trans_EF = transforms[0]

    for mat in transforms[1:]:
        trans_EF = trans_EF * mat

    trans_EF_cur = trans_EF

    trans_EF_cur = trans_EF_cur.subs(th1, joints[0])
    trans_EF_cur = trans_EF_cur.subs(th2, joints[1])
    trans_EF_cur = trans_EF_cur.subs(th3, joints[2])
    trans_EF_cur = trans_EF_cur.subs(th4, joints[3])
    trans_EF_cur = trans_EF_cur.subs(th5, joints[4])
    trans_EF_cur = trans_EF_cur.subs(th6, joints[5])

    return trans_EF_cur

# ───── 빠른 NumPy용 Jacobian / FK 함수 미리 생성 ─────
print("Building fast NumPy Jacobian / FK (one-time)")
jacobian_symbolic = jacobian_expr(DH_params)
J_func = sp.lambdify((th1, th2, th3, th4, th5, th6), jacobian_symbolic, "numpy")

fk_symbolic = trans_EF_eval([th1, th2, th3, th4, th5, th6], DH_params)
FK_func = sp.lambdify((th1, th2, th3, th4, th5, th6), fk_symbolic, "numpy")
# ───────────────────────────────────────────────────


# This is just for visualizing the robot

def plot_pose(joints, DH_params):
    # Convert to list if it's an ndarray
    if (isinstance(joints, np.ndarray)):
        joints = joints.flatten().tolist()

    transforms = joint_transforms(DH_params)

    trans_EF = trans_EF_eval(joints, DH_params)

    pos_EF = trans_EF[0:3, 3]

    xs = []
    ys = []
    zs = []

    J = sp.zeros(6, DOF)

    for joint in range(DOF):

        trans_joint = transforms[0]

        for mat in transforms[1:joint + 1]:
            trans_joint = trans_joint * mat

        pos_joint = trans_joint[0:3, 3]

        pos_joint = pos_joint.subs(th1, joints[0])
        pos_joint = pos_joint.subs(th2, joints[1])
        pos_joint = pos_joint.subs(th3, joints[2])
        pos_joint = pos_joint.subs(th4, joints[3])
        pos_joint = pos_joint.subs(th5, joints[4])
        pos_joint = pos_joint.subs(th6, joints[5])

        xs.append(pos_joint[0])
        ys.append(pos_joint[1])
        zs.append(pos_joint[2])

    xs.append(pos_EF[0])
    ys.append(pos_EF[1])
    zs.append(pos_EF[2])

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlim3d(-500, 500)
    ax.set_ylim3d(-500, 500)
    ax.set_zlim3d(0, 500)

    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
#선 스타일 등은 그냥 없애도 됨
    ax.plot(xs, ys, zs,
            linestyle='-',  # 선 스타일
            marker='o',  # 원형 마커
            markersize=5,  # 마커 크기
            markerfacecolor='white',  # 마커 안쪽 색
            markeredgecolor='red',  # 마커 테두리 색
            linewidth=2)  # 선 두께
    plt.show()


# joints_init is the current joint values for the robot
# target is the desired transformation matrix at the end effector
# set no_rotation to true if you only care about end effector position, not rotation
# set joint_lims to false if you want to allow the robot to ignore joint limits
# This is currently super slow since it's using all symbolic math
########################################################################################
deg = np.array([0.00, 11.26, 20.00, -31.26, 20.00, 20.00])
########################################################################################
rads = np.radians(deg)

# 2) 6×1 형태로 바꿔주기 (plot_pose가 flatten() → list() 처리하므로 1×6, 6×1 둘 다 OK)
joints = rads.reshape((6,1))

# 3) plot_pose 호출
plot_pose(joints, DH_params)

def get_end_effector_position(joints, DH_params):
    """
    joints: 6×1 또는 길이6의 1D array (각 θ₁…θ₆, rad)
    DH_params: 위에서 정의한 DH 파라미터 리스트
    return: numpy array [x, y, z]
    """
    # 1) joints를 리스트로 변환
    if isinstance(joints, np.ndarray):
        joints = joints.flatten().tolist()
    # 2) 심볼릭 트랜스폼 구하기
    transforms = joint_transforms(DH_params)  # sympy matrices list
    T = transforms[0]
    for mat in transforms[1:]:
        T = T * mat
    # 3) θ들 대입(substitute)
    subs_dict = {
        th1: joints[0],
        th2: joints[1],
        th3: joints[2],
        th4: joints[3],
        th5: joints[4],
        th6: joints[5]
    }
    T_end = T.subs(subs_dict)
    # 4) 숫자(float)로 변환 후 numpy array로
    x = float(T_end[0, 3])
    y = float(T_end[1, 3])
    z = float(T_end[2, 3])
    return np.array([x, y, z])

rads = np.radians(deg).reshape((6,1))
pos = get_end_effector_position(rads, DH_params)
print("End-effector position:", pos)  # [x, y, z]



# 필요하면 도(degree)로도 보기

print("degrees:", deg)