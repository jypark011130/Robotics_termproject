import numpy as np
from numpy import cos, sin, pi

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import sympy as sp

# list of DH parameters
# d, theta, a, alpha

DOF = 7

th1, th2, th3, th4, th5, th6, th7 = sp.symbols('th1 th2 th3 th4 th5 th6 th7')

# Value of pi from the symbolic library for convenience
spi = sp.pi

# Define DH table

DH_params = []

DH_params.append([83, th1, 0, 0])
DH_params.append([0, th2 + spi/2, 0, spi/2])
DH_params.append([0, th3, 97.5, 0])
DH_params.append([0, th4, 94, 0])
DH_params.append([0, th5 + spi/2, 68, spi/2])
DH_params.append([61, th6+ spi/2, 0, spi/2])
DH_params.append([46, th7, 0, 0])


# Using the combined DH transformation matrix
def DH_trans_matrix(params):
    d, theta, a, alpha = (params[0], params[1], params[2], params[3])

    mat = sp.Matrix(
        [[sp.cos(theta), -sp.sin(theta), 0, a],
         [sp.sin(theta) * sp.cos(alpha), sp.cos(theta) * sp.cos(alpha), -sp.sin(alpha), -d * sp.sin(alpha)],
         [sp.sin(theta) * sp.sin(alpha), sp.cos(theta) * sp.sin(alpha), sp.cos(alpha), d * sp.cos(alpha)],
         [0, 0, 0, 1]])

    return mat


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
    J_l = J_l.subs(th7, joints[6])

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
    trans_EF_cur = trans_EF_cur.subs(th7, joints[6])

    return trans_EF_cur

# ───── 빠른 NumPy용 Jacobian / FK 함수 미리 생성 ─────
print("Building fast NumPy Jacobian / FK (one-time)")
jacobian_symbolic = jacobian_expr(DH_params)
J_func = sp.lambdify((th1, th2, th3, th4, th5, th6, th7), jacobian_symbolic, "numpy")

fk_symbolic = trans_EF_eval([th1, th2, th3, th4, th5, th6, th7], DH_params)
FK_func = sp.lambdify((th1, th2, th3, th4, th5, th6, th7), fk_symbolic, "numpy")
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

def joint_limits(joints):
    # Joint 1
    if (joints[0] < -2 * pi / 3):

        joints[0] = -2 * pi / 3

    elif (joints[0] > 2 * pi / 3):

        joints[0] = 2 * pi / 3

    # Joint 2
    if (joints[1] < -0.95 * pi):

        joints[1] = -0.95 * pi

    elif (joints[1] > 0):

        joints[1] = 0

    # Joint 3
    if (joints[2] < -0.463 * pi):

        joints[2] = -0.463 * pi

    elif (joints[2] > 0.48 * pi):

        joints[2] = 0.48 * pi

    # Joint 4
    if (joints[3] < -0.97 * pi):

        joints[3] = -0.97 * pi

    elif (joints[3] > 0.97 * pi):

        joints[3] = 0.97 * pi

    return joints

    # Joint 5
    if (joints[4] < -3 * pi / 2):

        joints[4] = -3 * pi / 2

    elif (joints[4] > 3 * pi / 2):

        joints[4] = 3 * pi / 2

    # Joint 6
    if (joints[5] < -0.95 * pi):

        joints[5] = -0.95 * pi

    elif (joints[5] > 0.95 * pi):

        joints[5] = 0.95 * pi

    return joints

# joints_init is the current joint values for the robot
# target is the desired transformation matrix at the end effector
# set no_rotation to true if you only care about end effector position, not rotation
# set joint_lims to false if you want to allow the robot to ignore joint limits
# This is currently super slow since it's using all symbolic math
def i_kine(joints_init, target, DH_params, error_trace=True, no_rotation=False, joint_lims=True):
    joints = joints_init

    xr_desired = target[0:3, 0:3]
    xt_desired = target[0:3, 3]

    x_dot_prev = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    e_trace = []

    iters = 0;

    print("Finding symbolic jacobian")

    # We only do this once since it's computationally heavy
    #jacobian_symbolic = jacobian_expr(DH_params)

    print("Starting IK loop")

    final_xt = 0

    while (1):

        flat = joints.flatten()  # 1×6 배열
        jac = J_func(*flat)  # NumPy 6×6
        trans_EF_cur = FK_func(*flat)  # NumPy 4×4

        xr_cur = trans_EF_cur[0:3, 0:3]
        xt_cur = trans_EF_cur[0:3, 3]

        final_xt = xt_cur

        xt_dot = xt_desired - xt_cur

        # Find error rotation matrix
        R = xr_desired @ xr_cur.T

        # convert to desired angular velocity
        v = np.arccos((R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2)
        r = (0.5 * sin(v)) * np.array([[R[2, 1] - R[1, 2]],
                                       [R[0, 2] - R[2, 0]],
                                       [R[1, 0] - R[0, 1]]])

        # The large constant just tells us how much to prioritize rotation
        xr_dot = 200 * r * sin(v)

        # use this if you only care about end effector position and not rotation
        if (no_rotation):
            xr_dot = 0 * r

        xt_dot = xt_dot.reshape((3, 1))

        x_dot = np.vstack((xt_dot, xr_dot))

        x_dot_norm = np.linalg.norm(x_dot)

        # print(x_dot_norm)

        if (x_dot_norm > 25):
            x_dot /= (x_dot_norm / 25)

        x_dot_change = np.linalg.norm(x_dot - x_dot_prev)

        # This loop now exits if the change in the desired movement stops changing
        # This is useful for moving close to unreachable points
        if (x_dot_change < 0.005):
            break;

        x_dot_prev = x_dot

        e_trace.append(x_dot_norm)

        Lambda = 12
        Alpha = 1

        joint_change = Alpha * np.linalg.inv(jac.T @ jac + Lambda ** 2 * np.eye(DOF)) @ jac.T @ x_dot

        joints += joint_change

        #joints[0] =
        #joints[1] =
        # joints[2] =
        # joints[3] =
        # joints[4] =
        # joints[5] =
        joints[6] = 0.0
        if (joint_lims): joints = joint_limits(joints)

        iters += 1

    print("Done in {} iterations".format(iters))

    print("Final position is:")
    print(final_xt)

    return (joints, e_trace) if error_trace else joints

joints = np.array([[0],[0],[0],[0],[0],[0],[0.0]])

target = np.array([[1, 0, 0, 150],
                   [0, -1, 0, 150],
                   [0, 0, -1, 150],
                   [0, 0, 0, 1]])

new_j, e_trace = i_kine(joints, target, DH_params, error_trace=True, no_rotation=True)

plot_pose(new_j, DH_params)

plt.figure(figsize=(8,8))
plt.plot(e_trace)
plt.title('Error Trace')
plt.show()

#세타값은 어떻게 되는걸까?
# IK 돌려서 new_j, e_trace 구한 뒤


# 1×6 행렬 → 1차원 array로 바꾸기
theta_vals = new_j.flatten()

# 언패킹
theta1, theta2, theta3, theta4, theta5, theta6, theta7 = theta_vals

# 라디안으로 출력
print("θ1 =", theta1, "rad")
print("θ2 =", theta2, "rad")
print("θ3 =", theta3, "rad")
print("θ4 =", theta4, "rad")
print("θ5 =", theta5, "rad")
print("θ6 =", theta6, "rad")
print("θ6 =", theta7, "rad")

# 필요하면 도(degree)로도 보기
#deg = np.degrees(theta_vals)
#print("degrees:", theta1/3.141592*180,', ',theta2/3.141592*180,
#      ', ',theta3/3.141592*180,', ',theta4/3.141592*180,', ',theta5/3.141592*180,', ',theta6/3.141592*180)

