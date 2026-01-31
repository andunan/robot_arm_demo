import numpy as np
from hw2.SOARM101 import SOARM101

def quat_to_intrinsic_euler_xyz(q):
    x, y, z, w = q
    R = np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y],
    ], dtype=float)

    sy = np.clip(R[0, 2], -1.0, 1.0)
    ry = np.arcsin(sy)
    if abs(np.cos(ry)) < 1e-8:
        rx = np.arctan2(-R[1, 0], R[1, 1])
        rz = 0.0
    else:
        rx = np.arctan2(-R[1, 2], R[2, 2])
        rz = np.arctan2(-R[0, 1], R[0, 0])
    return np.array([rx, ry, rz], dtype=float)

arm = SOARM101()

np.random.seed(0)
ok = 0
N = 50
for k in range(N):
    q_true = (np.random.rand(6) * 2*np.pi - np.pi).tolist()
    quat, pos = arm.forward_kinematics(q_true)
    euler = quat_to_intrinsic_euler_xyz(quat)

    try:
        q_sol = arm.inverse_kinematics_numerical(euler, pos, q0=q_true)  # 用真值当 seed，更容易收敛
        quat2, pos2 = arm.forward_kinematics(q_sol)
        pos_err = np.linalg.norm(np.array(pos2) - np.array(pos))
        print(f"[{k}] pos_err(mm)={pos_err:.4f}")
        if pos_err < 1.0:  # 1mm 以内就很强了
            ok += 1
    except Exception as e:
        print(f"[{k}] FAIL: {e}")

print(f"PASS {ok}/{N}")
