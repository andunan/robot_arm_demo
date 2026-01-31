import numpy as np  # 解释: 导入 NumPy，用于数值计算和矩阵运算


class SOARM101:  # 解释: 定义机器人学类 SOARM101，包含正/逆运动学等方法
    """
    FK / IK for SOARM101
    units:
      - angles: radians
      - lengths: millimeters (mm)
    """  # 解释: 类的文档字符串，说明角度单位为弧度，长度单位为毫米

    def __init__(self):
        self.angles = [0, 0, 0, 0, 0, 0]
        self.rotation_axis = [2, 0, 0, 0, 1, 0]

        # 单位统一转化为mm
        self.link_offsets = [
            [0.0, 0.0, 0.0],  # 解释: 序号0的连杆偏移（基座到第1关节的平移）
            [0.0, 31.15, 119.7],  # 解释: 第1->第2连杆偏移（mm）
            [0.0, 112.35, -28.0],  # 解释: 第2->第3连杆偏移
            [0.0, 134.90, 4.85],  # 解释: 第3->第4连杆偏移
            [0.0, 54.80, 0.0],  # 解释: 第4->第5连杆偏移
            [0.0, 31.50, 20.0],  # 解释: 第5->第6连杆偏移（末端前的偏移）
        ]
        self.grab_position_offset = [0.0, 76.0, -11.5]

        # joint limits (rad)
        self.joint_limits = [
            (np.deg2rad(-30), np.deg2rad(30)),
            (np.deg2rad(45), np.deg2rad(90)),
            (np.deg2rad(-120), np.deg2rad(-90)),
            (np.deg2rad(-90), np.deg2rad(-60)),
            (np.deg2rad(-45), np.deg2rad(45)),
            (np.deg2rad(-20), np.deg2rad(10)),
        ]

    def transformation_matrix(self, angle, axis, offset):
        if axis == 0:  # x-axis
            return np.array(
                [
                    [1, 0, 0, offset[0]],  # 解释: 绕 x 轴的齐次变换矩阵（第一行）
                    [0, np.cos(angle), -np.sin(angle), offset[1]],  # 解释: 第二行
                    [0, np.sin(angle), np.cos(angle), offset[2]],  # 解释: 第三行
                    [0, 0, 0, 1],  # 解释: 齐次坐标的第四行
                ],
                dtype=float,
            )
        if axis == 1:  # y-axis
            return np.array(
                [
                    [np.cos(angle), 0, np.sin(angle), offset[0]],  # 解释: 绕 y 轴旋转矩阵行1
                    [0, 1, 0, offset[1]],  # 解释: 行2（y轴保持不变）
                    [-np.sin(angle), 0, np.cos(angle), offset[2]],  # 解释: 行3
                    [0, 0, 0, 1],  # 解释: 齐次行
                ],
                dtype=float,
            )
        if axis == 2:  # z-axis
            return np.array(
                [
                    [np.cos(angle), -np.sin(angle), 0, offset[0]],  # 解释: 绕 z 轴旋转矩阵行1
                    [np.sin(angle), np.cos(angle), 0, offset[1]],  # 解释: 行2
                    [0, 0, 1, offset[2]],  # 解释: 行3（z轴保持）
                    [0, 0, 0, 1],  # 解释: 齐次行
                ],
                dtype=float,
            )
        raise ValueError(f"Invalid axis: {axis}")  # 解释: 非法轴值时抛出异常

    def rotation_matrix_to_quaternion(self, R):
        R = np.array(R, dtype=float)  # 解释: 将输入转换为 NumPy 数组
        trace = np.trace(R)  # 解释: 计算旋转矩阵的迹（三个对角元素之和）
        if trace > 0:
            s = 2.0 * np.sqrt(trace + 1.0)  # 解释: 跟随常见数值稳定算法计算四元数
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        else:
            if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                w = (R[2, 1] - R[1, 2]) / s
                x = 0.25 * s
                y = (R[0, 1] + R[1, 0]) / s
                z = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                w = (R[0, 2] - R[2, 0]) / s
                x = (R[0, 1] + R[1, 0]) / s
                y = 0.25 * s
                z = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                w = (R[1, 0] - R[0, 1]) / s
                x = (R[0, 2] + R[2, 0]) / s
                y = (R[1, 2] + R[2, 1]) / s
                z = 0.25 * s
        return np.array([x, y, z, w], dtype=float)  # 解释: 返回四元数 [x,y,z,w] 格式

    def forward_kinematics(self, angles=None):
        if angles is None:
            angles = self.angles  # 解释: 若未给定角度，使用对象内保存的角度
        T = np.eye(4, dtype=float)  # 解释: 初始化为 4x4 单位矩阵（基坐标系）
        for i, a in enumerate(angles):
            T = T @ self.transformation_matrix(a, self.rotation_axis[i], self.link_offsets[i])
            # 解释: 依次右乘每个关节的变换（旋转+平移），构建末端齐次变换
        T = T @ self.transformation_matrix(0.0, 0, self.grab_position_offset)
        # 解释: 最后再应用夹爪偏移（没有额外旋转，仅平移）

        q = self.rotation_matrix_to_quaternion(T[:3, :3])  # 解释: 从 T 的旋转子矩阵得到四元数
        p = T[:3, 3]  # 解释: 提取平移向量（位置，单位 mm）
        return [q, p]  # quaternion [x y z w], position [mm]  # 解释: 返回四元数和位置

    # ---------- helpers for numerical IK ----------
    @staticmethod
    def _wrap_to_pi(x):
        return (x + np.pi) % (2 * np.pi) - np.pi  # 解释: 将角度限制到 (-pi, pi]

    @staticmethod
    def _intrinsic_euler_xyz_to_R(euler):
        # intrinsic XYZ == Rz * Ry * Rx (with fixed-axis matrices)
        rx, ry, rz = euler  # 解释: 拆分内旋顺序的三个欧拉角（按 X, Y, Z）
        Rx = np.array(
            [[1, 0, 0], [0, np.cos(rx), -np.sin(rx)], [0, np.sin(rx), np.cos(rx)]],
            dtype=float,
        )  # 解释: 绕 X 轴的旋转矩阵
        Ry = np.array(
            [[np.cos(ry), 0, np.sin(ry)], [0, 1, 0], [-np.sin(ry), 0, np.cos(ry)]],
            dtype=float,
        )  # 解释: 绕 Y 轴的旋转矩阵
        Rz = np.array(
            [[np.cos(rz), -np.sin(rz), 0], [np.sin(rz), np.cos(rz), 0], [0, 0, 1]],
            dtype=float,
        )  # 解释: 绕 Z 轴的旋转矩阵
        return Rz @ Ry @ Rx  # 解释: 内旋 XYZ 对应复合矩阵 Rz * Ry * Rx

    @staticmethod
    def _quat_to_R(q):
        x, y, z, w = q  # 解释: 拆分四元数分量，格式为 [x,y,z,w]
        return np.array(
            [
                [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
                [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
                [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y],
            ],
            dtype=float,
        )  # 解释: 将四元数转换为 3x3 旋转矩阵（常规公式）

    @staticmethod  # 把函数声明为静态
    def _R_to_rotvec(R):
        # rotation vector (axis*angle), stable enough for IK small steps
        tr = np.trace(R)  # 解释: 旋转矩阵迹
        cos_theta = np.clip((tr - 1.0) / 2.0, -1.0, 1.0)  # 解释: 通过迹计算 cos(theta)，并裁剪数值稳定
        theta = np.arccos(cos_theta)  # 解释: 计算旋转角度 theta
        if theta < 1e-12:
            return np.zeros(3, dtype=float)  # 解释: 若角度接近0，返回零向量避免数值问题
        v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]], dtype=float)
        # 解释: 计算反对称部分，用于得到旋转轴方向（未归一化）
        v = v / (2.0 * np.sin(theta))  # 解释: 归一化为旋转轴单位向量
        return theta * v  # 解释: 返回旋转向量 = 轴 * 角度

    @staticmethod
    def _clamp_to_limits(theta, limits):
        t = np.array(theta, dtype=float).reshape(-1)
        for i, (low, high) in enumerate(limits):
            t[i] = np.clip(t[i], low, high)
        return t

    def inverse_kinematics_numerical(
        self,
        euler_angles,
        position_mm,
        q0=[0.45, 1.41, -1.51, -1.50, -0.17, 0.07],
        max_iter=300,
        tol=1e-5,          # mm + rad 混合误差，这个阈值对 HW2 够用且更稳
        alpha=0.2,
        damping=2e-2,
    ):
        """
        Numerical IK (Damped Least Squares), with seed for continuity.

        Inputs:
          euler_angles: intrinsic Euler XYZ (rad)
          position_mm: [x,y,z] in mm
          q0: initial joint guess (list/np.array length 6). If None -> zeros
        Returns:
          angles (list length 6, rad)
        Raises:
          RuntimeError if not converged
        """
        target_p = np.array(position_mm, dtype=float).reshape(3)  # 解释: 目标位置向量
        R_target = self._intrinsic_euler_xyz_to_R(np.array(euler_angles, dtype=float).reshape(3))
        # 解释: 目标姿态的旋转矩阵（由欧拉角得到）

        theta = np.zeros(6, dtype=float) if q0 is None else np.array(q0, dtype=float).reshape(6)
        # theta = self._clamp_to_limits(theta, self.joint_limits)  # clamp initial guess

        for _ in range(max_iter):  # 解释: 进行最多 max_iter 次迭代
            q_cur, p_cur = self.forward_kinematics(theta.tolist())  # 解释: 计算当前的正运动学（四元数+位置）
            p_cur = np.array(p_cur, dtype=float).reshape(3)  # 解释: 转为 NumPy 向量
            R_cur = self._quat_to_R(q_cur)  # 解释: 当前姿态的旋转矩阵

            e_p = target_p - p_cur  # mm  # 解释: 位置误差向量（mm）
            R_err = R_target @ R_cur.T  # 解释: 目标相对于当前的误差旋转矩阵
            e_R = self._R_to_rotvec(R_err)  # rad  # 解释: 将误差旋转转换为旋转向量（轴角）
            e = np.hstack([e_p, 0.1 * e_R])  # 解释: 组合误差向量，姿态误差缩放0.1以匹配位置尺度

            if np.linalg.norm(e) < tol:
                theta = self._wrap_to_pi(theta)  # 解释: 若误差小于阈值，包装角度到 (-pi,pi] 并返回
                return theta.tolist()

            # numerical Jacobian
            J = np.zeros((6, 6), dtype=float)  # 解释: 初始化数值雅可比矩阵（6x6）
            eps = 1e-6  # 解释: 有限差分的小增量
            for j in range(6):
                t2 = theta.copy()
                t2[j] += eps  # 解释: 对第 j 个关节做微小扰动
                q2, p2 = self.forward_kinematics(t2.tolist())  # 解释: 计算扰动后的正运动学
                p2 = np.array(p2, dtype=float).reshape(3)
                R2 = self._quat_to_R(q2)  # 解释: 扰动后的旋转矩阵

                dp = (p2 - p_cur) / eps  # 解释: 位置对关节的数值导数（列向量）
                dR = R2 @ R_cur.T  # 解释: 扰动旋转相对于当前的旋转差
                drot = self._R_to_rotvec(dR) / eps  # 解释: 姿态的数值导数（旋转向量除以 eps）
                J[:3, j] = dp  # 解释: 将位置导数写入雅可比前3行
                J[3:, j] = drot  # 解释: 将姿态导数写入雅可比后3行

            # DLS solve: dtheta = J^T (J J^T + λ^2 I)^-1 e
            A = J @ J.T + (damping**2) * np.eye(6)  # 解释: 构造带阻尼项的矩阵（提高数值稳定性）
            dtheta = J.T @ np.linalg.solve(A, e)  # 解释: 解阻尼最小二乘问题得到关节增量

            theta = theta + alpha * dtheta  # update
            theta = self._wrap_to_pi(theta)
            # theta = self._clamp_to_limits(theta, self.joint_limits)  # enforce joint limits

        raise RuntimeError("Numerical IK did not converge")  # 解释: 若超出最大迭代仍未收敛则抛出异常

