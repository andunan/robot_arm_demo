import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
import numpy as np

from hw2.SOARM101 import SOARM101  # ✅ 用 package 导入


def quat_to_intrinsic_euler_xyz(q):
    """
    quaternion [x,y,z,w] -> intrinsic Euler XYZ (rad)
    (实现方式：先转 R，再按 XYZ 解)
    """
    x, y, z, w = q
    R = np.array(
        [
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y],
        ],
        dtype=float,
    )

    # intrinsic XYZ == fixed-axis ZYX extraction
    # Solve for rx, ry, rz from R = Rz*Ry*Rx
    sy = np.clip(R[0, 2], -1.0, 1.0)
    ry = np.arcsin(sy)
    if abs(np.cos(ry)) < 1e-8:
        # gimbal-ish: pick rz=0
        rx = np.arctan2(-R[1, 0], R[1, 1])
        rz = 0.0
    else:
        rx = np.arctan2(-R[1, 2], R[2, 2])
        rz = np.arctan2(-R[0, 1], R[0, 0])

    return np.array([rx, ry, rz], dtype=float)


class IKSubscriber(Node):
    def __init__(self):
        super().__init__("ik_subscriber")
        sid = "1155248691"

        self.subscription = self.create_subscription(
            TransformStamped,
            f"/hw2_q1_tf2_{sid}",
            self.listener_callback,
            10,
        )
        self.publisher_analytical = self.create_publisher(
            Float64MultiArray, f"/hw2_q1_ana_{sid}", 10
        )
        self.publisher_numerical = self.create_publisher(
            Float64MultiArray, f"/hw2_q1_num_{sid}", 10
        )

        self.arm = SOARM101()
        self.last_num_sol = None  # ✅ 用上一帧做初值，提高稳定性/连续性

    def listener_callback(self, msg: TransformStamped):
        q_msg = msg.transform.rotation
        p_msg = msg.transform.translation

        q = [float(q_msg.x), float(q_msg.y), float(q_msg.z), float(q_msg.w)]
        p = [float(p_msg.x), float(p_msg.y), float(p_msg.z)]  # ✅ mm

        euler = quat_to_intrinsic_euler_xyz(q)

        # ---- numerical IK (recommended) ----
        try:
            angles_num = self.arm.inverse_kinematics_numerical(
                euler_angles=euler,
                position_mm=p,
                q0=self.last_num_sol,
            )
            self.last_num_sol = angles_num
        except Exception as e:
            self.get_logger().error(f"Numerical IK failed: {e}")
            angles_num = [0.0] * 6

        self.publisher_numerical.publish(Float64MultiArray(data=angles_num))

        # ---- analytical IK (if your analytical function exists & expects euler+pos) ----
        # 你现在 analytical 的实现里也假设了 euler+position，所以这里也传 euler+p
        try:
            angles_ana = self.arm.inverse_kinematics_analytical(euler, p)
        except Exception as e:
            self.get_logger().warn(f"Analytical IK failed: {e}")
            angles_ana = [0.0] * 6

        self.publisher_analytical.publish(Float64MultiArray(data=angles_ana))


def main(args=None):
    rclpy.init(args=args)
    node = IKSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
