import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray


try:
    from hw2_msg.msg import TrajectoryCommand
    print("successful!")
except ImportError as e:
    print(f"import failure: {e}")
    raise e 
try:
    from hw2.save_trajectory_to_file import save_trajectory_to_file
    print("successful!")
except ImportError:
    try:
        from save_trajectory_to_file import save_trajectory_to_file
        print("successful!")
    except ImportError as e:
        raise e
try:
    from hw2.SOARM101 import SOARM101
    print("successful!")
except ImportError:
    try:
        from SOARM101 import SOARM101
    except ImportError as e:
        raise e
        

class CubicTrajectoryGenerator(Node):

    def euler_to_quaternion(self, euler):
        """
        Convert Euler angles (XYZ order) to quaternion
        """
        rx, ry, rz = euler
        # Calculate trigonometric functions
        cos_rx2 = np.cos(rx / 2)
        sin_rx2 = np.sin(rx / 2)
        cos_ry2 = np.cos(ry / 2)
        sin_ry2 = np.sin(ry / 2)
        cos_rz2 = np.cos(rz / 2)
        sin_rz2 = np.sin(rz / 2)
        # Calculate quaternion components
        x = sin_rx2 * cos_ry2 * cos_rz2 - cos_rx2 * sin_ry2 * sin_rz2
        y = cos_rx2 * sin_ry2 * cos_rz2 + sin_rx2 * cos_ry2 * sin_rz2
        z = cos_rx2 * cos_ry2 * sin_rz2 - sin_rx2 * sin_ry2 * cos_rz2
        w = cos_rx2 * cos_ry2 * cos_rz2 + sin_rx2 * sin_ry2 * sin_rz2
        return np.array([x, y, z, w])
    def quaternion_to_euler(self, q):
        """
        Convert quaternion to Euler angles (XYZ order)
        """
        x, y, z, w = q
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp) # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return np.array([roll, pitch, yaw])
    def quaternion_multiply(self, q1, q2):
        """
        Multiply two quaternions
        """
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        return np.array([x, y, z, w])
    def quaternion_conjugate(self, q):
        """
        Calculate conjugate of quaternion
        """
        x, y, z, w = q
        return np.array([-x, -y, -z, w])
    def quaternion_to_axis_angle(self, q):
        """
        Convert quaternion to axis-angle representation
        """
        x, y, z, w = q
        # Normalize quaternion
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        if norm == 0:
            return 0, np.array([1, 0, 0])
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        # Calculate angle
        angle = 2 * np.arccos(w)
        # Calculate axis
        if angle < 1e-6:
            axis = np.array([1, 0, 0])
        else:
            s = np.sqrt(1 - w*w)
            if s < 1e-6:
                axis = np.array([1, 0, 0])
            else:
                axis = np.array([x/s, y/s, z/s])
        return angle, axis

    def __init__(self):

        super().__init__('cubic_trajectory_generator')
        # initialize robot
        self.arm = SOARM101()
        self.sid = "1155248691"
        # create subscription
        self.subscription = self.create_subscription(
            TrajectoryCommand,
            f'/hw2_q1_ctrl_msg',
            self.trajectory_command_callback,
            10)

        # create publisher
        self.publisher = self.create_publisher(
            Float64MultiArray,
            f'/joint_angles_command', 10) #hw2_q2_joint_cmd_{self.sid}

        # save trajectory
        self.trajectory_points = []
        self.current_trajectory = None
        self.timer = None
        self.current_index = 0
        self.frequency = 1.0 # default


    def trajectory_command_callback(self, msg):
        """excute trajectory command"""
        self.get_logger().info(f'Received trajectory command with duration: {msg.duration}s, frequency: {msg.frequency}Hz')
        # form task space trajectory
        task_space_trajectory = self.generate_task_space_trajectory(msg)
        # transform to joint space
        joint_space_trajectory = self.convert_to_joint_space(task_space_trajectory)
        # save the trajectory
        self.save_trajectory(joint_space_trajectory)
        # publish
        self.start_trajectory_publishing(joint_space_trajectory, msg.frequency)
    

    def generate_task_space_trajectory(self, msg):
        """
        Using AI to improve task space trajectory generation
        Ensure pose continuity
        """
        trajectory = []
        # Convert Euler angles to quaternion for interpolation
        start_euler = np.array(msg.start_euler_xyz)
        end_euler = np.array(msg.end_euler_xyz)
        # Euler angles to quaternion
        start_quat = self.euler_to_quaternion(start_euler)
        end_quat = self.euler_to_quaternion(end_euler)
        # Time parameters
        duration = msg.duration
        frequency = msg.frequency
        dt = 1.0 / frequency
        n_steps = int(duration * frequency)
        for step in range(n_steps + 1):
            t = step * dt
            tau = t / duration if duration > 0 else 0
            # Position interpolation (cubic polynomial)
            pos_traj = []
            for i in range(3):
                start = msg.start_position[i]
                end = msg.end_position[i]
                v0 = msg.v_start_position[i]
                v1 = msg.v_end_position[i]
                a0 = start
                a1 = v0
                a2 = (3*(end-start) - (2*v0 + v1)*duration) / (duration**2) if duration > 0 else 0
                a3 = (2*(start-end) + (v0 + v1)*duration) / (duration**3) if duration > 0 else 0
                pos = a0 + a1*t + a2*(t**2) + a3*(t**3)
                pos_traj.append(pos)
            # Orientation interpolation (quaternion SLERP)
            if np.dot(start_quat, end_quat) < 0:
                end_quat = -end_quat # Ensure shortest path
            dot = np.clip(np.dot(start_quat, end_quat), -1, 1)
            theta = np.arccos(dot)
            if theta < 1e-6:
                quat = start_quat
            else:
                quat = (np.sin((1-tau)*theta) * start_quat +
                       np.sin(tau*theta) * end_quat) / np.sin(theta)
            # Quaternion to Euler angles
            euler = self.quaternion_to_euler(quat)
            trajectory.append({
                'time': t,
                'position': np.array(pos_traj),
                'euler': np.array(euler),
                'quaternion': quat # Save quaternion for IK
            })
        return trajectory

    def convert_to_joint_space(self, task_space_trajectory):
        joint_trajectory = []
        last_q = np.array([0.45, 1.41, -1.51, -1.50, -0.17, 0.07])


        def unwrap_to_prev(prev, curr):
            curr = np.array(curr, dtype=float)
            curr = (curr + np.pi) % (2*np.pi) - np.pi
            if prev is None:
                return curr
            prev = np.array(prev, dtype=float)
            return prev + ((curr - prev + np.pi) % (2*np.pi) - np.pi)


        for point in task_space_trajectory:
            position_mm = point["position"]
            euler = point["euler"]

            try:
                q = self.arm.inverse_kinematics_numerical(
                    euler_angles=euler,
                    position_mm=position_mm,
                    q0=last_q,
                    max_iter=500,     # 稍微给多点迭代，减少“收敛到另一支”
                    tol=1e-4,         # 别太苛刻，苛刻更容易乱跳
                    alpha=0.2,       # 步子小一点更稳
                    damping=2e-2,   # 阻尼大一点更不容易发散/跳支
                )

                q = unwrap_to_prev(last_q, q)
                joint_trajectory.append(q.tolist())
                last_q = q

            except Exception as e:
                self.get_logger().error(f"IK failed at {position_mm}: {e}")


        return joint_trajectory

    def save_trajectory(self, joint_trajectory):

        if len(joint_trajectory) == 0:
            return
        # trajectory_by_joint[joint_index] = [angle1, angle2, ...]
        trajectory_by_joint = [[] for _ in range(6)]
        for angles in joint_trajectory:
            for i in range(6):
                trajectory_by_joint[i].append(angles[i])

        save_trajectory_to_file(trajectory_by_joint)
        self.get_logger().info(f'Saved trajectory with {len(joint_trajectory)} points')
    def start_trajectory_publishing(self, joint_trajectory, frequency):

        if self.timer is not None:
            self.timer.cancel()
        self.current_trajectory = joint_trajectory
        self.current_index = 0
        self.frequency = frequency

        period = 1.0 / frequency
        self.timer = self.create_timer(period, self.publish_next_point)
        self.get_logger().info(f'Starting trajectory publishing at {frequency}Hz')
    def publish_next_point(self):

        if (self.current_trajectory is None or
            self.current_index >= len(self.current_trajectory)):

            if self.timer is not None:
                self.timer.cancel()
                self.timer = None
            self.get_logger().info('Trajectory publishing completed')
            return

        joint_angles = self.current_trajectory[self.current_index]

        msg = Float64MultiArray()
        msg.data = [float(angle) for angle in joint_angles]
        self.publisher.publish(msg)

        self.current_index += 1
        # show the progress
        if self.current_index % 10 == 0:
            progress = (self.current_index / len(self.current_trajectory)) * 100
            self.get_logger().info(f'Published point {self.current_index}/{len(self.current_trajectory)} ({progress:.1f}%)')

def main(args=None):

    import sys
    if '--help' in sys.argv or '-h' in sys.argv:
        sys.exit(0)

    try:
        rclpy.init(args=args)

    except Exception as e:

        return

    try:
        node = CubicTrajectoryGenerator()

    except Exception as e:

        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':

    main()


