import rclpy
from rclpy.node import Node
from hw2_msg.msg import TrajectoryCommand
import math

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher = self.create_publisher(
            TrajectoryCommand,
            '/hw2_q1_ctrl_msg',
            10
        )

        # 用一次性 timer，发完就停
        self.timer = self.create_timer(0.5, self.send_once)
        self.sent = False

    def send_once(self):
        if self.sent:
            return

        msg = TrajectoryCommand()

        msg.start_position = [-100.0, 200.0, 50.0]   # 起始位置 (mm)
        msg.end_position   = [100.0, 200.0, 50.0]   # 结束位置 (mm)

        msg.start_euler_xyz = [-math.pi/2, 0.0, 0.0]       # 起始欧拉角 (rad)
        msg.end_euler_xyz   = [-math.pi/2, 0.0, 0.0]       # 结束欧拉角 (rad)
        
        # 速度设为0（从静止开始，到静止结束）
        msg.v_start_position = [0.0, 0.0, 0.0]
        msg.v_end_position   = [0.0, 0.0, 0.0]
        msg.w_start_euler    = [0.0, 0.0, 0.0]
        msg.w_end_euler      = [0.0, 0.0, 0.0]
        
        # 持续时间（秒）和控制频率（Hz）
        msg.duration  = 10.0    # 10秒完成
        msg.frequency = 50.0  # 50Hz控制频率

        self.publisher.publish(msg)
        self.get_logger().info("TrajectoryCommand sent once (with v/w)")

        self.sent = True


def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
