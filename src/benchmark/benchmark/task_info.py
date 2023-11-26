import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32

class TaskInfo(Node):

    def __init__(self):
        super().__init__('task_info')
        self.subscription = self.create_subscription(
            UInt32,
            '/vrx/patrolandfollow/current_phase',
            self.listener_callback,
            10,
        )

    def listener_callback(self, msg):
        phase = msg.data
        print(f'Current phase: {phase}')
        if phase == 2:
            print('TASK SUCCESS')
            exit(0)

def main(args=None):
    rclpy.init(args=args)
    task_info = TaskInfo()
    rclpy.spin(task_info)
    task_info.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
