import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import ParamVec


class TaskInfo(Node):

    def __init__(self):
        super().__init__('task_info')
        self.subscription = self.create_subscription(
            ParamVec,
            '/vrx/task/info',
            self.listener_callback,
            10,
        )

    def listener_callback(self, msg):
        for param in msg.params:
            if param.name == 'state':
                value = param.value.string_value
                print(f'State: {value}')
                if value == 'finished':
                    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    task_info = TaskInfo()
    rclpy.spin(task_info)
    task_info.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
