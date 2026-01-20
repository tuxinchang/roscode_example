import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from charge_interfaces.action import Charge


class ChargeActionClient(Node):
    def __init__(self):
        super().__init__('charge_action_client')

        # 目标电量
        self.declare_parameter('target_percent', 100)
        self.target = int(self.get_parameter('target_percent').value)

        # 可选：自动取消阈值（-1 表示不自动取消）
        self.declare_parameter('cancel_at_percent', -1)
        self.cancel_at = int(self.get_parameter('cancel_at_percent').value)

        self._client = ActionClient(self, Charge, 'charge')
        self._goal_handle = None
        self._canceled = False

    def send_goal(self):
        self.get_logger().info('Waiting for /charge action server...')
        self._client.wait_for_server()

        goal = Charge.Goal()
        goal.target_percent = int(self.target)

        self.get_logger().info(f'Sending goal: target_percent={self.target}')
        future = self._client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted.')
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, msg):
        fb = msg.feedback
        self.get_logger().info(f'[Feedback] current={fb.current_percent}% progress={fb.progress*100:.1f}%')

        # 自动取消：演示 Cancel（可选）
        if self.cancel_at >= 0 and (not self._canceled) and fb.current_percent >= self.cancel_at:
            self._canceled = True
            self.get_logger().warn(f'Auto cancel at {fb.current_percent}% ...')
            self._goal_handle.cancel_goal_async()

    def result_cb(self, future):
        res = future.result().result
        self.get_logger().info(
            f'[Result] success={res.success}, final={res.final_percent}%, '
            f'elapsed={res.elapsed_seconds:.2f}s, msg="{res.message}"'
        )
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ChargeActionClient()
    node.send_goal()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
