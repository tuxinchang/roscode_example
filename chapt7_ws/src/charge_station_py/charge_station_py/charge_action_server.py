import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from charge_interfaces.action import Charge


class ChargeActionServer(Node):
    def __init__(self):
        super().__init__('charge_action_server')

        self.declare_parameter('rate_percent_per_sec', 5)
        self.rate = int(self.get_parameter('rate_percent_per_sec').value)

        self.declare_parameter('policy', 'reject')
        self.policy = str(self.get_parameter('policy').value).strip()

        self._active_goal_handle: Optional[rclpy.action.server.ServerGoalHandle] = None

        # 关键：ReentrantCallbackGroup + MultiThreadedExecutor 才能边执行边响应 cancel/新 goal
        cb_group = ReentrantCallbackGroup()

        self._server = ActionServer(
            self,
            Charge,
            'charge',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,   # 同步函数
            callback_group=cb_group,
        )

        self.get_logger().info(f'Charge Action Server started on /charge | rate={self.rate}%/s | policy={self.policy}')

    def goal_callback(self, goal_request: Charge.Goal) -> GoalResponse:
        tgt = int(goal_request.target_percent)
        if tgt <= 0 or tgt > 100:
            self.get_logger().warn(f'Reject goal: target_percent={tgt} (must be 1..100)')
            return GoalResponse.REJECT

        if self._active_goal_handle is not None:
            if self.policy == 'reject':
                self.get_logger().warn(f'Reject goal {tgt}% because another goal is active.')
                return GoalResponse.REJECT
            elif self.policy == 'preempt':
                self.get_logger().warn('Preempt: abort current goal and accept new goal.')
                try:
                    self._active_goal_handle.abort()
                except Exception as e:
                    self.get_logger().warn(f'Abort active goal failed: {e}')
                self._active_goal_handle = None
                return GoalResponse.ACCEPT
            else:
                self.get_logger().warn(f'Unknown policy={self.policy}, fallback to reject.')
                return GoalResponse.REJECT

        self.get_logger().info(f'Accept goal: target_percent={tgt}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel request received -> ACCEPT')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self._active_goal_handle = goal_handle

        target = int(goal_handle.request.target_percent)
        current = 0

        feedback = Charge.Feedback()
        start_time = self.get_clock().now()

        self.get_logger().info(f'Executing: charge to {target}% ...')

        while rclpy.ok():
            # 取消：立刻退出
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9

                result = Charge.Result()
                result.success = False
                result.final_percent = int(current)
                result.elapsed_seconds = float(elapsed)
                result.message = 'Charging canceled (emergency task).'

                self.get_logger().info('Goal canceled. Clean up resources.')
                self._active_goal_handle = None
                return result

            # 推进充电
            current = min(current + self.rate, target)

            feedback.current_percent = int(current)
            feedback.progress = float(current / target)
            goal_handle.publish_feedback(feedback)

            if current >= target:
                break

            time.sleep(1.0)

        goal_handle.succeed()
        elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9

        result = Charge.Result()
        result.success = True
        result.final_percent = int(current)
        result.elapsed_seconds = float(elapsed)
        result.message = 'Charging completed.'

        self.get_logger().info('Goal succeeded.')
        self._active_goal_handle = None
        return result


def main():
    rclpy.init()
    node = ChargeActionServer()

    # 关键：多线程执行器，避免 execute 循环阻塞 cancel/goal 回调
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

