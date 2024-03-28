#!/usr/bin/env python3
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from action_tutorials_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci', callback_group=ReentrantCallbackGroup())
        self.rate = self.create_rate(1)

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        self.execution_finished = False

        self._action_client.wait_for_server()

        future = self._action_client.send_goal_async(
            goal_msg)
        self.executor.spin_until_future_complete(future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')

        future = goal_handle.get_result_async()
        return future
    
    def get_result(self, future):

        self.executor.spin_until_future_complete(future)
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))


    def execute(self):
        while rclpy.ok():
            future = self.send_goal(10)
            self.get_result(future)

            # 以下のようなコードを実行すると、AddTwoIntsClientのcallback_groupを指定すべき
            # future_1 = self.send_goal(10)
            # future_2 = self.send_goal(10)
            # self.get_result(future_1)
            # self.get_result(future_2)

            self.get_logger().info('Sleeping for 1 second')
            self.rate.sleep()


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(action_client)
    executor.create_task(action_client.execute)
    executor.spin()


if __name__ == '__main__':
    main()
