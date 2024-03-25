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
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class FirstClient:

    def __init__(self, node):
        self.node = node
        self._action_client = ActionClient(self.node, Fibonacci, 'first_fibonacci', callback_group=ReentrantCallbackGroup())

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        self.execution_finished = False

        self._action_client.wait_for_server()

        future = self._action_client.send_goal_async(
            goal_msg)
        
        rclpy.spin_until_future_complete(self.node, future, MultiThreadedExecutor())

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal rejected :(')
            return
        self.node.get_logger().info('Goal accepted :)')

        future = goal_handle.get_result_async()
        return future
    
    def get_result(self, future):
        rclpy.spin_until_future_complete(self.node, future, MultiThreadedExecutor())
        result = future.result().result
        self.node.get_logger().info('Result: {0}'.format(result.sequence))


# def main(args=None):
#     rclpy.init(args=args)

#     action_client = FibonacciActionClient()

#     executor = rclpy.executors.MultiThreadedExecutor()
#     executor.add_node(action_client)
#     future = action_client.send_goal(10)
#     action_client.get_result(future)

#     executor.spin()



# if __name__ == '__main__':
#     main()