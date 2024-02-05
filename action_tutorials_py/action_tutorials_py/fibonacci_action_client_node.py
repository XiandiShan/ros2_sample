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
from rclpy.executors import MultiThreadedExecutor

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class FibonacciActionClientNode(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
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

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))



def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = FibonacciActionClientNode()
    executor.add_node(node)
    node.get_result(node.send_goal(10))

    try:
        executor.spin()
    except Exception as e:
        node.get_logger().info('Exception in fibonacci_action_client: {}'.format(e))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()