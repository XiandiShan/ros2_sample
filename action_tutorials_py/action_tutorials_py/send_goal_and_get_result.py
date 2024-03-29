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


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        self.execution_finished = False

        self._action_client.wait_for_server()

        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        rclpy.spin_until_future_complete(self, future, MultiThreadedExecutor())

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')

        future = goal_handle.get_result_async()
        return future
    
    def get_result(self, future):
        rclpy.spin_until_future_complete(self, future, MultiThreadedExecutor())
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))

    # def send_goal(self, order):
    #     goal_msg = Fibonacci.Goal()
    #     goal_msg.order = order
    #     self.execution_finished = False

    #     self._action_client.wait_for_server()

    #     self._send_goal_future = self._action_client.send_goal_async(
    #         goal_msg,
    #         feedback_callback=self.feedback_callback)

    #     self._send_goal_future.add_done_callback(self.goal_response_callback)
    #     self.rate = self.create_rate(10)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        self.execution_finished = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

    # def get_result(self):
    #     self.get_logger().info('Waiting for result')
    #     while rclpy.ok():
    #         if self.execution_finished:
    #             break
    #         self.rate.sleep()
    #     self.get_logger().info('Execution finished')
    
    def execute(self):
        future=self.send_goal(10)
        self.get_result(future)

def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(action_client)
    # executor.create_task(action_client.execute)
    future=action_client.send_goal(10)
    action_client.get_result(future)

    executor.spin()



if __name__ == '__main__':
    main()