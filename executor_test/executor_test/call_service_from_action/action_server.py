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
# limitations under the License.import time

from action_tutorials_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from executor_test.call_service_from_action.service_client import AddTwoIntsClient


class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup())

        self.add_two_ints_client_1 = AddTwoIntsClient()
        self.add_two_ints_client_2 = AddTwoIntsClient()

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        self.add_two_ints_client_1.call_add_two_ints()
        self.add_two_ints_client_2.call_add_two_ints()

        goal_handle.succeed()

        result = Fibonacci.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    node = FibonacciActionServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    # Set executor for client node used in this node
    client_executor_1 = rclpy.executors.MultiThreadedExecutor()
    client_executor_1.add_node(node.add_two_ints_client_1)

    # Set executor for client node used in this node
    client_executor_2 = rclpy.executors.MultiThreadedExecutor()
    client_executor_2.add_node(node.add_two_ints_client_2)


    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        executor.shutdown()


if __name__ == '__main__':
    main()
