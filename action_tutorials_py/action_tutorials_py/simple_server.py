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

import time

from action_tutorials_interfaces.action import Fibonacci


import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_tutorials_py.send_goal_and_get_result import FibonacciActionClient



class SimpleServer(Node):

    def __init__(self):
        super().__init__('simple_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'another_fibonacci',
            self.execute_callback)
        
        self.action_client = FibonacciActionClient()

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        future=self.action_client.send_goal(goal_handle.request.order)
        self.action_client.get_result(future)

        goal_handle.succeed()

        result = Fibonacci.Result()
        return result

def main(args=None):
    rclpy.init(args=args)

    simple_server = SimpleServer()

    try:
        rclpy.spin(simple_server)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
