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
from demo_nodes_py.services.add_two_ints_client import AddTwoIntsClient



class FirstServer(Node):

    def __init__(self):
        super().__init__('first_fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'first_fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # feedback_msg = Fibonacci.Feedback()
        # feedback_msg.partial_sequence = [0, 1]

        # for i in range(1, goal_handle.request.order):
        #     # self.add_two_ints_client.call_add_two_ints()
        #     feedback_msg.partial_sequence.append(
        #         feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
        #     self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
        #     goal_handle.publish_feedback(feedback_msg)
        #     time.sleep(1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        # result.sequence = feedback_msg.partial_sequence
        return result

def main(args=None):
    rclpy.init(args=args)

    node = FirstServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
