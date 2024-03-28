# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import sys

from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from executor_test.call_service_from_action.action_client import FibonacciActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class AddTwoIntsServer(Node):

    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback, callback_group=ReentrantCallbackGroup())
        self.action_client = FibonacciActionClient()

    def add_two_ints_callback(self, request, response):
        future = self.action_client.send_goal(10)
        self.action_client.get_result(future)

        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args)

    node = AddTwoIntsServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    client_executor = MultiThreadedExecutor()
    client_executor.add_node(node.action_client)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
