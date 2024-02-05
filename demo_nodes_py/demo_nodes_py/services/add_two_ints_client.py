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

from example_interfaces.srv import AddTwoInts

import rclpy

class AddTwoIntsClient:
    def __init__(self, node):
        self.node = node
        self.client = self.node.create_client(AddTwoInts, 'add_two_ints', callback_group=self.node.callback_group)

    def call_add_two_ints(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        req = AddTwoInts.Request()
        req.a = 2
        req.b = 3
        future = self.client.call_async(req)
        # self.node.executor.spin_until_future_complete(future)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            self.node.get_logger().info('Result of add_two_ints: %d' % future.result().sum)
            res = AddTwoInts.Response()
            res.sum = future.result().sum
            return res 
        else:
            self.node.get_logger().error('Exception while calling service: %r' % future.exception())
            return None

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('add_two_ints_client')

    cli = node.create_client(AddTwoInts, 'add_two_ints')
    while not cli.wait_for_service(timeout_sec=1.0):
        print('service not available, waiting again...')
    req = AddTwoInts.Request()
    req.a = 2
    req.b = 3
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info('Result of add_two_ints: %d' % future.result().sum)
    else:
        node.get_logger().error('Exception while calling service: %r' % future.exception())

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
