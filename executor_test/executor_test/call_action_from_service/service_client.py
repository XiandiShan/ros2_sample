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
from rclpy.node import Node
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup



class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        self.rate = self.create_rate(1)

    def call_add_two_ints(self):

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        req = AddTwoInts.Request()
        req.a = 2
        req.b = 3
        future = self.client.call_async(req)
        self.executor.spin_until_future_complete(future)
        if future.result() is not None:
            self.get_logger().info('Result of add_two_ints: %d' % future.result().sum)
            res = AddTwoInts.Response()
            res.sum = future.result().sum
            return res
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())
            return None
        
    def execute(self):
        while rclpy.ok():
            self.call_add_two_ints()
            self.get_logger().info('Sleeping for 1 second')
            self.rate.sleep()
        
def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.create_task(node.execute)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()