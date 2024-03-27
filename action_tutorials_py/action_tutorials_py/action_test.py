from rclpy.executors import MultiThreadedExecutor
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from .fibonacci_action_client_spin_future import FibonacciActionClientSpinFuture
from demo_nodes_py.services.add_two_ints_client import AddTwoIntsClient

class Ros2ActionTest(Node):

    def __init__(self, node_name):
        super().__init__(node_name)
        self.callback_group = ReentrantCallbackGroup()
        self.fibonacci_action_client = FibonacciActionClientSpinFuture(self)
        self.add_two_ints_client = AddTwoIntsClient(self)
        self.rate = self.create_rate(10)

    def create_thread(self, order):
        # self.add_two_ints_client.call_add_two_ints()
        future = self.fibonacci_action_client.send_goal(order)
        self.fibonacci_action_client.get_result(future)

    def test(self):
         """In this test, we will send a goal to the Fibonacci action server and
         then send a goal to the Fibonacci action server in a separate thread.
         """
         while rclpy.ok():
            future = self.fibonacci_action_client.send_goal(2)
            # send_goal_thread = threading.Thread(target=self.create_thread, args=(10,))
            # send_goal_thread.start()
            # send_goal_thread.join()
            self.fibonacci_action_client.get_result(future)
            self.rate.sleep()


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    node = Ros2ActionTest("ros2_action_test")

    executor.add_node(node)
    executor.create_task(node.test)

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

