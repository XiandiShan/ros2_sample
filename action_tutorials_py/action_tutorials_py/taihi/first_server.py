import time
from action_tutorials_interfaces.action import Fibonacci
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
# from action_tutorials_py.first_server import main


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
		# time.sleep(5)
		goal_handle.succeed()
		result = Fibonacci.Result()
		self.get_logger().info('Result: {0}'.format(result.sequence))
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
