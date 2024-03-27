import time
from action_tutorials_interfaces.action import Fibonacci
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import MutuallyExclusiveCallbackGroup, Node, ReentrantCallbackGroup


class SecondServer(Node):
	def __init__(self):
		super().__init__('second_fibonacci_action_server')
        #CallBackGroupに分けるのがこれ大事
        #なければReentrantCallbackGroupが必須
		cb = MutuallyExclusiveCallbackGroup()
		cb2 = MutuallyExclusiveCallbackGroup()
		self._action_server = ActionServer(
        	self,
        	Fibonacci,
        	'second_fibonacci',
        	self.execute_callback,
        	callback_group=cb2)
		self.my_client = ActionClient(self, Fibonacci, 'first_fibonacci', callback_group=cb)
		
	def execute_callback(self, goal_handle):
		self.get_logger().info('Second: Executing goal...')
		goal_msg = Fibonacci.Goal()
		goal_msg.order = 10
		self.my_client.wait_for_server()
		future = self.my_client.send_goal_async(goal_msg)
		
		
		self.executor.spin_until_future_complete(future)
		_goal_handle = future.result()
		if not _goal_handle.accepted:
			self.get_logger().info('Goal rejected :(')
			return
		self.get_logger().info('Client Goal accepted :)')
		future = _goal_handle.get_result_async()
		
		
		self.executor.spin_until_future_complete(future)
		self.get_logger().info('Client Goal finished :)')

		goal_handle.succeed()
		result = Fibonacci.Result()

		return result
			
			
		# def with_deadlock(order):
		# 	goal_msg = Fibonacci.Goal()
		# 	goal_msg.order = order
		# 	self.my_client.wait_for_server()
		# 	future = self.my_client.send_goal_async(goal_msg)

        # 	#　rclpy　spinは問題の原因になる
		# 	rclpy.spin_until_future_complete(self, future)
    	# 	# これもだめ
		# 	# rclpy.spin_until_future_complete(self, future, executor=self.executor)
		# 	goal_handle = future.result()
		# 	if not goal_handle.accepted:
		# 		self.get_logger().info('Goal rejected :(')
		# 		return  # Add a return statement here
			
		# 	self.get_logger().info('Client Goal accepted :)')
		# 	future = goal_handle.get_result_async()
			
			
		# 	rclpy.spin_until_future_complete(self, future)
		# 	self.get_logger().info('Client Goal finished :)')
		# 	return future.result().result
		
		
		def without_deadlock(order):
			goal_msg = Fibonacci.Goal()
			goal_msg.order = 10
			self.my_client.wait_for_server()
			future = self.my_client.send_goal_async(goal_msg)
			
			
			self.executor.spin_until_future_complete(future)
			goal_handle = future.result()
			if not goal_handle.accepted:
				self.get_logger().info('Goal rejected :(')
				return
			self.get_logger().info('Client Goal accepted :)')
			future = goal_handle.get_result_async()
			
			
			self.executor.spin_until_future_complete(future)
			self.get_logger().info('Client Goal finished :)')

			goal_handle.succeed()
			result = Fibonacci.Result()

			return result
		
			# result = without_deadlock(goal_handle.request.order)
    		# # result = with_deadlock(goal_handle.request.order)
			# goal_handle.succeed()
			# result = Fibonacci.Result()
			# self.get_logger().info('Second: Result: {0}'.format(result.sequence))
			# self.get_logger().info('Second: Result: {0}'.format(result.sequence))


def main(args=None):
	rclpy.init(args=args)

	node = SecondServer()
	executor = rclpy.executors.MultiThreadedExecutor()
	executor.add_node(node)
	
	try:
		executor.spin()
	except KeyboardInterrupt:
		pass


if __name__ == '__main__':
	main()
