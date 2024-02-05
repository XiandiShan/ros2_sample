# ros2_sample

## Usage
```
mkdir ~/ros2_sample_ws/src
git clone git@github.com:XiandiShan/ros2_sample.git ~/ros2_sample_ws/src/
cd ~/ros2_sample_ws && colcon build --allow-overriding action_tutorials_cpp action_tutorials_interfaces action_tutorials_py demo_nodes_py
source ~/ros2_sample_ws/install/setup.bash
```
Run action server node
```
ros2 run action_tutorials_py fibonacci_action_server
```
Run client node (Offical)
```
ros2 run action_tutorials_py fibonacci_action_client
```
