# 1 The launch_manager_node package is use to manage ros launch 

## How to use it

- build: 
``` 
colcon build --packages-select aid_robot_py 
```
- run: 
``` 
ros2 run  aid_robot_py launch_manager_node 
```

## Example: 

### start launch
```
ros2 service call /start_launch aid_robot_msgs/srv/ControlLaunch launch_file:\ \'/home/box/robot_sim_ws/install/turtlebot3_cartographer/share/turtlebot3_cartographer/launch/cartographer.launch.py\'

response:
aid_robot_msgs.srv.ControlLaunch_Response(success=True, message='Started /home/box/robot_sim_ws/install/turtlebot3_cartographer/share/turtlebot3_cartographer/launch/cartographer.launch.py')
or:
aid_robot_msgs.srv.ControlLaunch_Response(success=false, message='/home/box/robot_sim_ws/install/turtlebot3_cartographer/share/turtlebot3_cartographer/launch/cartographer.launch.py is already running')

```


### stop launch

```
ros2 service call /stop_launch aid_robot_msgs/srv/ControlLaunch launch_file:\ \'/home/box/robot_sim_ws/install/turtlebot3_cartographer/share/turtlebot3_cartographer/launch/cartographer.launch.py\'


response:
aid_robot_msgs.srv.ControlLaunch_Response(success=True, message='Stopped /home/box/robot_sim_ws/install/turtlebot3_cartographer/share/turtlebot3_cartographer/launch/cartographer.launch.py')

or:
response:
aid_robot_msgs.srv.ControlLaunch_Response(success=False, message='/home/box/robot_sim_ws/install/turtlebot3_cartographer/share/turtlebot3_cartographer/launch/cartographer.launch.py is not running')

```

### get launch status

```
ros2 service call /query_launch_status aid_robot_msgs/srv/ControlLaunch launch_file:\ \'/home/box/robot_sim_ws/install/turtlebot3_cartographer/share/turtlebot3_cartographer/launch/cartographer.launch.py\'

response:
aid_robot_msgs.srv.ControlLaunch_Response(success=True, message='running')
or
response:
aid_robot_msgs.srv.ControlLaunch_Response(success=True, message='exited')
or
response:
aid_robot_msgs.srv.ControlLaunch_Response(success=True, message='not found')

```

# 2 The map_manager_node package is use to manage map

- build: 
``` 
colcon build --packages-select aid_robot_py 
```
- run: 
``` 
ros2 run  aid_robot_py map_manager_node 
```