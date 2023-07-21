import sys
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.srv import ManageLifecycleNodes

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from aid_robot_msgs.srv import StatusChange
from aid_robot_msgs.srv import ControlLaunch


class MapManagerNode(Node):
    def __init__(self):
        super().__init__('status_manager_server')

        self.create_service(StatusChange, 'mode_set', self.mode_set_callback)
        
        self.stop_launch_client = self.create_client(ControlLaunch, 'stop_launch')

        self.maping_launch_file = '/root/robot_sim_ws/install/nav2_bringup/share/nav2_bringup/launch/navigation_launch.py'
        self.sensor_launch_file = '/root/robot_sim_ws/install/robot_bringup/share/robot_bringup/launch/robot_bringup.launch.xml'
        

    def start_launch(self,file):
        req = ControlLaunch.Request()
        req.launch_file = file
        req.parameter = ''
        node = rclpy.create_node('start_launch_node')
        start_launch_client = node.create_client(ControlLaunch, 'start_launch')
        while not start_launch_client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('service start_launch not available, waiting again...')
        future = start_launch_client.call_async(req)
        rclpy.spin_until_future_complete(node, future, None, 1.0)
        if future.result() is not None:
            response = future.result()
            print('Result: %s' % (response.message))
            return True
        else:
            print('Service call failed %r' % (future.exception(),))
            return False

    def stop_launch(self,file):
        req = ControlLaunch.Request()
        req.launch_file = file
        req.parameter = ''
        node = rclpy.create_node('stop_launch_node')
        stop_launch_client = node.create_client(ControlLaunch, 'stop_launch')
        while not stop_launch_client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('service start_launch not available, waiting again...')
        future = stop_launch_client.call_async(req)
        rclpy.spin_until_future_complete(node, future, None, 1.0)
        if future.result() is not None:
            response = future.result()
            print('Result: %s' % (response.message))
            return True
        else:
            print('Service call failed %r' % (future.exception(),))
            return False

    def mode_set_callback(self, request, response):
        action = request.action
        # add code find map and delete
        if action == "start_mapping":
            if self.start_launch(self.maping_launch_file) == True:
                response.message = "ok"
            else:
                response.message = "false"
        if action == "stop_mapping":
            if self.stop_launch(self.maping_launch_file) == True:
                response.message = "ok"
            else:
                response.message = "false"
        if action == "start_sensor":
            if self.start_launch(self.sensor_launch_file) == True:
                response.message = "ok"
            else:
                response.message = "false"
        if action == "stop_sensor":
            if self.stop_launch(self.sensor_launch_file) == True:
                response.message = "ok"
            else:
                response.message = "false"
        return response


    def run(self):
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        finally:
            rclpy.shutdown()

def main():
    rclpy.init()
    node = MapManagerNode()
    node.run()


if __name__ == '__main__':
    main()
