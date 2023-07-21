import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.publisher import Publisher
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowWaypoints
from std_msgs.msg import Empty
from std_msgs.msg import String


class PatrolPlanner(Node):

    def __init__(self):
        super().__init__('patrol_planner')

        # 创建发布者，发布 geometry_msgs/PoseStamped topic
        self.publisher_ = self.create_publisher(String, 'patrol_pose', 10)

        # 创建客户端，等待 navigation2 的服务启动
        self.client_ = ActionClient(self, FollowWaypoints, 'FollowWaypoints')

        # 创建订阅者，订阅 std_msgs/Empty topic
        self.subscription_ = self.create_subscription(
            Empty, 'stop_patrol', self.stop_callback, 10)

        # 订阅处理 geometry_msgs/PoseStamped topic
        self.subscription_ = self.create_subscription(
            Path, 'patrol_path', self.topic_callback, 10)

        self.send_goal_future = None
        # 等待 navigation2 的服务启动
        while not self.client_.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for the action server to start...')

    def stop_callback(self, msg):
        self.get_logger().info('Goal cancel stop_callback.')
        # 接收到停止指令时，取消当前 goal 并停止定时器
        if self.send_goal_future is not None:
            cancel_future = self.send_goal_future.result().cancel_goal_async()
            self.get_logger().info('Goal cancelled.')

    def topic_callback(self, msg):
        # 接收到 nav_msgs/msg/Path topic 时，将消息打印到终端
        self.get_logger().info(f'Received path with {len(msg.poses)} poses')


        # 向 navigation2 发送消息，控制机器人执行巡逻
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = msg.poses
        self.send_goal_future = self.client_.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.missed_waypoints))
        

    def feedback_callback(self, feedback_msg):
        # 接收到 feedback 消息时，将消息打印到终端
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback:{feedback.current_waypoint}')

    def destroy(self):
        # 关闭客户端
        self.client_.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = PatrolPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
