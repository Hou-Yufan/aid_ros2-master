import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

class TransformPublisher(Node):
    def __init__(self):
        super(TransformPublisher,self).__init__('transform_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/base_link_pose', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node=self)
        self.timer_ = self.create_timer(0.1, self.publish_transformed_pose)

    def publish_transformed_pose(self):
        try:
            # 创建一个PointStamped消息，其中header中的frame_id为map，point中的坐标为(1, 0, 0)


            # 将point_in坐标从map坐标系转换到base_link坐标系

            now = rclpy.time.Time()
            transform_out = self.tf_buffer.lookup_transform(
                        'map',
                        'base_link',
                        now)
            # 创建一个Pose消息，其中位置和方向分别为point_out坐标系中的值
            pose_out = PoseStamped()
            pose_out.header.frame_id = 'base_link'

            pose_out.pose.position.x = transform_out.transform.translation.x
            pose_out.pose.position.y = transform_out.transform.translation.y
            pose_out.pose.position.z = transform_out.transform.translation.z

            pose_out._pose.orientation.x = transform_out.transform.rotation.x
            pose_out._pose.orientation.y = transform_out.transform.rotation.y
            pose_out._pose.orientation.z = transform_out.transform.rotation.z
            pose_out._pose.orientation.w = transform_out.transform.rotation.w

            # 将转换后的Pose消息发布到话题上
            self.publisher_.publish(pose_out)
        except Exception as e:
            str(e)
            #self.get_logger().error('Failed to lookup transform: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)

    transform_publisher = TransformPublisher()

    rclpy.spin(transform_publisher)

    transform_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
