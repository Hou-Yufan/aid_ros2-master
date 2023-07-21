import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
import base64


class MapTransform(Node):

    def __init__(self):
        super().__init__('map_transform')
        self.subscription = self.create_subscription(OccupancyGrid, '/map',
                                                     self.callback, 1)
        self.publisher = self.create_publisher(OccupancyGrid, '/map_base64', 1)

    def callback(self, msg):
        # Convert data field to 2D numpy array
        data = msg.data
        width = msg.info.width
        height = msg.info.height
        array2d = [[0 for x in range(width)] for y in range(height)]
        for i in range(height):
            for j in range(width):
                index = i * width + j
                h = height - i - 1
                if (data[index] == -1):
                    array2d[h][j] = 100
                else:
                    array2d[h][j] = (100 - data[index]) * 2

        # Convert 2D numpy array to grayscale image
        img = cv2.cvtColor(np.array(array2d, dtype=np.uint8),
                           cv2.COLOR_GRAY2BGR)
        img[np.where((img == [100, 100, 100]).all(axis=2))] = [170, 108, 82]
        img[np.where(((img > [100, 100, 100]) &
                      (img <= [150, 150, 150])).all(axis=2))] = [185, 125, 100]
        img[np.where((img > [150, 150, 150]).all(axis=2))] = [200, 145, 127]

        # Convert image to JPEG format
        retval, buffer = cv2.imencode('.jpg', img)
        jpeg_data = buffer.tobytes()

        # Convert JPEG data to base64 format
        base64_data = base64.b64encode(jpeg_data)

        # Convert base64_data to sequence of int8
        int_array = np.frombuffer(base64_data, dtype=np.uint8)

        # Create new OccupancyGrid message
        send_msg = OccupancyGrid()
        send_msg.header = msg.header
        send_msg.info = msg.info

        # Convert int8 numpy array to Python list
        send_msg.data = int_array.tolist()

        # Publish new message
        self.publisher.publish(send_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapTransform()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
