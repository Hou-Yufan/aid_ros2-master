import rclpy
from rclpy.node import Node
from aid_robot_msgs.srv import AICmd

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import cv2
import json
import time
import ctypes
import numpy as np
from multiprocessing import Process, sharedctypes

from ros_model.handpose import HandPose
from ros_model.track import Track
from ros_model.face import Face
from ros_model.detecter import Detecter

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# ros2 service call  /cam_stop aid_robot_msgs/srv/AICmd cmd:\ \'abc\'
# ros2 launch robot_bringup robot_bringup.launch.xml
class AiManagerNode(Node):
    def __init__(self):
        super().__init__("ai_manager_server")
        # AI功能操作接口
        self.create_service(AICmd, "cam_status", self.cam_status)
        self.create_service(AICmd, "cam_start", self.cam_start)
        self.create_service(AICmd, "cam_stop", self.cam_stop)
        self.create_service(AICmd, "follow_start", self.follow_start)
        self.create_service(AICmd, "follow_stop", self.follow_stop)

        # 启动开关
        self.sign_mem = sharedctypes.RawArray(ctypes.c_uint8, 1)
        self.sign = np.frombuffer(self.sign_mem, dtype=ctypes.c_uint8)

        # 深度相机相关
        self.depth_mem = sharedctypes.RawArray(ctypes.c_uint8, 921600)
        self.depth = np.frombuffer(self.depth_mem, dtype=ctypes.c_uint8).reshape(480,640,3)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
        )
        self.sub=self.create_subscription(Image, "/camera/color/image_raw", self.listener_callback, qos_profile)

        # 控制器
        self.control_publisher = self.create_publisher(Twist, "/cmd_vel", 1) # 控制器
        self.control_sign_mem = sharedctypes.RawArray(ctypes.c_float, 3)
        self.control_sign = np.frombuffer(self.control_sign_mem, dtype=ctypes.c_float)

        self.controler = self.create_timer(0.1, self.wheel_control) # 定时发布器

        self.keep_counter = 0

        # aidstream 进程维护                                                                                                                                                                                         
        try:
            import aidstream
            self.keeper = self.create_timer(1, self.aidstream_keeper)
            self.width = 1920
            self.height = 1088
        except:
            self.width = 640
            self.height = 480


        # 跟随开关
        self.follow_sign_mem = sharedctypes.RawArray(ctypes.c_uint8, 2)
        self.follow_sign = np.frombuffer(self.follow_sign_mem, dtype=ctypes.c_uint8)

        self.following = False
        self.follow_publisher = self.create_publisher(Bool, "/follow_status",1)
        timer_period = 0.2  # 定时发布的时间间隔，秒为单位
        self.timer = self.create_timer(timer_period, self.timer_callback) # 定时发布器


        self.rgbcam = ["hand", "follow", "face"]
        self.depthcam = ["object"]

    def wheel_control(self):
        if self.control_sign[0]:
            msg = Twist()
            msg.angular.z = np.float64(self.control_sign[1])
            msg.linear.x = np.float64(self.control_sign[2])
            self.control_publisher.publish(msg)

    def timer_callback(self):
        msg=Bool()
        if self.follow_sign[0]:
            self.following = True
        else:
            self.following = False
        msg.data = self.following
        self.follow_publisher.publish(msg)

    def listener_callback(self,msg):
        data = np.array(msg.data.tolist()).reshape(480,640,3)
        self.depth[:] = data


    def aidstream_keeper(self):
        if self.sign[:] == 1:
            f = open("/tmp/mmkv/info.txt","w")
            f.write(str(self.keep_counter))
            f.close()
            if self.keep_counter < 10:
                self.keep_counter += 1
            else:
                self.keep_counter = 0

    def run(self):
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        finally:
            rclpy.shutdown()

    def cam_status(self, request, response):
        # todo: 查询当前状态
        response.success = True
        response.message = ""
        return response

    def cam_start(self, request, response):
        # todo: 启动算法
        self.sign[:] = 0
        time.sleep(0.1)
        if request.cmd in self.rgbcam:
            if request.cmd == self.rgbcam[0]:
                self.sign[:] = 1
                thread = Process(target = HandPose, args = [self.sign_mem, self.control_sign_mem])
                thread.start()

            elif request.cmd == self.rgbcam[1]:
                self.sign[:] = 1
                thread = Process(target = Track, args = [self.sign_mem, self.follow_sign_mem, self.control_sign_mem])
                thread.start()

            elif request.cmd == self.rgbcam[2]:
                self.sign[:] = 1
                thread = Process(target = Face, args = [self.sign_mem, self.follow_sign_mem, self.control_sign_mem])
                thread.start()

            response.success = True
            response.message = json.dumps({"w": self.width, "h": self.height})

        elif request.cmd in self.depthcam:
            if request.cmd == self.depthcam[0]:
                self.sign[:] = 1
                thread = Process(target = Detecter, args = [self.sign_mem, self.depth_mem])
                thread.start()

            response.success = True
            response.message = json.dumps({"w": self.width, "h": self.height})

        else:
            response.success = False
            response.message = "Unknow cmd"
        return response

    def cam_stop(self, request, response):
        # todo: 停止算法
        self.sign[:] = 0

        response.success = True
        response.message = ""
        return response

    def follow_start(self, request, response):
        # todo: 开始跟随
        self.follow_sign[1] = 1
        response.success = True
        response.message = ""
        return response

    
    def follow_stop(self, request, response):
        # todo: 停止跟随
        self.follow_sign[1] = 0
        response.success = True
        response.message = ""
        return response


def main():
    rclpy.init()
    node = AiManagerNode()
    node.run()


if __name__ == "__main__":
    main()
