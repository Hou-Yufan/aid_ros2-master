import base64
import cv2
import json
import math
import numpy as np
import os
import rclpy
import time
import yaml
from aid_robot_msgs.srv import MapImage
from aid_robot_msgs.srv import MapLinkedDataList
from aid_robot_msgs.srv import MapList
from aid_robot_msgs.srv import MapOperationAdd
from aid_robot_msgs.srv import OperationAdd
from aid_robot_msgs.srv import OperationDelete
from aid_robot_msgs.srv import OperationGet
from aid_robot_msgs.srv import OperationUpdate
from aid_robot_msgs.srv import ForbiddenGet
from aid_robot_msgs.srv import GetCurrentForbidden
from aid_robot_msgs.srv import SetCurrentMap, GetCurrentMap
from aid_robot_msgs.srv import GetCurrentForbidden
from aid_robot_msgs.msg import StartToEndPoint as ForbiddenLine
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, Path, MapMetaData
from rclpy.node import Node

from .db import SQLite


class MapManagerNode(Node):
    def __init__(self, db_file_path="/root/maps/db.sqlite"):
        super().__init__("map_manager_server")
        print(f"db file is {db_file_path}")
        self.conn = SQLite(db_file_path)
        # self.declare_parameter("map_manager_server")
        # self.map_path = self.get_parameter("map_path").get_parameter_value().string_array_value

        # 地图操作
        self.create_service(MapOperationAdd, "add_map", self.add_map_callback)
        self.create_service(OperationDelete, "delete_map", self.delete_data_callback)
        self.create_service(OperationUpdate, "update_map", self.update_data_callback)
        self.create_service(MapList, "get_map_list", self.get_map_list_callback)
        self.create_service(MapImage, "get_map_image", self.get_map_image_callback)
        self.create_service(SetCurrentMap, "set_current_map_id", self.set_current_map_callback)
        self.create_service(GetCurrentMap, "get_current_map_id", self.get_current_map_callback)

        # 位置点操作
        self.create_service(OperationGet, "get_waypoint", self.get_data_callback)
        self.create_service(OperationAdd, "add_waypoint", self.add_data_callback)
        self.create_service(OperationDelete, "delete_waypoint", self.delete_data_callback)
        self.create_service(OperationUpdate, "update_waypoint", self.update_data_callback)
        self.create_service(MapLinkedDataList, "get_map_waypoint_list", self.get_data_list_callback)

        # 禁行线操作
        self.create_service(ForbiddenGet, "get_forbidden", self.get_forbidden_callback)
        self.create_service(OperationAdd, "add_forbidden", self.add_data_callback)
        self.create_service(OperationDelete, "delete_forbidden", self.delete_data_callback)
        self.create_service(OperationUpdate, "update_forbidden", self.update_data_callback)
        self.create_service(MapLinkedDataList, "get_map_forbidden_list", self.get_data_list_callback)
        self.create_service(GetCurrentForbidden, "get_current_forbidden", self.get_current_forbidden)

    def set_current_map_callback(self, request, response):
        # 设置为当前地图接口
        try:
            #print("set_current_map in") # debug print
            sql_clear_current_map = "delete from current_map"
            self.conn.execSQL(sql_clear_current_map)
            #print("delete current") # debug print

            map_id = request.id

            sql_get_map_file = "select file_path, name from map where id=?"
            result = self.conn.execSQL(sql_get_map_file, (map_id,))
            #print("get map file path") # debug print

            sql_insert_current_map = "insert into current_map(id, file_path, name) values(?, ?, ?)"
            self.conn.execSQL(sql_insert_current_map, (map_id, result[0]["file_path"], result[0]["name"]))
            response.success = True
            #print("set_current_map ok") # debug print
        except Exception as e:
            # todo: 记录详细错误信息到日志
            print(e)

            response.success = False
        return response

    def get_current_map_callback(self, request, response):
        # 请求当前地图接口
        try:
            #print("get_current_map in") # debug print
            sql_get_current = "select id, file_path, name from current_map"
            result = self.conn.execSQL(sql_get_current)
        except Exception as e:
            #print("get_current_map error") # debug print
            response.success = False
            response.map_id = 0
            response.map_file = ""
            response.map_name = ""
        else:
            response.success = len(result) > 0
            if response.success:
                #print("get_current_map ok") # debug print
                _ = result[0]
                response.map_id = _.get("id", 0)
                response.map_file = _.get("file_path", "")
                response.map_name = _.get("name", "")
            else:
                #print("no current map") # debug print
                response.map_id = 0
                response.map_file = ""
                response.map_name = ""
        return response

    def add_map_callback(self, request, response):
        #print("add_map in") # debug print
        try:
            map_name = request.map_name.strip()
            file_path = request.map_file.strip()
            creator_id = 0
            sql_check_map_name = "select 1 from map where name=? and creator_id=?"
            result = self.conn.execSQL(sql_check_map_name, (map_name, creator_id))
            if len(result) > 0:
                # print("add_map failed") # debug print
                response.success = False
                response.message = "map name duplicated"
            else:
                # 判断新增的地图文件是否存在
                if os.path.exists(f"{file_path}.yaml"):
                    sql_add_map = "insert into map(name, file_path, create_timestamp, edit_timestamp, creator_id) values(?, ?, ?, ?, ?)"
                    now = int(time.time())
                    self.conn.execSQL(sql_add_map, (map_name, file_path, now, now, creator_id))
                    response.success = True
                    response.message = "ok"
                    #print("add_map ok") # debug print
                else:
                    # print("add_map not exists") # debug print
                    response.success = False
                    response.message = "map file not exists"
        except Exception as e:
            # todo: 记录详细错误信息到日志
            print(e)

            response.success = False
            response.message = "error"
            #print("add_map error") # debug print
        return response

    def get_map_image_callback(self, request, response):
        #print("get_map_image in") # debug print
        # 默认状态 / 没找到地图记录 / 查询出现问题，都是以下值作为返回值
        response.success = False
        response.map = OccupancyGrid()
        response.map_file = ""
        # 通过地图id，查找地图文件路径
        map_id = (request.id,)
        try:
            sql_query_map_file = "select file_path from map where id=?"
            result = self.conn.execSQL(sql_query_map_file, map_id)
        except Exception as e:
            # todo: 记录详细错误信息到日志
            print(e)
            #print("get_map_image error") # debug print
        else:
            print(f"select map:{result}, {len(result)}")
            if len(result) > 0:
                print("get_map_image in2") # debug print
                _ = result[0]
                map_yaml_file = "{}.yaml".format(_["file_path"])
                response.success = os.path.exists(map_yaml_file)
                response.map_file = _["file_path"]
                if response.success:
                    # 打开 filepath，并转成 nav_msgs/OccupancyGrid 类型
                    self.__map_pgm_to_grid(map_yaml_file, response.map)
                    #print("get_map_image ok") # debug print
            print("get_map_image out") # debug print
        return response

    def get_map_list_callback(self, request, response):
        #print("get_map_list in") # debug print
        # 查询地图列表
        try:
            sql_query_map_list = "select id, name, create_timestamp from map where creator_id=0"
            result = self.conn.execSQL(sql_query_map_list)

            response.success = len(result) > 0
            if response.success:
                response.map_list = json.dumps(result)
                #print("get_map_list ok") # debug print

            else:
                response.map_list = json.dumps([])
                #print("get_map_list no map") # debug print
        except Exception as e:
            #print("get_map_list error") # debug print
            # todo: 记录详细错误信息到日志
            print(e)

            response.success = False
            response.map_list = json.dumps([])

        return response

    def get_data_callback(self, request, response):
        #print("get_data in") # debug print
        data_id = (request.id,)
        sql_get_data = f"select id, frame_id, point_list from {request.data_type} where id=?"
        try:
            result = self.conn.execSQL(sql_get_data, data_id)
            response.success = len(result) > 0
            if response.success:
                #print("get_data ok") # debug print
                _ = result[0]
                response.message = self.__point_list_to_path(_["frame_id"], json.loads(_["point_list"]))

            else:
                #print("get_data no data") # debug print
                response.message = None  # "waypoint not found"
        except Exception as e:
            #print("get_data error") # debug print
            # todo: 记录详细错误信息到日志
            print(e)

            response.success = False
            response.message = None

        return response

    def add_data_callback(self, request, response):
        # print("add_data in") # debug print
        map_id = request.map_id
        try:
            sql_add_data = f"insert into {request.data_type}(map_id, point_list, frame_id, create_timestamp, edit_timestamp, creator_id) values (?, ?, ?, ?, ?, ?)"
            now = int(time.time())
            point_list = request.data
            print(f"request.data {request.data} type {type(request.data)}")
            print(f"request.frame_id {request.frame_id} type {type(request.frame_id)}")
            self.conn.execSQL(sql_add_data, (map_id, point_list, request.frame_id, now, now, 0))
            # print(4)
            response.success = True
            # print("add_data ok") # debug print
            response.message = "ok"
        except Exception as e:
            # print("add_data error") # debug print
            # todo: 记录详细错误信息到日志
            print(e)

            response.success = False
            response.message = "error"
        return response

    def delete_data_callback(self, request, response):
        #print("delete_data in") # debug print
        data_id = (request.id,)
        try:
            if request.data_type == "map":
                #print("delete_data in map") # debug print
                # 如果是删除 map，则同步删除map对应的其他数据
                sql_get_map_file = "select file_path from map where id=?"
                result = self.conn.execSQL(sql_get_map_file, data_id)
                if len(result):
                    # 删除地图对应的定位点记录
                    sql_delete_waypoint = "delete from waypoint where map_id=?"
                    self.conn.execSQL(sql_delete_waypoint, data_id)
                    #print("delete_data 1") # debug print
                    # 删除地图对应的禁行线记录
                    sql_delete_forbidden = "delete from forbidden where map_id=?"
                    self.conn.execSQL(sql_delete_forbidden, data_id)
                    #print("delete_data 2") # debug print

                    sql_delete_current_map = "delete from current_map where id=?"
                    self.conn.execSQL(sql_delete_current_map, data_id)
                    #print("delete_data 3") # debug print

                    # 删除文件系统里对应地图文件
                    rm_cmd = f"rm -f {result[0]['file_path']}.*"
                    print(rm_cmd)
                    os.system(rm_cmd)

            sql_delete_data = f"delete from {request.data_type} where id=?"
            self.conn.execSQL(sql_delete_data, data_id)
            response.success = True
            response.message = "ok"
            #print("delete_data ok") # debug print
        except Exception as e:
            #print("delete_data error") # debug print
            print(e)
            response.success = False
            response.message = "error"
        return response

    def update_data_callback(self, request, response):
        #print("update_data in") # debug print
        try:
            sql_get_data = f"select 1 from {request.data_type} where id=?"
            result = self.conn.execSQL(sql_get_data, (request.id,))
            response.success = len(result) > 0
            if response.success:
                data = json.loads(request.data)
                sql_keys = ["edit_timestamp=?"]
                sql_values = [int(time.time())]
                print(f"data {data}")
                for k, v in data.items():
                    sql_keys.append(f"{k}=?")
                    sql_values.append(v)
                    print(f"k {k}, v {v}, type(v) {type(v)}")
                sql_values.append(request.id)  # id 这个一定要放在更新值列表之后因为是最后作为 where 子句的条件存在
                print(f"sql_values {sql_values}")
                update_data_str = ",".join(sql_keys)  # 将字段更新列表拼接成字符串
                sql_update_data = f"update {request.data_type} set {update_data_str} where id=?"
                print(f"sql_update_data {sql_update_data}")

                self.conn.execSQL(sql_update_data, sql_values)
                response.message = "ok"
                #print("update_data ok") # debug print
            else:
                #print("update_data no data") # debug print
                response.message = "data not found"
        except Exception as e:
            #print("update_data error") # debug print
            # todo: 记录详细错误信息到日志
            print(e)

            response.success = False
            response.message = "error"
        return response

    def get_data_list_callback(self, request, response):
        #print("get_data_list in") # debug print
        map_id = (request.map_id, 0)
        try:
            sql_get_data_list = f"select id, frame_id, point_list from {request.data_type} where map_id=? and creator_id=?"
            result = self.conn.execSQL(sql_get_data_list, map_id)
            response.success = len(result) > 0
            if response.success:
                #print("get_data_list ok") # debug print
                response.message = json.dumps(result)
            else:
                #print("get_data_list no data") # debug print
                response.message = json.dumps([])
        except Exception as e:
            # todo: 记录详细错误信息到日志
            print(e)

            response.success = False
            response.message = "error"
            #print("get_data_list error") # debug print
        return response

    def get_forbidden_callback(self, request, response):
        data_id = (request.map_id,)
        sql_get_data = f"select id, frame_id, point_list from forbidden where map_id=?"
        print(f"get_forbidden_callback cmd: {sql_get_data}")
        try:
            result = self.conn.execSQL(sql_get_data, data_id)
            response.success = len(result) > 0
            if response.success:
                print(f"get_forbidden_callback ok {result} type of result {type(result)}")
                for forbidden_item in result:
                    # 防止地图的禁止线数据又错误的空字符串的情况，所以增加一个检查，避免在转换json对象时出错导致失败
                    if len(forbidden_item['point_list']) == 0:
                        continue
                    pl = json.loads(forbidden_item['point_list'])
                    print(f"pl is {pl}")

                    if isinstance(pl, list) and len(pl):
                        response.message = self.__point_list_to_forbiddenlines(pl)
                        print("get_forbidden_callback done")
                        break
            else:
                print("get_forbidden_callback no data") # debug print
                response.success = True
                response.message = []  # "forbidden not found"
        except Exception as e:
            print("get_forbidden_callback error") # debug print
            # todo: 记录详细错误信息到日志
            print(e)

            response.success = False
            response.message = []

        return response

    def get_current_forbidden(self, request, response):
        try:
            sql_get_current_map = "select id from current_map"
            result = self.conn.execSQL(sql_get_current_map)
            if len(result) > 0:
                map_id = (result[0].get("id", 0), )
                sql_get_current_forbidden = "select id, frame_id, point_list from forbidden where map_id=?"
                print(f"get_current_forbidden_callback cmd: {sql_get_current_forbidden}, map_id={map_id}")

                result = self.conn.execSQL(sql_get_current_forbidden, map_id)
                response.success = len(result) > 0
                if response.success:
                    print(f"get_current_forbidden_callback ok {result} type of result {type(result)}")
                    for forbidden_item in result:
                        # 防止地图的禁止线数据又错误的空字符串的情况，所以增加一个检查，避免在转换json对象时出错导致失败
                        if len(forbidden_item['point_list']) == 0:
                            continue
                        pl = json.loads(forbidden_item['point_list'])
                        print(f"pl is {pl}")

                        if isinstance(pl, list) and len(pl):
                            response.message = self.__point_list_to_forbiddenlines(pl)
                            print("get_forbidden_callback done")
                            break
                else:
                    print("get_current_forbidden_callback no data") # debug print
                    response.message = []  # "forbidden not found"
            else:
                raise Exception("current map not set")

        except Exception as e:
            print("get_current_forbidden error") # debug print
            # todo: 记录详细错误信息到日志
            print(e)
            response.success = False
            response.message = []

        return response

    def run(self):
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        finally:
            rclpy.shutdown()

    def __map_pgm_to_grid(self, map_file_path: str, msg: OccupancyGrid):
        with open(map_file_path, "r", encoding="utf-8") as f:
            map_yaml_data = yaml.load(f, Loader=yaml.FullLoader)

        print(f"map_yaml_data\n{map_yaml_data}")

        map_img_file_path = map_yaml_data["image"]

        # 读取pgm文件
        np_array2d = cv2.imread(map_img_file_path, -1)

        # Convert 2D numpy array to grayscale image
        img = cv2.cvtColor(np_array2d, cv2.COLOR_GRAY2BGR)

        img[np.where((img == [205, 205, 205]).all(axis=2))] = [170, 108, 82]
        img[np.where((img == [254, 254, 254]).all(axis=2))] = [200, 145, 127]

        # Convert image to JPEG format
        retval, buffer = cv2.imencode(".jpg", img)
        jpeg_data = buffer.tobytes()

        # Convert JPEG data to base64 format
        base64_data = base64.b64encode(jpeg_data)

        # Convert base64_data to sequence of int8
        int_array = np.frombuffer(base64_data, dtype=np.uint8)

        # Create new OccupancyGrid message
        msg.header.frame_id = map_yaml_data.get("frame_id", "map")
        msg.header.stamp = self.get_clock().now().to_msg()
        info = MapMetaData()
        info.map_load_time = msg.header.stamp
        info.resolution = map_yaml_data.get("resolution", 0.0)
        if len(img.shape) == 3:
            height, width, channels = img.shape
        else:
            height, width = img.shape
        info.width = width
        info.height = height
        origin = Pose()
        orientation = Quaternion()
        orientation.w = float(math.sin(map_yaml_data.get("origin")[2] / 2))
        orientation.x = 0.0
        orientation.y = 0.0
        orientation.z = float(math.sin(map_yaml_data.get("origin")[2] / 2))
        origin.orientation = orientation
        position = Point()
        position.x = float(map_yaml_data.get("origin")[0])
        position.y = float(map_yaml_data.get("origin")[1])
        # position.z = map_yaml_data.get("origin")[2] # 暂时写死为0.0
        position.z = 0.0
        origin.position = position
        info.origin = origin
        msg.info = info
        # Convert int8 numpy array to Python list
        msg.data = int_array.tolist()
        return msg

    def __point_list_to_path(self, frame_id: str, point_list: list):
        msg = Path()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for point in point_list:
            pose = PoseStamped()
            position = point["position"]
            pose.pose.position.x = float(position["x"])
            pose.pose.position.y = float(position["y"])
            pose.pose.position.z = float(position["z"])
            orientation = point["orientation"]
            pose.pose.orientation.x = float(orientation["x"])
            pose.pose.orientation.y = float(orientation["y"])
            pose.pose.orientation.z = float(orientation["z"])
            pose.pose.orientation.w = float(orientation["w"])
            msg.poses.append(pose)
        return msg

    def __path_to_point_list(self, path: Path):
        point_list = []
        for pose in path.poses:
            point = {
                "position": {
                    "x": pose.pose.position.x,
                    "y": pose.pose.position.y,
                    "z": pose.pose.position.z
                },
                "orientation": {
                    "x": pose.pose.orientation.x,
                    "y": pose.pose.orientation.y,
                    "z": pose.pose.orientation.z,
                    "w": pose.pose.orientation.w
                }
            }
            point_list.append(point)
        return point_list

    def __point_list_to_forbiddenlines(self, point_list: list):
        msg = []
        for point_pair in point_list:
            forbiddenline = ForbiddenLine()
            start = point_pair["start"]
            end = point_pair["end"]
            forbiddenline.start.x = float(start['x'])
            forbiddenline.start.y = float(start['y'])
            forbiddenline.start.z = float(start['z'])
            forbiddenline.end.x = float(end['x'])
            forbiddenline.end.y = float(end['y'])
            forbiddenline.end.z = float(end['z'])
            msg.append(forbiddenline)
        return msg

    def __forbiddenlines_to_point_list(self, forbiddenlines: list):
        point_list = []
        for forbiddenline in forbiddenlines:
            item = {
                "start": {"x": 0.0, "y":0.0, "z":0.0},
                "end": {"x": 0.0, "y":0.0, "z":0.0}
            }
            item["start"]["x"] = float(forbiddenline.start.x)
            item["start"]["y"] = float(forbiddenline.start.y)
            item["start"]["z"] = float(forbiddenline.start.z)
            item["end"]["x"] = float(forbiddenline.end.x)
            item["end"]["y"] = float(forbiddenline.end.y)
            item["end"]["z"] = float(forbiddenline.end.z)
            point_list.append(item)
        return point_list


def main():
    rclpy.init()
    node = MapManagerNode()
    node.run()


if __name__ == "__main__":
    main()
