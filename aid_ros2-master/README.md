# Aidros Docker配置与镜像创建

[CF] 2023.4.20.byVG

*硬件环境为Firefly-AIO3588Q 8+64G，机器人机体为Aidbot-Homekit*

## 1. 系统配置

### 1.1 系统环境安装

- 参考<Aidbot-docker异构系统安装配置>

### 1.2 网络配置

- 在宿主机配置网络，yaml文件在git中下载（Jade-rabbit-robot/aid_ros2/robot_bringup/script）

  ```
  sudo cp 01-netcfg.yaml /etc/netplan
  sudo netplan apply
  ```
  
## 2. Ros 容器创建

- 拉取 ros-foxy 版本镜像文件

  ```
  docker pull arm64v8/ros:foxy
  ```
  
- 拉起一个容器

  ```
  docker run -it -v /dev:/dev --net=host --privileged --name=aid-ros arm64v8/ros:foxy
  ```

- 测试串口通讯，如果没有报错再继续进行安装配置

  ```
  echo "test..." > /dev/ttyS0
  ```
  
## 3. Aidros环境配置

*以下操作在刚刚创建的docker内进行*

### 3.1  Install ROS 2 packages
```
apt update
apt install python3-argcomplete ros-dev-tools
```
### 3.2  安装图形驱动
- 安装GLX库
	```
	apt install -y libgl1-mesa-glx libgl1-mesa-dri libglx-mesa0
	```
- 更新libqt5opengl5-dev
	```
	sed -i 's/.*wiki.t-firefly.com.*/\#&/' /etc/apt/sources.list
	apt install libqt5opengl5-dev
	sed -i '/.*wiki.t-firefly.com.*/s/^#//' /etc/apt/sources.list
	```

### 3.3 安装传感器依赖与相关库

- 安装依赖

	```
	apt install libudev-dev libgflags-dev nlohmann-json3-dev 
	ros-foxy-image-transport ros-foxy-image-publisher
	```
- 安装glog
    ```
	wget -c https://github.com/google/glog/archive/refs/tags/v0.6.0.tar.gz -O glog- 0.6.0.tar.gz
	tar -xzvf glog-0.6.0.tar.gz
	cd glog-0.6.0
	mkdir build && cd build
	cmake .. && make -j4
	make install
	ldconfig 
	```

- 安装magic_enum

	```
	wget -c https://github.com/Neargye/magic_enum/archive/refs/tags/v0.8.0.tar.gz -O  magic_enum-0.8.0.tar.gz
	tar -xzvf magic_enum-0.8.0.tar.gz
	cd magic_enum-0.8.0
	mkdir build && cd build
	cmake .. && make -j4
	make install
	ldconfig
	```
	
- 安装libusb

	```
	git clone -b v1.0.26 https://github.com/libusb/libusb
  cd libusb
  ./autogen.sh && make -j4
  make install
  ldconfig
  ```
- 安装libuvc
    ```
    git clone https://github.com/libuvc/libuvc.git
    cd libuvc
    mkdir build && cd build
    cmake .. && make -j4
    make install
    ldconfig 
    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib" >> ~/.bashrc
    ```

### 3.4 安装功能包

- 安装pcl

  ```
  apt install ros-foxy-pcl-ros
  ```
-  安装cartographer
  ```
  apt install ros-foxy-cartographer-ros
  ```
-  安装rosbridge
  ```
   apt install ros-foxy-rosbridge-server
  ```
-  安装键盘遥控
  ```
  apt install ros-foxy-teleop-twist-keyboard
  ```

## 4. Aidros部署

### 4.1 下载代码

``` 
mkdir -p ~/aid_ros_ws/src && cd ~/aid_ros_ws/src
git clone https://github.com/Jade-rabbit-robot/aid_ros2.git
```

### 4.2 配置与编译

- 复制数据库文件

  ```
  cd ~/aid_ros_ws/src/aid_ros2/aid_robot_py/aid_robot_py && cp db.sql ~/aid_ros_ws/
  ```

- 编译

  ```
  cd ~/aid_ros_ws
  source /opt/ros/foxy/setup.bash
  colcon build --cmake-args  -DCMAKE_BUILD_TYPE=Release
  ```

### 4.3 测试功能

- 配置启动文件

  ```
  echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
  echo "source ~/aid_ros_ws/install/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```

- 启动Aidbot

  ```
  ros2 launch robot_bringup robot_bringup.launch.xml
  ```

  **启动成功再进行下一步，可以结合前端图形界面对各项功能进行测试**

## 5. 保存容器为镜像与导出

*退出容器到宿主系统*

### 5.1 保存容器为镜像

- 查询ros容器ID

  ```
  docker ps -a
  ```

- 保存容器为镜像，[CONTAINER ID] = 刚刚查询的容器ID

  ```
  docker commit -a "aidlux.com" -m "Aidbot-Homekit" [CONTAINER ID] aidlux/ros:foxy
  ```

- 查看保存好的镜像

  ```
  docker images
  ```

### 5.2 打包导出镜像

```
docker save -o aidlux.ros.homekit.tar aidlux/ros:foxy
```

## Docker基本操作

- 退出容器（容器内）

  ```
  exit
  ctrl + D
  ```

- 容器启动关闭

  ```
  docker start aid-ros
  docker stop aid-ros
  ```

- 进入容器

  ```
  docker attach aid-ros
  ```
  
  
