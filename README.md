<div align="center">

# T-DT 2024 Radar

> 2024年东北大学T-DT实验室 RoboMaster超级对抗赛 雷达代码

<a href="./LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Secrecy-yellow"></a>
<a href="https://neutdt.cn"><img alt="License" src="https://img.shields.io/badge/Home%20Page-T--DT-green"></a>

<img src=".github/NEU.webp" width=300/>
<img src=".github/t-dt.jpg" width=300/>

</dev>

<br>

--------

<br>

<div align="left">

## 1. 模块介绍

| 模块 | 说明 |
| --- | --- |
| [`lidar`](./src/lidar/) | 激光雷达模块 |
| [`camera`](./src/roborts_navigation/) | 相机模块 |
| [`interface`](./src/interface/) | 自定义消息接口 |
| [`llm_decision`](./src/robot_localization/) | 决策模块 |
| [`livox_driver`](./src/robots_perception/) | Livox驱动 |
| [`process_fusion`](./src/genshin_simulation/) | 传感器后融合模块 |
| [`utils`](./src/roborts_vision/) | 视觉模块 |
| [`roborts_decision`](./src/roborts_decision/) | 决策模块 |

## 2. 依赖

```bash
sudo apt-get install ros-humble-desktop-full python3-colcon-common-extensions libeigen3-dev libasio-dev libgoogle-glog-dev libpcl-dev ros-humble-sophus ros-humble-eigen3-cmake-module ros-humble-marti-nav-msgs libceres-dev ros-humble-libg2o ros-humble-sophus libpcg-cpp-dev libbackward-cpp-dev ros-humble-backward-ros ros-humble-gstam
sudo apt-get install gdbserver clangd lldb clang libomp-dev
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb
sudo dpkg -i cuda-keyring_1.0-1_all.deb
sudo apt-get update
sudo apt-get install libnvinfer-dev
sudo apt-get install curl
curl https://packages.neutdt.cn/ubuntu/ubuntu_setup.sh | bash
sudo apt-get install tdtenv-libosqp-eigen-dev tdtenv-libosqp-dev libtdtvision-dev, livox-sdk2
sudo apt install ros-humble-laser-proc ros-humble-urg-node-msgs ros-humble-urg-node
```
sudo 

Gazebo仿真依赖(arm64下不支持)

```bash
sudo apt-get install ros-humble-gazebo-dev ros-humble-velodyne ros-humble-velodyne-gazebo-plugins ros-humble-gazebo-plugins
```

如果您使用的是g++-12, 希望使用clang和clangd, 请确保安装了以下包:

```bash
sudo apt-get install libstdc++12-dev
```

Python脚本的依赖，以下二选一

```bash
pip3 install -r requirements.txt
```

或

```bash
sudo apt install python3-numpy python3-opencv python3-tqdm python3-json5
```

## 3. docker

### docker arm64模拟

执行以下指令

```bash
sudo apt-get install qemu-user-static
sudo docker run --rm --privileged multiarch/qemu-user-static --reset -p yes


```

## 4. 进程间通信消息名称及用途

### 4.1 ROS2 通信

#### 定位

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| cloud_registered | topic< sensor_msgs::msg::PointCloud2 > |  |
| cloud_registered_body | topic< sensor_msgs::msg::PointCloud2 > |  |
| cloud_effected | topic< sensor_msgs::msg::PointCloud2 > |  |
| Laser_map | topic< sensor_msgs::msg::PointCloud2 > |  |
| aft_mapped_to_init | topic< nav_msgs::msg::Odometry > |  |
| path | topic< nav_msgs::msg::Path > |  |
| planner_normal | topic< visualization_msgs::msg::Marker > |  |

...... 等待定位负责人完善

#### 导航

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| navigation2usart | topic< vision_interface::msg::Navigation2Usart > | 将导航控制信息发给串口 |
| navigation_debug | topic< vision_interface::msg::NavigationDebug > | 将导航Debug信息反馈给导航主线程 |
| navigation2_debug | topic< vision_interface::msg::Navigation2Debug > | 将导航信息发送给Debug程序 |
| NavMap2Debug | serve< vision_interface::srv::Nav2DebugMap > | 用于Debug程序获得导航地图图片 |

...... 等待其它模块负责人完善

### 4.2 黑板 通信

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| vision_mode | Memory< int > | 视觉当前处于装甲板模式还是大符模式 |
| match_info | Memory< tdttoolkit::MatchInfo > | 当前比赛的实时信息 |

## 5. 工具

可用VSCode使用Ctrl+Shift+B使用常见编译任务(需安装工作区推荐插件)，例如下方指令为编译单个包指令，已集成进tasks.json

```bash
colcon build --packages-select 功能包名称
```

已实现VSCode下使用gdbserver或lldb-server进行程序调试的配置文件，按下F5即可使用，需安装相应软件

</div>
