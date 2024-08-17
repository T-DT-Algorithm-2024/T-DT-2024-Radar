<div align="center">

# T-DT 2024 Radar

> 2024年东北大学T-DT实验室 RoboMaster超级对抗赛 雷达代码

<a href="./LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Secrecy-yellow"></a>
<a href="https://neutdt.cn"><img alt="License" src="https://img.shields.io/badge/Home%20Page-T--DT-green"></a>

<p align="center">
  <img src=".github/NEU.jpg" width="300"/>
  
  <img src=".github/T-DT.jpg" width="300"/>
</p>


</dev>

<br>

--------

<br>

<div align="left">

# 版本和发布记录

### 0.0.1 当前版本

v0.0.1beta
初始化仓库，加入基础内容，初版README

# 项目介绍
通过激光雷达目标检测和单目相机目标检测，进行传感器后融合，实现了传感器之间的完全解耦合，完全避免了联合标定带来的误差，同时开发难度不随传感器数量增加而增加。~~(如果和你关系好的队伍不幸被淘汰了,可以把相机/雷达直接借过来用。)~~

**如果你没有激光雷达，也可以直接使用本项目的单目相机方案 (在RM2023的0.6m误差规则下取得了最高91%的准确率,荣获2023年雷达MVP)**
## 硬件条件
- 激光雷达 Livox Avia
- 单目相机 Hikvision CH-120-10UC
- CPU i7-12700KF
- GPU RTX A4000 * 2
## 项目结构说明
本项目提供了除串口、相机驱动、模型训练外雷达站的全部功能

### 单目相机:五点标定(键盘微调), 透视变换方案

### 识别:三层神经网络结构
| 名称 | 大小 | 用途 |
| --- | --- | --- |
| yolov5s | 1280x1280 | 识别机器人 |
| yolov5s | 192x192 | 识别装甲板 |
| resnet18 | 224x224 | 数字分类 |

三层网络实现了更好的鲁棒性和可修复性，极大地降低了模型训练和数据集整理的难度和时间，

其中第二层和第三层网络使用多batch推理，速度提升很大

RTX A4000 实测50Hz

RTX 3050M(35W极致阉割版) 实测16Hz

由于使用了时间同步，所以只要推理速度>10Hz 也能正常使用
### 激光雷达
ICP配准+KdTree离群点检测+欧几里得聚类
## 1. 模块介绍

| 模块 | 说明 |
| --- | --- |
| [`lidar`](./src/lidar/) | 激光雷达模块 |
| [`camera`](./src/tdt_vision/) | 相机模块（无相机驱动） |
| [`interface`](./src/interface/) | 自定义消息接口 |
| [~~`llm_decision`~~](./src/llm_decision/) | ~~大模型决策模块~~ |
| [`livox_driver`](./src/livox_driver/) | Livox驱动 |
| [`fusion`](./src/fusion/) | 传感器后融合模块 |
| [`utils`](./src/工具包/) | 视觉模块 |

## 2. 依赖

```bash
Ubuntu 22.04
ROS2 (Humble)
CUDA+CUDNN+TensorRT(8)
OpenCV
PCL
Livox_SDK(1)
```
如果您使用的是g++-12, 希望使用clang和clangd, 请确保安装了以下包:

```bash
sudo apt-get install libstdc++12-dev
```

## 3. 进程间通信消息名称及用途

### 3.1 ROS2 通信 （注意QoS）

#### 激光雷达

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| livox/lidar | topic< sensor_msgs::msg::PointCloud2 > | Livox驱动接口 |
| livox/map | topic< sensor_msgs::msg::PointCloud2 > | 3D地图可视化 |
| livox/lidar_dynamic | topic< sensor_msgs::msg::PointCloud2 > | 动态点云 |
| livox/cluster | topic< sensor_msgs::msg::PointCloud2 > | 聚类结果 |
| livox/lidar_detect | topic< vision_interface::msg::RadarWarn > | 激光雷达预警 |



#### 相机

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| camera_image | topic< sensor_msgs::msg::Image > | 相机驱动接口 |
| detect_result | topic< vision_interface::msg::DetectResult > | 识别结果 |
| resolve_result | topic< vision_interface::msg::DetectResult > | 解算结果 |


#### 传感器融合

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| kalman_detect | topic<vision_interface::msg::DetectResult> | 卡尔曼节点输出 |
| match_info | topic<vision_interface::msg::MatchInfo > | 当前比赛的实时信息 |
| Radar2Sentry | topic<vision_interface::msg::Radar2Sentry> | 发送给串口的最终结果 |
## 4. 工具

可用VSCode使用Ctrl+Shift+B使用常见编译任务(需安装工作区推荐插件)，例如下方指令为编译单个包指令，已集成进tasks.json

```bash
colcon build --packages-select 功能包名称
```

已实现VSCode下使用gdbserver或lldb-server进行程序调试的配置文件，按下F5即可使用，需安装相应插件
## 5. 测试
    
```bash
ros2 launch tdt_vision run_rosbag.launch.py #通过rosbag启动相机
ros2 launch dynamic_cloud lidar.launch.py #启动激光雷达识别
ros2 run debug_map debug_map #启动地图可视化
ros2 launch livox_ros2_driver livox_lidar_launch.py #启动Livox驱动
```
测试ros2bag下载
[百度网盘](https://pan.baidu.com/s/1ogRvs3v1OMCVUbAlUsOGQA?pwd=52rm)

修改tdt_vision/launch对应的launch文件中的rosbag路径即可进程内播放对应的rosbag
</div>
