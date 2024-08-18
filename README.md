<div align="center">

# T-DT 2024 Radar

> 2024年东北大学T-DT实验室 RoboMaster超级对抗赛 雷达代码

<a href="./LICENSE"><img alt="License" src="https://img.shields.io/badge/License-MIT-yellow"></a>
<a href="https://neutdt.cn"><img alt="Home Page" src="https://img.shields.io/badge/Home%20Page-T--DT-green"></a>

<p align="center">
  <!-- <img src=".github/NEU.jpg" width="300"/> -->

  <img src=".github/T-DT.jpg" width="300"/>
</p>

</div>

<br>

--------

<br>

<div align="left">

# 版本和发布记录

### 当前版本

**v0.0.1beta**
- 初始化仓库，加入基础内容，初版README

# 项目介绍

本项目通过激光雷达和单目相机的目标检测，进行传感器后融合，实现了传感器之间的完全解耦合，避免了联合标定带来的误差，同时开发难度不随传感器数量增加而增加。~~(如果和你关系好的队伍不幸被淘汰了，可以把相机/雷达直接借过来用。)~~

**如果你没有激光雷达，也可以直接使用本项目的单目相机方案 (在RM2023的0.6m误差规则下取得了最高91%的准确率，荣获2023年雷达MVP)**
## 项目优势
- 1.即插即用，不依赖联合标定，脱离空间(机械结构)上的限制
- 2.不依赖相机和雷达之间的帧间匹配，脱离时间上的限制
- 3.直接使用直角坐标系的信息，更加直观
- 4.三层神经网络实现了更好的鲁棒性和可修复性，极大地降低了模型训练和数据集整理的难度和时间。
- 5.低耦合，易于维护和扩展
- 6.雷达全自动配准，节约3分钟部署时间
## 硬件条件

- 激光雷达 Livox Avia
- 单目相机 Hikvision CH-120-10UC
- CPU i7-12700KF
- GPU RTX A4000 * 2

## 项目结构说明

本项目提供了除串口、相机驱动、模型训练外雷达站的全部功能

### 单目相机

- 五点标定(键盘微调)
- 透视变换方案

### 识别

- 三层神经网络结构

| 名称 | 大小 | 用途 |
| --- | --- | --- |
| yolov5s | 1280x1280 | 识别机器人 |
| yolov5s | 192x192 | 识别装甲板 |
| resnet18 | 224x224 | 数字分类 |

建议根据相机分辨率调整模型大小，以提高推理速度。
- RTX A4000 实测50Hz
- RTX 3050M (35W极致阉割版) 实测16Hz

由于使用了时间同步，只要推理速度>10Hz 也能正常使用。

**模型存储在 model/ONNX 文件夹下**
- 提供了onnx自动转换trt，如果没有检测到TensorRT编译的模型，会自动编译对应模型。

### 激光雷达

- ICP配准
- KdTree离群点检测
- 欧几里得聚类
- 飞镖检测
- 空中机器人检测

### 传感器融合

- 使用卡尔曼滤波器对激光雷达识别到的目标进行跟踪，同时将相机识别结果向卡尔曼轨迹进行匹配，最后融合卡尔曼滤波器结果和相机识别结果，输出最终结果。

### 工具包
- 进程内播放rosbag (ros2 jazzy已支持)

## 模块介绍

| 模块 | 说明 |
| --- | --- |
| [`lidar`](./src/lidar/) | 激光雷达模块 |
| [`camera`](./src/tdt_vision/) | 相机模块（无相机驱动） |
| [`interface`](./src/interface/) | 自定义消息接口 |
| [~~`llm_decision`~~](./src/llm_decision/) | ~~大模型决策模块~~ |
| [`livox_driver`](./src/livox_driver/) | Livox驱动 |
| [`fusion`](./src/fusion/) | 传感器后融合模块 |
| [`utils`](./src/工具包/) | 视觉模块 |

## 依赖

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

## 进程间通信消息名称及用途

### 1. ROS2 通信 （注意QoS）

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
## 工具

可用VSCode使用Ctrl+Shift+B使用常见编译任务(需安装工作区推荐插件)，例如下方指令为编译单个包指令，已集成进tasks.json

```bash
colcon build --packages-select 功能包名称
```

已实现VSCode下使用gdbserver或lldb-server进行程序调试的配置文件，按下F5即可使用，需安装相应插件
## 测试
    
```bash
ros2 launch tdt_vision run_rosbag.launch.py #通过rosbag启动相机
ros2 launch dynamic_cloud lidar.launch.py #启动激光雷达识别
ros2 run debug_map debug_map #启动地图可视化
ros2 launch livox_ros2_driver livox_lidar_launch.py #启动Livox驱动
```
测试ros2bag下载
[百度网盘](https://pan.baidu.com/s/1ogRvs3v1OMCVUbAlUsOGQA?pwd=52rm)

修改tdt_vision/launch对应的launch文件中的rosbag路径即可进程内播放对应的rosbag
## 可视化
Launch文件已集成foxglove-bridge,启动后直接打开foxglove-studio即可查看
## TODO
多相机/雷达从当前逻辑(结构)上可以实现，但是并没有进行对应ros2接口的适配，后续更新
# 联系方式
Email: shenxuewen0127@gmail.com

QQ: 2738226430
</div>
