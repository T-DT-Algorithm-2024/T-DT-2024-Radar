import os
import sys
import yaml
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import TimerAction, Shutdown
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch

def dump_params(param_file_path, node_name):
    with open(param_file_path, 'r') as file:
        return [yaml.safe_load(file)[node_name]['ros__parameters']]

def generate_launch_description():
    
    def get_localization_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='localization_node',
            extra_arguments=[{'use_intra_process_comms': True},
                             {'use_multi_threaded_executor': True}],
        )
        
    def get_dynamic_cloud_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='dynamic_cloud_node',
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        
    def get_tracker_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='tracker_node',
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        
    def get_kalman_filter_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='kalman_filter_node',
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        
    def get_foxglove_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='foxglove_bridge_node',
            parameters=[ {'send_buffer_limit': 1000000000}],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_container(*nodes):
        # 打印当前路径
        print(os.getcwd())
        return ComposableNodeContainer(
            name='lidar_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=list(nodes),
            output='both',
            emulate_tty=True,
            on_exit=Shutdown(),
        )
        
    
    # 创建节点描述
    localization_node = get_localization_node('localization', 'tdt_radar::Localization')
    dynamic_cloud_node = get_dynamic_cloud_node('dynamic_cloud', 'tdt_radar::DynamicCloud')
    tracker_node = get_tracker_node('tracker', 'tdt_radar::Tracker')
    kalman_filter_node = get_kalman_filter_node('kalman_filter', 'tdt_radar::KalmanFilter')
    # foxglove_node = get_foxglove_node('foxglove_bridge', 'foxglove_bridge::FoxgloveBridge')

    # 创建节点容器
    lidar_detector = get_container(
                                    localization_node,
                                    dynamic_cloud_node,
                                    tracker_node,
                                    kalman_filter_node
                                    # foxglove_node
                                   )
    # cmd = launch.actions.ExecuteProcess(cmd=['ros2', 'bag', 'play', 'config/merged_bag/merged_bag_0.db3', '--loop', '--start-offset', '250'])

    return LaunchDescription([
            lidar_detector
            ])