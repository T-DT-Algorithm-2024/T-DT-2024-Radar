import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory

sys.path.append(os.path.join(get_package_share_directory('tdt_vision'), 'launch'))

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import TimerAction, Shutdown
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
        
    def get_rosbag_player_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='rosbag_player_node',
            parameters=[ {'rosbag_file': 
                # '/home/tdt/rosbag/ros2bags/radar_record0531_2032_54/merged_bag/merged_bag_0.db3'
                '/home/shenxw/Rosbag/适应性录像第二把/merged_bag/merged_bag_0.db3'
                }],
            extra_arguments=[{'use_intra_process_comms': True}]
        )   
    def get_foxglove_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='foxglove_bridge_node',
            parameters=[ {'send_buffer_limit': 1000000000}],
            extra_arguments=[{'use_intra_process_comms': True},
                             {'use_multi_threaded_executor': True}]
        )
  
    def get_radar_detect_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='radar_detect_node',
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        
    def get_radar_resolve_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='radar_resolve_node',
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_camera_detector_container(radar_detect_node,radar_resolve_node,foxglove_node,ros_bag_player_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                #变向设置启动顺序
                radar_detect_node,
                radar_resolve_node,
                foxglove_node,
                ros_bag_player_node
            ],
            output='both',
            emulate_tty=True,
            on_exit=Shutdown(),
        )
    # 创建节点描述
    radar_detect_node = get_radar_detect_node('tdt_vision', 'tdt_radar::Detect')
    radar_resolve_node = get_radar_resolve_node('tdt_vision', 'tdt_radar::Resolve')
    foxglove_node = get_foxglove_node('foxglove_bridge', 'foxglove_bridge::FoxgloveBridge')
    ros_bag_player_node = get_rosbag_player_node('rosbag_player', 'RosbagPlayer')

    # 创建节点容器
    cam_detector = get_camera_detector_container(radar_detect_node,radar_resolve_node,foxglove_node,ros_bag_player_node)
    # debug_container = get_debug_container(tdt_debug_node)
    plugin_map_launch_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('tdt_vision'), 'launch', 'map_server_launch.py')]),
             )
    return LaunchDescription([
            cam_detector,
            plugin_map_launch_cmd
        ])