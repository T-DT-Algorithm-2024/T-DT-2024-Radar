import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.conditions import IfCondition


def generate_launch_description():
    ld = LaunchDescription()
    map_path = 'src/tdt_vision/maps/map.yaml'

    map_server_node = Node(
        package="nav2_map_server",
        executable='map_server',
        output='screen', emulate_tty=True,
        parameters=[{'yaml_filename': map_path,'frame_id':'rm_frame'}]
    )
    # 激活map_server
    activate_map_server = ExecuteProcess(
        cmd=['sh', '-c',
             ' echo "指令激活 map_server" && \
                  ros2 lifecycle set /map_server configure && ros2 lifecycle set /map_server activate && \
                  echo "激活 map_server 完成"'
             ],
        name='configure_map_server',
        output='screen'
    )
    ld.add_action(map_server_node)
    ld.add_action(activate_map_server)

    return ld
