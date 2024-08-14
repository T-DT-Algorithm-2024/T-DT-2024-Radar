from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([

        # 播放 ROS bag 文件
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', 'config/dapeng_car','--loop'],
            output='screen'),

        # 启动第一个节点
        Node(
            package='dynamic_cloud',
            executable='dynamic_cloud_node',
            name='dynamic_cloud_node'),
        Node(
            package='track_node',
            executable='track_node',
            name='track_node'
        ),
        Node(
            package='accumulate_cloud',
            executable='accumulate_cloud',
            name='accumulate_cloud'
        )
        ,
        Node(
            package='kalman_filter',
            executable='kalman_filter',
            name='kalman_filter'
        ),
    ])
