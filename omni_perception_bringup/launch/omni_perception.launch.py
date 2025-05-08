from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('omni_perception_bringup')
    
    # 配置文件路径
    config_file = os.path.join(pkg_share, 'config', 'omni_perception_params.yaml')
    
    # 创建节点
    perception_node = Node(
        package='rm_omni_perception',
        executable='omni_perception_node',
        output='both',
        parameters=[config_file],
        remappings=[
            ('/image_raw', '/camera1/image_raw')
        ],
        arguments=['--ros-args', '--log-level', 'debug']
    )

    return LaunchDescription([
        # 启动节点
        perception_node
    ]) 