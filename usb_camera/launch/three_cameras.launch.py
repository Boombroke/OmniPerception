from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取配置文件路径
    config_file = os.path.join(
        get_package_share_directory('usb_camera'),
        'config',
        'camera_params.yaml'
    )
    print(f"Loading config from: {config_file}")

    # 创建三个相机节点
    camera1_node = Node(
        package='usb_camera',
        executable='usb_camera_node',
        name='camera1',
        parameters=[config_file],
        # namespace='camera1',
        output='screen'
    )

    camera2_node = Node(
        package='usb_camera',
        executable='usb_camera_node',
        name='camera2',
        parameters=[config_file],
        # namespace='camera2',
        output='screen'
    )

    camera3_node = Node(
        package='usb_camera',
        executable='usb_camera_node',
        name='camera3',
        parameters=[config_file],
        # namespace='camera3',
        output='screen'
    )

    return LaunchDescription([
        camera1_node,
        camera2_node,
        camera3_node
    ]) 